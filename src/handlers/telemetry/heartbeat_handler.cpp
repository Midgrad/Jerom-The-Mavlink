#include "heartbeat_handler.h"

#include <QDebug>
#include <QJsonArray>
#include <QTimerEvent>

#include "mavlink_protocol_helpers.h"
#include "mode_helper_factory.h"
#include "vehicle_tmi.h"

using namespace md::domain;

namespace
{
constexpr int onlineTimout = 2000;

constexpr char mavId[] = "mavId";

Vehicle::Type decodeMavType(uint8_t type)
{
    switch (type)
    {
    case MAV_TYPE_FIXED_WING:
    case MAV_TYPE_KITE:
    case MAV_TYPE_FLAPPING_WING:
        return Vehicle::FixedWing;
    case MAV_TYPE_TRICOPTER:
    case MAV_TYPE_QUADROTOR:
    case MAV_TYPE_HEXAROTOR:
    case MAV_TYPE_OCTOROTOR:
        return Vehicle::Copter;
    case MAV_TYPE_COAXIAL:
    case MAV_TYPE_HELICOPTER:
        return Vehicle::RotaryWing;
    case MAV_TYPE_VTOL_DUOROTOR:
    case MAV_TYPE_VTOL_QUADROTOR:
    case MAV_TYPE_VTOL_TILTROTOR:
    case MAV_TYPE_VTOL_RESERVED2:
    case MAV_TYPE_VTOL_RESERVED3:
    case MAV_TYPE_VTOL_RESERVED4:
    case MAV_TYPE_VTOL_RESERVED5:
        return Vehicle::Vtol;
    case MAV_TYPE_AIRSHIP:
    case MAV_TYPE_FREE_BALLOON:
        return Vehicle::Airship;
    case MAV_TYPE_GENERIC:
    default:
        return Vehicle::Generic;
    }
}

std::string decodeState(uint8_t state)
{
    switch (state)
    {
    case MAV_STATE_BOOT:
        return "Preparing";
    case MAV_STATE_CALIBRATING:
        return "Calibrating";
    case MAV_STATE_STANDBY:
        return "Standby";
    case MAV_STATE_ACTIVE:
        return "Active";
    case MAV_STATE_CRITICAL:
        return "Critical";
    case MAV_STATE_EMERGENCY:
        return "Emergency";
    case MAV_STATE_POWEROFF:
        return "Poweroff";
    case MAV_STATE_FLIGHT_TERMINATION:
        return "Flight Termination";
    case MAV_STATE_UNINIT:
    default:
        return "Unknown";
    }
}
} // namespace

HeartbeatHandler::HeartbeatHandler(MavlinkHandlerContext* context,
                                   IVehiclesService* vehiclesService,
                                   ICommandsService* commandsService, QObject* parent) :
    AbstractCommandHandler(context, parent),
    m_vehiclesService(vehiclesService)
{
    connect(vehiclesService, &IVehiclesService::vehicleRemoved, this, [this](Vehicle* vehicle) {
        if (!m_vehicleTimers.contains(vehicle))
            delete m_vehicleTimers.take(vehicle);
    });

    commandsService->requestCommand(tmi::setMode)
        ->subscribe(
            [this](const QVariant& target, const QVariantList& args) {
                if (!args.isEmpty())
                    this->sendMode(target, args.first().toString());
            },
            this);

    commandsService->requestCommand(tmi::setArmed)
        ->subscribe(
            [this](const QVariant& target, const QVariantList& args) {
                if (!args.isEmpty())
                    this->sendArm(target, args.first().toBool());
            },
            this);
}

HeartbeatHandler::~HeartbeatHandler()
{
    for (QBasicTimer* timer : m_vehicleTimers.values())
    {
        delete timer;
    }
}

void HeartbeatHandler::parse(const mavlink_message_t& message)
{
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
        this->processHeartbeat(message);
}

void HeartbeatHandler::sendMode(const QVariant& vehicleId, const QString& mode)
{
    qDebug() << "setMode" << vehicleId << mode;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    auto modeHelper = m_modeHelpers.value(mavId);
    if (!modeHelper)
        return;

    mavlink_message_t message;
    mavlink_set_mode_t setMode;

    setMode.target_system = mavId;
    setMode.base_mode = m_baseModes.value(mavId, 0);
    setMode.custom_mode = modeHelper->modeToCustomMode(mode);

    mavlink_msg_set_mode_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                     &setMode); // TODO: link channel
    emit sendMessage(message);
}

void HeartbeatHandler::sendArm(const QVariant& vehicleId, bool arm)
{
    qDebug() << "setArm" << vehicleId << arm;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    if (arm)
    {
        // 21196: force arming/disarming (e.g. allow arming to override preflight checks)
        int force = 0;
        this->sendCommandLong(mavId, MAV_CMD_COMPONENT_ARM_DISARM, { 1, force, 0, 0, 0, 0, 0 }, 0);
    }
    else
    {
        this->sendCommandLong(mavId, MAV_CMD_COMPONENT_ARM_DISARM, { 0, 0, 0, 0, 0, 0, 0 }, 0);
    }
}

void HeartbeatHandler::processHeartbeat(const mavlink_message_t& message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&message, &heartbeat);

    // Ignore generic heartbeat
    auto type = ::decodeMavType(heartbeat.type);
    if (type == Vehicle::Generic)
        return;

    // Get or create vehicle
    Vehicle* vehicle = m_vehiclesService->vehicle(m_context->vehicleIds.value(message.sysid));

    // TODO: todo with request vehicle with mavId
    for (Vehicle* vehicleCandidate : m_vehiclesService->vehicles())
    {
        quint8 mavIdandidate = vehicleCandidate->parameter(::mavId).toUInt();
        if (mavIdandidate == message.sysid)
        {
            vehicle = vehicleCandidate;
            m_context->vehicleIds.insert(message.sysid, vehicle->id);
        }
    }

    // Create new vehicle on heartbeat from unknown mav
    if (!vehicle)
    {
        vehicle = new Vehicle(type, QString("MAV %1").arg(message.sysid));
        vehicle->setParameter(::mavId, message.sysid);
        m_vehiclesService->saveVehicle(vehicle);
        m_context->vehicleIds.insert(message.sysid, vehicle->id);
        emit vehicleObtained(vehicle);
    }

    // Vehicle timer and base mode
    if (!m_vehicleTimers.contains(vehicle))
    {
        m_vehicleTimers[vehicle] = new QBasicTimer();
    }
    m_vehicleTimers[vehicle]->start(::onlineTimout, this); // TODO: to settings
    m_baseModes[message.sysid] = heartbeat.base_mode;

    // Telemetry values
    QVariantMap properties(
        { { tmi::state, QString::fromStdString(::decodeState(heartbeat.system_status)) },
          { tmi::armed, (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) },
          { tmi::online, true } });

    // Obtain specific mode with mode helper
    if (!m_modeHelpers.contains(message.sysid))
        m_modeHelpers.insert(message.sysid,
                             QSharedPointer<data_source::IModeHelper>(
                                 data_source::ModeHelperFactory::create(heartbeat.autopilot,
                                                                        heartbeat.type)));
    auto modeHelper = m_modeHelpers.value(message.sysid);
    if (modeHelper)
    {
        properties.insert(tmi::modes, QJsonArray::fromStringList(modeHelper->availableModes()));
        properties.insert(tmi::mode, modeHelper->customModeToMode(heartbeat.custom_mode));
    }

    m_context->pTree->appendProperties(vehicle->id.get().toString(), properties);
}

void HeartbeatHandler::timerEvent(QTimerEvent* event)
{
    for (QBasicTimer* timer : m_vehicleTimers.values())
    {
        if (timer->timerId() != event->timerId())
            continue;

        Vehicle* vehicle = m_vehicleTimers.key(timer);
        if (vehicle)
            m_context->pTree->appendProperties(vehicle->id.get().toString(),
                                               { { tmi::online, false } });

        timer->stop();
    }
}

import QtQuick 2.12
import QtQuick.Layouts 1.3
import Industrial.Controls 1.0 as Controls

Preflight {
    id: root

    Controls.DelayButton {
        width: parent.width
        flat: true
        enabled: selectedVehicle && selectedVehicle.online
        fillColor: Controls.Theme.colors.negative
        borderColor: Controls.Theme.colors.border
        text: params.armed ? qsTr("Disarm throttle"): qsTr("Arm throttle")
        onActivated: dashboardController.sendCommand("setArmed", [ !params.armed ])
        Layout.fillWidth: true
    }
}

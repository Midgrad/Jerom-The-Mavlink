import QtQuick 2.6
import Industrial.Controls 1.0 as Controls
import Industrial.Indicators 1.0 as Indicators
import Dreka 1.0

Preflight {
    id: root

    Controls.DelayButton {
        width: parent.width
        flat: true
        enabled: selectedVehicle && selectedVehicle.online
        fillColor: Controls.Theme.colors.negative
        borderColor: Controls.Theme.colors.border
        text: params.armed ? qsTr("Disarm throttle"): qsTr("Arm throttle")
        onActivated: controller.sendCommand("setArmed", [ !params.armed ])
    }
}

// ui/qml/DetectScreen.qml
// Home screen: fullscreen camera preview + 3 nav buttons + status bar.
// "Track" button is disabled until backend.calibrationComplete is true.
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: detectScreen
    anchors.fill: parent

    property string pendingSystemAction: ""

    Component.onCompleted: {
        backend.stopTrackingIfActive()
        backend.requestStatus()
    }

    // --- Camera preview ---
    Image {
        id: preview
        anchors.fill: parent
        // source changes every time FrameProvider increments frameCounter.
        source: "image://frame/" + backend.frameCounter
        fillMode: Image.PreserveAspectCrop
        cache: false
        // Black background while no frame is available
        Rectangle {
            anchors.fill: parent
            color: "#0e1117"
            visible: preview.status !== Image.Ready
        }
        Text {
            anchors.centerIn: parent
            text: "Geen signaal"
            color: "#aaaaaa"
            font.pixelSize: 24
            visible: preview.status !== Image.Ready
        }
    }

    Column {
        id: powerButtons
        anchors {
            left: parent.left
            bottom: statusBar.top
            leftMargin: 24
            bottomMargin: 16
        }
        spacing: 12

        Rectangle {
            width: 180
            height: 48
            radius: 8
            color: rebootArea.containsMouse ? "#b7791f" : "#d29922"
            border.color: "#d29922"
            border.width: 1
            Behavior on color { ColorAnimation { duration: 100 } }

            Text {
                anchors.centerIn: parent
                text: "Herstart"
                color: "#0e1117"
                font.pixelSize: 17
                font.weight: Font.Bold
            }

            MouseArea {
                id: rebootArea
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    detectScreen.pendingSystemAction = "reboot"
                    systemDialog.open()
                }
            }
        }

        Rectangle {
            width: 180
            height: 48
            radius: 8
            color: shutdownArea.containsMouse ? "#a21d2e" : "#cf222e"
            border.color: "#cf222e"
            border.width: 1
            Behavior on color { ColorAnimation { duration: 100 } }

            Text {
                anchors.centerIn: parent
                text: "Afsluiten"
                color: "#ffffff"
                font.pixelSize: 17
                font.weight: Font.Bold
            }

            MouseArea {
                id: shutdownArea
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    detectScreen.pendingSystemAction = "shutdown"
                    systemDialog.open()
                }
            }
        }
    }

    // --- Navigation buttons (bottom-right cluster) ---
    Column {
        id: navButtons
        anchors {
            right:  parent.right
            bottom: statusBar.top
            rightMargin:  24
            bottomMargin: 16
        }
        spacing: 12

        // Instellingen
        Rectangle {
            width: 180; height: 52
            radius: 8
            color: settingsArea.containsMouse ? "#30363d" : "#21262d"
            border.color: "#30363d"; border.width: 1
            Behavior on color { ColorAnimation { duration: 100 } }

            Text {
                anchors.centerIn: parent
                text: "Instellingen"
                color: "#ffffff"
                font.pixelSize: 18
                font.weight: Font.Medium
            }
            MouseArea {
                id: settingsArea
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: root.navigateTo(Qt.resolvedUrl("SettingsScreen.qml"))
            }
        }

        // Kalibreer
        Rectangle {
            width: 180; height: 52
            radius: 8
            color: calibArea.containsMouse ? "#30363d" : "#21262d"
            border.color: "#30363d"; border.width: 1
            Behavior on color { ColorAnimation { duration: 100 } }

            Text {
                anchors.centerIn: parent
                text: "Kalibreer"
                color: "#ffffff"
                font.pixelSize: 18
                font.weight: Font.Medium
            }
            MouseArea {
                id: calibArea
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: root.navigateTo(Qt.resolvedUrl("CalibrateScreen.qml"))
            }
        }

        Rectangle {
            width: 180; height: 52
            radius: 8
            color: debugArea.containsMouse ? "#30363d" : "#21262d"
            border.color: "#30363d"; border.width: 1
            Behavior on color { ColorAnimation { duration: 100 } }

            Text {
                anchors.centerIn: parent
                text: "Detectie debug"
                color: "#ffffff"
                font.pixelSize: 18
                font.weight: Font.Medium
            }
            MouseArea {
                id: debugArea
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: root.navigateTo(Qt.resolvedUrl("DetectorDebugScreen.qml"))
            }
        }

        // Track — disabled until calibration is complete
        Rectangle {
            width: 180; height: 52
            radius: 8
            color: {
                if (!backend.calibrationComplete) return "#161b22"
                return trackArea.containsMouse ? "#1a7f3c" : "#2ea44f"
            }
            border.color: backend.calibrationComplete ? "#2ea44f" : "#30363d"
            border.width: 1
            opacity: backend.calibrationComplete ? 1.0 : 0.45
            Behavior on color   { ColorAnimation { duration: 100 } }
            Behavior on opacity { NumberAnimation { duration: 100 } }

            Text {
                anchors.centerIn: parent
                text: "Track"
                color: "#ffffff"
                font.pixelSize: 18
                font.weight: Font.Bold
            }
            MouseArea {
                id: trackArea
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: backend.calibrationComplete ? Qt.PointingHandCursor : Qt.ForbiddenCursor
                enabled: backend.calibrationComplete
                onClicked: {
                    root.navigateTo(Qt.resolvedUrl("TrackScreen.qml"))
                }
            }
        }
    }

    Dialog {
        id: systemDialog
        modal: true
        width: Math.min(parent.width - 64, 420)
        x: Math.round((parent.width - width) / 2)
        y: Math.round((parent.height - implicitHeight) / 2)
        padding: 20
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        background: Rectangle {
            radius: 12
            color: "#161b22"
            border.color: "#30363d"
            border.width: 1
        }

        contentItem: Column {
            spacing: 16

            Text {
                text: detectScreen.pendingSystemAction === "reboot" ? "Systeem herstarten?" : "Systeem afsluiten?"
                color: "#ffffff"
                font.pixelSize: 22
                font.weight: Font.Medium
            }

            Text {
                width: systemDialog.width - systemDialog.leftPadding - systemDialog.rightPadding
                wrapMode: Text.WordWrap
                color: "#aaaaaa"
                font.pixelSize: 14
                text: detectScreen.pendingSystemAction === "reboot"
                    ? "Tracking wordt eerst gestopt. Daarna herstart de Raspberry Pi netjes via systemd."
                    : "Tracking wordt eerst gestopt. Daarna sluit de Raspberry Pi netjes af via systemd."
            }
        }

        footer: Item {
            implicitHeight: footerRow.implicitHeight + 20

            RowLayout {
                id: footerRow
                anchors {
                    left: parent.left
                    right: parent.right
                    bottom: parent.bottom
                    leftMargin: 20
                    rightMargin: 20
                    bottomMargin: 20
                }
                spacing: 12

                Item { Layout.fillWidth: true }

                Button {
                    text: "Annuleer"
                    onClicked: systemDialog.close()
                }

                Button {
                    text: detectScreen.pendingSystemAction === "reboot" ? "Herstart" : "Sluit af"
                    onClicked: {
                        systemDialog.close()
                        if (detectScreen.pendingSystemAction === "reboot")
                            backend.rebootSystem()
                        else if (detectScreen.pendingSystemAction === "shutdown")
                            backend.shutdownSystem()
                    }
                }
            }
        }
    }

    // --- Status bar (bottom) ---
    Rectangle {
        id: statusBar
        anchors {
            left:   parent.left
            right:  parent.right
            bottom: parent.bottom
        }
        height: 36
        color:  "#161b22"
        opacity: 0.88

        RowLayout {
            anchors {
                verticalCenter: parent.verticalCenter
                left:  parent.left
                right: parent.right
                leftMargin:  16
                rightMargin: 16
            }
            spacing: 24

            // Status message from backend
            Text {
                text: backend.statusMsg.length > 0 ? backend.statusMsg
                    : backend.socketConnected ? "Verbonden"
                    : "Wachten op backend…"
                color: "#aaaaaa"
                font.pixelSize: 13
                elide: Text.ElideRight
                Layout.fillWidth: true
            }

            // FreeD Hz
            Text {
                text: backend.freedEnabled ? "FreeD: " + backend.freedHz.toFixed(1) + " Hz" : "FreeD: uit"
                color: !backend.freedEnabled ? "#8b949e"
                    : (backend.freedHz > 25 ? "#2ea44f" : (backend.freedHz > 0 ? "#d29922" : "#aaaaaa"))
                font.pixelSize: 13
            }

            // IMU status dot
            Row {
                spacing: 6
                Rectangle {
                    width: 10; height: 10
                    radius: 5
                    anchors.verticalCenter: parent.verticalCenter
                    color: !backend.imuEnabled ? "#8b949e" : (backend.imuOk ? "#2ea44f" : "#cf222e")
                }
                Text {
                    text: backend.imuEnabled ? "IMU" : "IMU uit"
                    color: backend.imuEnabled ? "#aaaaaa" : "#8b949e"
                    font.pixelSize: 13
                    anchors.verticalCenter: parent.verticalCenter
                }
            }

            // Socket connectivity dot
            Row {
                spacing: 6
                Rectangle {
                    width: 10; height: 10
                    radius: 5
                    anchors.verticalCenter: parent.verticalCenter
                    color: backend.socketConnected ? "#2ea44f" : "#cf222e"
                }
                Text {
                    text: "Backend"
                    color: "#aaaaaa"
                    font.pixelSize: 13
                    anchors.verticalCenter: parent.verticalCenter
                }
            }

            // Calibration indicator
            Row {
                spacing: 6
                Rectangle {
                    width: 10; height: 10
                    radius: 5
                    anchors.verticalCenter: parent.verticalCenter
                    color: backend.calibrationComplete ? "#2ea44f" : "#d29922"
                }
                Text {
                    text: "Kalibr."
                    color: "#aaaaaa"
                    font.pixelSize: 13
                    anchors.verticalCenter: parent.verticalCenter
                }
            }
        }
    }
}

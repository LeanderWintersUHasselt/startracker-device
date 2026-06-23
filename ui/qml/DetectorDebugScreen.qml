import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: debugScreen
    anchors.fill: parent

    Component.onCompleted: {
        backend.stopTrackingIfActive()
        backend.requestStatus()
        backend.setDetectorDebugMode("off")
    }

    Component.onDestruction: {
        backend.setDetectorDebugMode("off")
    }

    Rectangle {
        anchors.fill: parent
        color: "#0e1117"
    }

    Image {
        id: preview
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            bottom: controls.top
            margins: 20
        }
        source: "image://frame/" + backend.frameCounter
        fillMode: Image.PreserveAspectFit
        cache: false

        Rectangle {
            anchors.fill: parent
            color: "#0e1117"
            visible: preview.status !== Image.Ready
        }

        Text {
            anchors.centerIn: parent
            visible: preview.status !== Image.Ready
            text: backend.detectorDebugMode === "off"
                ? "Kies een detector om de debug-grid te tonen"
                : "Wachten op preview…"
            color: "#aaaaaa"
            font.pixelSize: 24
        }
    }

    Rectangle {
        anchors {
            left: preview.left
            right: preview.right
            top: preview.top
        }
        height: 88
        radius: 12
        color: "#161b22"
        opacity: 0.92

        Column {
            anchors {
                fill: parent
                margins: 16
            }
            spacing: 6

            Text {
                text: backend.detectorDebugMode === "normal"
                    ? "Normale detector: raw, value, enhanced, mask, accepted, overlay"
                    : backend.detectorDebugMode === "light"
                        ? "Light detector: raw, gray, threshold, all contours (G=ok R=area B=circ), accepted, overlay"
                        : "Selecteer een detector. De preview toont maximaal 6 relevante stappen van raw tot overlay."
                color: "#ffffff"
                font.pixelSize: 20
                font.weight: Font.Medium
            }

            Text {
                text: "Gebruik dit scherm om stationaire jitter in de detecties rechtstreeks te vergelijken."
                color: "#aaaaaa"
                font.pixelSize: 14
            }
        }
    }

    Rectangle {
        id: controls
        anchors {
            left: parent.left
            right: parent.right
            bottom: parent.bottom
        }
        height: 128
        color: "#161b22"
        opacity: 0.96

        RowLayout {
            anchors {
                fill: parent
                leftMargin: 20
                rightMargin: 20
                topMargin: 18
                bottomMargin: 18
            }
            spacing: 14

            Rectangle {
                Layout.preferredWidth: 220
                Layout.fillHeight: true
                radius: 10
                color: backend.detectorDebugMode === "normal"
                    ? "#2ea44f"
                    : (normalArea.containsMouse ? "#30363d" : "#21262d")
                border.color: backend.detectorDebugMode === "normal" ? "#2ea44f" : "#30363d"
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "Normale detector"
                    color: "#ffffff"
                    font.pixelSize: 18
                    font.weight: Font.Medium
                }

                MouseArea {
                    id: normalArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: backend.setDetectorDebugMode("normal")
                }
            }

            Rectangle {
                Layout.preferredWidth: 220
                Layout.fillHeight: true
                radius: 10
                color: backend.detectorDebugMode === "light"
                    ? "#2ea44f"
                    : (lightArea.containsMouse ? "#30363d" : "#21262d")
                border.color: backend.detectorDebugMode === "light" ? "#2ea44f" : "#30363d"
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "Light detector"
                    color: "#ffffff"
                    font.pixelSize: 18
                    font.weight: Font.Medium
                }

                MouseArea {
                    id: lightArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: backend.setDetectorDebugMode("light")
                }
            }

            Rectangle {
                Layout.preferredWidth: 140
                Layout.fillHeight: true
                radius: 10
                color: backend.detectorDebugMode === "off"
                    ? "#d29922"
                    : (offArea.containsMouse ? "#30363d" : "#21262d")
                border.color: backend.detectorDebugMode === "off" ? "#d29922" : "#30363d"
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "Uit"
                    color: "#ffffff"
                    font.pixelSize: 18
                    font.weight: Font.Medium
                }

                MouseArea {
                    id: offArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: backend.setDetectorDebugMode("off")
                }
            }

            Item { Layout.fillWidth: true }

            Rectangle {
                Layout.preferredWidth: 150
                Layout.fillHeight: true
                radius: 10
                color: backArea.containsMouse ? "#30363d" : "#21262d"
                border.color: "#30363d"
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "Terug"
                    color: "#ffffff"
                    font.pixelSize: 18
                    font.weight: Font.Medium
                }

                MouseArea {
                    id: backArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: root.navigateBack()
                }
            }
        }
    }
}

// ui/qml/main.qml
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

Window {
    id: root
    visible: true
    width:  Screen.width
    height: Screen.height
    title:  "StarTracker"
    color:  "#0e1117"

    // Dark theme palette, imported into child screens via root.xxx
    readonly property color bgColor:    "#0e1117"
    readonly property color panelColor: "#161b22"
    readonly property color textPrim:   "#ffffff"
    readonly property color textSec:    "#aaaaaa"
    readonly property color accentOk:   "#2ea44f"
    readonly property color accentErr:  "#cf222e"
    readonly property color accentWarn: "#d29922"
    readonly property color btnNormal:  "#21262d"
    readonly property color btnHover:   "#30363d"

    property var keyboardTarget: null
    property bool keyboardShift: false

    // Navigation helpers used by child screens
    function navigateTo(url) { mainLoader.source = url }
    function navigateBack()  { mainLoader.source = Qt.resolvedUrl("DetectScreen.qml") }

    function isTextInput(item) {
        if (!item) return false
        try {
            return item.text !== undefined && item.cursorPosition !== undefined
        } catch (e) {
            return false
        }
    }

    function insertKeyboardText(value) {
        if (!isTextInput(keyboardTarget)) return
        keyboardTarget.forceActiveFocus()
        keyboardTarget.insert(keyboardTarget.cursorPosition, value)
    }

    function backspaceKeyboardText() {
        if (!isTextInput(keyboardTarget)) return
        keyboardTarget.forceActiveFocus()
        var pos = keyboardTarget.cursorPosition
        if (pos > 0) keyboardTarget.remove(pos - 1, pos)
    }

    function hideKeyboard() {
        if (isTextInput(keyboardTarget)) {
            keyboardTarget.editingFinished()
            keyboardTarget.focus = false
        }
        keyboardTarget = null
    }

    onActiveFocusItemChanged: {
        if (isTextInput(activeFocusItem)) keyboardTarget = activeFocusItem
    }

    onClosing: function(close) {
        backend.stopTrackingIfActive()
    }

    Loader {
        id: mainLoader
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            bottom: keyboardPanel.visible ? keyboardPanel.top : parent.bottom
        }
        source: Qt.resolvedUrl("DetectScreen.qml")
    }

    Rectangle {
        id: keyboardPanel
        visible: root.isTextInput(root.keyboardTarget) && root.keyboardTarget.activeFocus
        anchors {
            left: parent.left
            right: parent.right
            bottom: parent.bottom
        }
        height: visible ? Math.min(330, Math.max(260, parent.height * 0.36)) : 0
        color: "#0b0f14"
        border.color: "#30363d"
        border.width: 1
        z: 100

        Behavior on height { NumberAnimation { duration: 120; easing.type: Easing.OutCubic } }

        component KeyButton: Rectangle {
            property string label: ""
            property string value: label
            property real keyWeight: 1
            signal pressed()

            Layout.fillWidth: true
            Layout.preferredWidth: keyWeight
            Layout.preferredHeight: 46
            radius: 8
            color: keyArea.pressed ? "#2ea44f" : (keyArea.containsMouse ? "#30363d" : "#21262d")
            border.color: "#30363d"
            border.width: 1

            Text {
                anchors.centerIn: parent
                text: label
                color: "#ffffff"
                font.pixelSize: 18
                font.weight: Font.Medium
            }

            MouseArea {
                id: keyArea
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                preventStealing: true
                onClicked: parent.pressed()
            }
        }

        Column {
            anchors {
                fill: parent
                leftMargin: 18
                rightMargin: 18
                topMargin: 14
                bottomMargin: 14
            }
            spacing: 8

            RowLayout {
                width: parent.width
                spacing: 8
                Repeater {
                    model: ["1","2","3","4","5","6","7","8","9","0","-","_"]
                    KeyButton {
                        label: modelData
                        onPressed: root.insertKeyboardText(value)
                    }
                }
            }

            RowLayout {
                width: parent.width
                spacing: 8
                Repeater {
                    model: ["q","w","e","r","t","y","u","i","o","p"]
                    KeyButton {
                        label: root.keyboardShift ? modelData.toUpperCase() : modelData
                        value: label
                        onPressed: root.insertKeyboardText(value)
                    }
                }
            }

            RowLayout {
                width: parent.width
                spacing: 8
                Item { Layout.fillWidth: true; Layout.preferredWidth: 0.5 }
                Repeater {
                    model: ["a","s","d","f","g","h","j","k","l"]
                    KeyButton {
                        label: root.keyboardShift ? modelData.toUpperCase() : modelData
                        value: label
                        onPressed: root.insertKeyboardText(value)
                    }
                }
                Item { Layout.fillWidth: true; Layout.preferredWidth: 0.5 }
            }

            RowLayout {
                width: parent.width
                spacing: 8
                KeyButton {
                    label: root.keyboardShift ? "shift" : "SHIFT"
                    keyWeight: 1.45
                    onPressed: root.keyboardShift = !root.keyboardShift
                }
                Repeater {
                    model: ["z","x","c","v","b","n","m"]
                    KeyButton {
                        label: root.keyboardShift ? modelData.toUpperCase() : modelData
                        value: label
                        onPressed: root.insertKeyboardText(value)
                    }
                }
                KeyButton {
                    label: "<"
                    keyWeight: 1.45
                    onPressed: root.backspaceKeyboardText()
                }
            }

            RowLayout {
                width: parent.width
                spacing: 8
                KeyButton { label: "."; onPressed: root.insertKeyboardText(value) }
                KeyButton { label: "/"; onPressed: root.insertKeyboardText(value) }
                KeyButton { label: ":"; onPressed: root.insertKeyboardText(value) }
                KeyButton { label: "@"; onPressed: root.insertKeyboardText(value) }
                KeyButton {
                    label: "spatie"
                    value: " "
                    keyWeight: 4
                    onPressed: root.insertKeyboardText(value)
                }
                KeyButton {
                    label: "Verberg"
                    keyWeight: 1.6
                    onPressed: root.hideKeyboard()
                }
            }
        }
    }
}

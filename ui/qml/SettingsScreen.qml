// ui/qml/SettingsScreen.qml
// Runtime settings plus editable backend config.
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: settingsScreen
    anchors.fill: parent

    property int currentTab: 0

    property string editIp: backend.freedIp
    property int editPort: backend.freedPort
    property bool editFreedEnabled: backend.freedEnabled
    property double editCamOffX: backend.cameraOffsetX
    property double editCamOffY: backend.cameraOffsetY
    property double editCamOffZ: backend.cameraOffsetZ
    property var editConfig: ({})

    property var configSections: [
        { title: "Camera", description: "Camera-resolutie, belichting en lenspositie. Deze waarden gelden na herstart van de camera-pipeline.", fields: [
            { key: "width", label: "Breedte", type: "int", defaultValue: 2304 },
            { key: "height", label: "Hoogte", type: "int", defaultValue: 1296 },
            { key: "camera", label: "Camera index", type: "int", defaultValue: 0 },
            { key: "fps", label: "FPS", type: "int", defaultValue: 20 },
            { key: "shutter", label: "Shutter", type: "int", defaultValue: 1500 },
            { key: "gain", label: "Gain", type: "double", defaultValue: 0.0 },
            { key: "lens_position", label: "Lenspositie", type: "double", defaultValue: 10.0 },
            { key: "awb_gain_r", label: "AWB rood", type: "double", defaultValue: 0.0 },
            { key: "awb_gain_b", label: "AWB blauw", type: "double", defaultValue: 0.0 }
        ]},
        { title: "Sterdetectie", description: "Volledige detectie voor mapbouw en kalibratie.", fields: [
            { key: "sat_max", label: "Saturatie max", type: "int", defaultValue: 120 },
            { key: "val_min", label: "Waarde min", type: "int", defaultValue: 140 },
            { key: "bg_kernel", label: "Background kernel", type: "int", defaultValue: 31 },
            { key: "peak_floor", label: "Peak floor", type: "int", defaultValue: 8 },
            { key: "morph_kernel", label: "Morph kernel", type: "int", defaultValue: 3 },
            { key: "area_min", label: "Area min", type: "int", defaultValue: 151 },
            { key: "area_max", label: "Area max", type: "int", defaultValue: 650 },
            { key: "circ_min", label: "Circulariteit min", type: "double", defaultValue: 0.55 }
        ]},
        { title: "Live detectie", description: "Snelle detectie die tijdens tracking gebruikt wordt.", fields: [
            { key: "light_downsample", label: "Downsample", type: "int", defaultValue: 2 },
            { key: "light_blur_kernel", label: "Blur kernel", type: "int", defaultValue: 0 },
            { key: "light_threshold", label: "Threshold", type: "int", defaultValue: 200 },
            { key: "light_area_min", label: "Area min", type: "int", defaultValue: 21 },
            { key: "light_area_max", label: "Area max", type: "int", defaultValue: 150 },
            { key: "light_circ_min", label: "Circulariteit min", type: "double", defaultValue: 0.80 },
            { key: "use_heavy_detector", label: "Gebruik zware detector (trager, betere centroids)", type: "bool", defaultValue: false }
        ]},
        { title: "Localisatie", description: "RANSAC en homografie-drempels voor absolute localisatie.", fields: [
            { key: "ransac_iter", label: "RANSAC iteraties", type: "int", defaultValue: 5000 },
            { key: "thresh_px", label: "Threshold px", type: "double", defaultValue: 15.0 },
            { key: "verbose", label: "Verbose logging", type: "bool", defaultValue: false }
        ]},
        { title: "Tracking", description: "Kwaliteitsdrempels voor live tracking en tracking-loss.", fields: [
            { key: "max_px", label: "Max px zoekvenster", type: "double", defaultValue: 50.0 },
            { key: "max_rot_deg", label: "Max rotatie/frame", type: "double", defaultValue: 10.0 },
            { key: "min_match_pct", label: "Min match %", type: "double", defaultValue: 30.0 },
            { key: "max_reproj_m", label: "Max reproj. fout (m)", type: "double", defaultValue: 0.05 },
            { key: "min_tracking_stars", label: "Minimum sterren", type: "int", defaultValue: 3, minValue: 3 }
        ]},
        { title: "Mapbouw", description: "Parameters voor het scannen en samenvoegen van nieuwe starmaps.", fields: [
            { key: "map_fps", label: "Map FPS", type: "int", defaultValue: 3 },
            { key: "merge_radius_px", label: "Merge radius px", type: "double", defaultValue: 37.0 },
            { key: "min_frame_count", label: "Min frame count", type: "int", defaultValue: 5 },
            { key: "max_new_per_frame", label: "Max nieuw/frame", type: "int", defaultValue: 5 },
            { key: "min_inlier_ratio", label: "Min inlier ratio", type: "double", defaultValue: 0.60 }
        ]},
        { title: "Bestanden", description: "Kalibratiehoogte en paden die de daemon bij startup gebruikt.", fields: [
            { key: "calib_height", label: "Kalibratiehoogte", type: "double", defaultValue: 1.399 },
            { key: "star_map", label: "Star map", type: "string", defaultValue: "star_maps/star_map.csv" },
            { key: "intrinsics", label: "Intrinsics", type: "string", defaultValue: "" },
            { key: "intrinsics_backup", label: "Intrinsics backup", type: "string", defaultValue: "" },
            { key: "intrinsics_fixed", label: "Intrinsics fixed", type: "string", defaultValue: "" }
        ]}
    ]

    function configVal(key, def) {
        var v = backend.configValues[key]
        return v !== undefined ? v : def
    }
    function setConfigVal(key, value) {
        var obj = {}
        obj[key] = value
        backend.setConfig(obj)
    }

    function cloneMap(src) {
        var out = {}
        if (!src) return out
        for (var k in src) out[k] = src[k]
        return out
    }

    function configValue(key, fallbackValue) {
        if (editConfig && editConfig[key] !== undefined) return editConfig[key]
        return fallbackValue
    }

    function setConfigValue(key, value) {
        var next = cloneMap(editConfig)
        next[key] = value
        editConfig = next
    }

    function formatConfigValue(field) {
        var v = configValue(field.key, field.defaultValue)
        if (field.type === "int") return Math.round(Number(v)).toString()
        if (field.type === "double") return Number(v).toString()
        return v === undefined || v === null ? "" : v.toString()
    }

    function commitConfigText(field, text) {
        if (field.type === "string") {
            setConfigValue(field.key, text)
            return
        }

        var parsed = field.type === "int" ? parseInt(text) : parseFloat(text)
        if (isNaN(parsed)) return
        if (field.minValue !== undefined && parsed < field.minValue) parsed = field.minValue
        setConfigValue(field.key, parsed)
    }

    Rectangle {
        anchors.fill: parent
        color: "#0e1117"
    }

    Connections {
        target: backend
        function onSettingsUpdated() {
            settingsScreen.editIp = backend.freedIp
            settingsScreen.editPort = backend.freedPort
            settingsScreen.editFreedEnabled = backend.freedEnabled
            settingsScreen.editCamOffX = backend.cameraOffsetX
            settingsScreen.editCamOffY = backend.cameraOffsetY
            settingsScreen.editCamOffZ = backend.cameraOffsetZ
        }
        function onConfigUpdated() {
            settingsScreen.editConfig = settingsScreen.cloneMap(backend.configValues)
        }
    }

    Component.onCompleted: {
        backend.requestStatus()
        backend.requestConfig()
    }

    component PanelBg: Rectangle {
        color: "#161b22"
        radius: 12
        border.color: "#30363d"
        border.width: 1
    }

    component TabButton: Rectangle {
        property string label: ""
        property bool selected: false
        signal clicked()

        Layout.fillWidth: true
        height: 42
        radius: 8
        color: selected ? "#238636" : (tabArea.containsMouse ? "#21262d" : "transparent")
        border.color: selected ? "#2ea44f" : "#30363d"
        border.width: 1

        Text {
            anchors.centerIn: parent
            text: label
            color: "#ffffff"
            font.pixelSize: 15
            font.weight: selected ? Font.Bold : Font.Medium
        }

        MouseArea {
            id: tabArea
            anchors.fill: parent
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor
            onClicked: parent.clicked()
        }
    }

    component ConfigField: Rectangle {
        property var field

        width: parent.width
        implicitHeight: Math.max(fieldInfo.implicitHeight, 42) + 20
        radius: 10
        color: "#0e1117"
        border.color: "#30363d"
        border.width: 1

        RowLayout {
            anchors { fill: parent; margins: 10 }
            spacing: 12

            Column {
                id: fieldInfo
                Layout.fillWidth: true
                spacing: 3

                Text {
                    text: field.label
                    color: "#ffffff"
                    font.pixelSize: 14
                    font.weight: Font.Medium
                }
                Text {
                    text: field.key
                    color: "#8b949e"
                    font.pixelSize: 12
                    font.family: "monospace"
                }
            }

            Switch {
                visible: field.type === "bool"
                checked: !!settingsScreen.configValue(field.key, field.defaultValue)
                onToggled: settingsScreen.setConfigValue(field.key, checked)
            }

            Rectangle {
                visible: field.type !== "bool"
                Layout.preferredWidth: field.type === "string" ? 360 : 150
                height: 42
                radius: 6
                color: "#010409"
                border.color: configInput.activeFocus ? "#2ea44f" : "#30363d"
                border.width: 1

                TextInput {
                    id: configInput
                    anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                    verticalAlignment: TextInput.AlignVCenter
                    text: settingsScreen.formatConfigValue(field)
                    color: "#ffffff"
                    selectionColor: "#238636"
                    selectedTextColor: "#ffffff"
                    font.pixelSize: 14
                    font.family: "monospace"
                    inputMethodHints: Qt.ImhNoAutoUppercase | Qt.ImhNoPredictiveText
                    onEditingFinished: settingsScreen.commitConfigText(field, text)
                }
            }
        }
    }

    Rectangle {
        id: settingsTitleBar
        anchors { top: parent.top; left: parent.left; right: parent.right }
        height: 56
        color: "#161b22"

        RowLayout {
            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left; right: parent.right
                leftMargin: 16; rightMargin: 16
            }

            Rectangle {
                width: 40; height: 40; radius: 6
                color: sbBackArea.containsMouse ? "#30363d" : "transparent"
                Text { anchors.centerIn: parent; text: "\u2190"; color: "#ffffff"; font.pixelSize: 22 }
                MouseArea {
                    id: sbBackArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: root.navigateBack()
                }
            }

            Text {
                text: "Instellingen"
                color: "#ffffff"
                font.pixelSize: 20
                font.weight: Font.Medium
                Layout.fillWidth: true
                leftPadding: 8
            }
        }
    }

    Rectangle {
        id: settingsTabs
        anchors { top: settingsTitleBar.bottom; left: parent.left; right: parent.right }
        height: 66
        color: "#0e1117"

        RowLayout {
            anchors { fill: parent; leftMargin: 32; rightMargin: 32; topMargin: 12; bottomMargin: 12 }
            spacing: 12

            TabButton {
                label: "Runtime"
                selected: settingsScreen.currentTab === 0
                onClicked: settingsScreen.currentTab = 0
            }
            TabButton {
                label: "Config"
                selected: settingsScreen.currentTab === 1
                onClicked: settingsScreen.currentTab = 1
            }
        }
    }

    Flickable {
        visible: settingsScreen.currentTab === 0
        anchors {
            top: settingsTabs.bottom; left: parent.left
            right: parent.right; bottom: parent.bottom
        }
        contentHeight: runtimeCol.implicitHeight + 48
        clip: true

        Column {
            id: runtimeCol
            anchors { top: parent.top; left: parent.left; right: parent.right; margins: 32 }
            spacing: 32
            topPadding: 24

            PanelBg {
                width: parent.width
                implicitHeight: commSection.implicitHeight + 32

                Column {
                    id: commSection
                    anchors { fill: parent; margins: 16 }
                    spacing: 16

                    Text { text: "Communicatie"; color: "#ffffff"; font.pixelSize: 18; font.weight: Font.Medium }
                    Text {
                        text: "Stel de FreeD-uitvoer in en beslis of de UDP-stream actief moet zijn."
                        color: "#8b949e"; font.pixelSize: 13
                        wrapMode: Text.WordWrap
                        width: parent.width
                    }

                    RowLayout {
                        width: parent.width
                        spacing: 16

                        Switch {
                            checked: settingsScreen.editFreedEnabled
                            onToggled: settingsScreen.editFreedEnabled = checked
                        }
                        Text {
                            text: "FreeD-uitvoer"
                            color: "#ffffff"
                            font.pixelSize: 15
                            font.weight: Font.Medium
                            Layout.fillWidth: true
                        }
                    }

                    RowLayout {
                        width: Math.min(parent.width, 620)
                        spacing: 16

                        Column {
                            spacing: 6
                            Layout.fillWidth: true
                            Text { text: "IP-adres"; color: "#aaaaaa"; font.pixelSize: 13 }
                            Rectangle {
                                width: parent.width; height: 42; radius: 6
                                color: "#0e1117"; border.color: ipField.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                                TextInput {
                                    id: ipField
                                    anchors { fill: parent; leftMargin: 12; rightMargin: 12 }
                                    verticalAlignment: TextInput.AlignVCenter
                                    text: settingsScreen.editIp
                                    color: "#ffffff"; font.pixelSize: 15; font.family: "monospace"
                                    inputMethodHints: Qt.ImhNoAutoUppercase | Qt.ImhNoPredictiveText
                                    onTextChanged: settingsScreen.editIp = text
                                }
                            }
                        }

                        Column {
                            spacing: 6
                            Layout.preferredWidth: 140
                            Text { text: "Poort"; color: "#aaaaaa"; font.pixelSize: 13 }
                            Rectangle {
                                width: parent.width; height: 42; radius: 6
                                color: "#0e1117"; border.color: portField.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                                TextInput {
                                    id: portField
                                    anchors { fill: parent; leftMargin: 12; rightMargin: 12 }
                                    verticalAlignment: TextInput.AlignVCenter
                                    text: settingsScreen.editPort.toString()
                                    color: "#ffffff"; font.pixelSize: 15; font.family: "monospace"
                                    validator: IntValidator { bottom: 1; top: 65535 }
                                    onTextChanged: {
                                        var v = parseInt(text)
                                        if (!isNaN(v)) settingsScreen.editPort = v
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // \u2500\u2500 Camera-offset \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
            PanelBg {
                width: parent.width
                implicitHeight: camOffSection.implicitHeight + 32

                Column {
                    id: camOffSection
                    anchors { fill: parent; margins: 16 }
                    spacing: 16

                    Text { text: "Camera-offset"; color: "#ffffff"; font.pixelSize: 18; font.weight: Font.Medium }
                    Text {
                        text: "Verschuiving van tracker naar optisch knooppunt van de cameralenzen (trackerlichaamsvlak: X/Y horizontaal, Z omhoog naar plafond)."
                        color: "#8b949e"; font.pixelSize: 13
                        wrapMode: Text.WordWrap
                        width: parent.width
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "X  (m)"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: camOffXInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: camOffXInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.editCamOffX.toFixed(4)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onTextChanged: { var v = parseFloat(text); if (!isNaN(v)) settingsScreen.editCamOffX = v }
                            }
                        }
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "Y  (m)"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: camOffYInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: camOffYInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.editCamOffY.toFixed(4)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onTextChanged: { var v = parseFloat(text); if (!isNaN(v)) settingsScreen.editCamOffY = v }
                            }
                        }
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "Z  (m)"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: camOffZInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: camOffZInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.editCamOffZ.toFixed(4)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onTextChanged: { var v = parseFloat(text); if (!isNaN(v)) settingsScreen.editCamOffZ = v }
                            }
                        }
                    }
                }
            }

            // \u2500\u2500 ESKF configuratie \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
            PanelBg {
                width: parent.width
                implicitHeight: eskfSection.implicitHeight + 32

                Column {
                    id: eskfSection
                    anchors { fill: parent; margins: 16 }
                    spacing: 16

                    Text {
                        text: "ESKF"
                        color: "#ffffff"
                        font.pixelSize: 18
                        font.weight: Font.Medium
                    }
                    Text {
                        text: "Ruisparameters voor de Error-State Kalman Filter."
                        color: "#8b949e"; font.pixelSize: 13
                        wrapMode: Text.WordWrap; width: parent.width
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "Ruisschaal (noise_scale)"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: noiseScaleInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: noiseScaleInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.configVal("eskf_noise_scale", 1.0).toFixed(1)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onEditingFinished: {
                                    var v = parseFloat(text)
                                    if (!isNaN(v)) settingsScreen.setConfigVal("eskf_noise_scale", v)
                                    else text = settingsScreen.configVal("eskf_noise_scale", 1.0).toFixed(1)
                                }
                            }
                        }
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "Positie sigma [m]"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: sigmaPosInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: sigmaPosInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.configVal("eskf_sigma_cam_pos", 0.05).toFixed(4)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onEditingFinished: {
                                    var v = parseFloat(text)
                                    if (!isNaN(v)) settingsScreen.setConfigVal("eskf_sigma_cam_pos", v)
                                    else text = settingsScreen.configVal("eskf_sigma_cam_pos", 0.05).toFixed(4)
                                }
                            }
                        }
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "Attitude sigma [rad]"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: sigmaAttInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: sigmaAttInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.configVal("eskf_sigma_cam_att", 0.01).toFixed(4)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onEditingFinished: {
                                    var v = parseFloat(text)
                                    if (!isNaN(v)) settingsScreen.setConfigVal("eskf_sigma_cam_att", v)
                                    else text = settingsScreen.configVal("eskf_sigma_cam_att", 0.01).toFixed(4)
                                }
                            }
                        }
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "Snelheidsdemping [s]  (0 = uit)"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: velDecayInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: velDecayInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.configVal("vel_decay_s", 0.0).toFixed(2)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onEditingFinished: {
                                    var v = parseFloat(text)
                                    if (!isNaN(v)) settingsScreen.setConfigVal("vel_decay_s", v)
                                    else text = settingsScreen.configVal("vel_decay_s", 0.0).toFixed(2)
                                }
                            }
                        }
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "Smoothing min cutoff [Hz]  (0 = uit)"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: smoothCutoffInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: smoothCutoffInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.configVal("smooth_min_cutoff", 0.0).toFixed(1)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onEditingFinished: {
                                    var v = parseFloat(text)
                                    if (!isNaN(v)) settingsScreen.setConfigVal("smooth_min_cutoff", v)
                                    else text = settingsScreen.configVal("smooth_min_cutoff", 0.0).toFixed(1)
                                }
                            }
                        }
                    }

                    RowLayout {
                        width: parent.width; spacing: 12
                        Text { text: "Smoothing beta"; color: "#aaaaaa"; font.pixelSize: 13; Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 160; height: 38; radius: 6
                            color: "#010409"; border.color: smoothBetaInput.activeFocus ? "#2ea44f" : "#30363d"; border.width: 1
                            TextInput {
                                id: smoothBetaInput
                                anchors { fill: parent; leftMargin: 10; rightMargin: 10 }
                                verticalAlignment: TextInput.AlignVCenter
                                text: settingsScreen.configVal("smooth_beta", 0.5).toFixed(2)
                                color: "#ffffff"; font.pixelSize: 14; font.family: "monospace"
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onEditingFinished: {
                                    var v = parseFloat(text)
                                    if (!isNaN(v)) settingsScreen.setConfigVal("smooth_beta", v)
                                    else text = settingsScreen.configVal("smooth_beta", 0.5).toFixed(2)
                                }
                            }
                        }
                    }

                    Text { text: "Plafond hoogte (default = 2.4 m)"; color: "#aaaaaa"; font.pixelSize: 13 }
                    RowLayout {
                        width: Math.min(parent.width, 500); spacing: 8
                        TextField {
                            id: ceilingHeightField
                            Layout.preferredWidth: 120
                            text: settingsScreen.configVal("ceiling_height_m", 2.4).toFixed(3)
                            color: "#ffffff"
                            font.pixelSize: 14
                            font.family: "monospace"
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                            background: Rectangle {
                                color: "#21262d"; radius: 6
                                border.color: ceilingHeightField.activeFocus ? "#2ea44f" : "#30363d"
                                border.width: 1
                            }
                            validator: RegularExpressionValidator {
                                regularExpression: /^[0-9]{1,2}([,.][0-9]{0,3})?$/
                            }
                        }
                        Text { text: "m"; color: "#8b949e"; font.pixelSize: 13 }
                        Rectangle {
                            width: 60; height: 36; radius: 6
                            color: applyHeightArea.containsMouse ? "#1a7f3c" : "#21262d"
                            border.color: "#2ea44f"; border.width: 1
                            Text { anchors.centerIn: parent; text: "OK"; color: "#2ea44f"; font.pixelSize: 13 }
                            MouseArea {
                                id: applyHeightArea
                                anchors.fill: parent
                                hoverEnabled: true
                                cursorShape: Qt.PointingHandCursor
                                onClicked: {
                                    var v = parseFloat(ceilingHeightField.text.replace(",", "."))
                                    if (!isNaN(v) && v >= 1.0 && v <= 6.0)
                                        settingsScreen.setConfigVal("ceiling_height_m", v)
                                    else
                                        ceilingHeightField.text = settingsScreen.configVal("ceiling_height_m", 2.4).toFixed(3)
                                }
                            }
                        }
                    }
                }
            }

            Rectangle {
                width: 140; height: 46; radius: 8
                color: runtimeSaveArea.containsMouse ? "#1a7f3c" : "#2ea44f"
                Text { anchors.centerIn: parent; text: "Opslaan"; color: "#ffffff"; font.pixelSize: 16; font.weight: Font.Medium }
                MouseArea {
                    id: runtimeSaveArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        backend.setSettings(settingsScreen.editIp, settingsScreen.editPort)
                        backend.setRuntimeFlags(settingsScreen.editFreedEnabled, true)
                        backend.setCameraOffset(settingsScreen.editCamOffX, settingsScreen.editCamOffY, settingsScreen.editCamOffZ)
                        root.navigateBack()
                    }
                }
            }
        }
    }

    Flickable {
        visible: settingsScreen.currentTab === 1
        anchors {
            top: settingsTabs.bottom; left: parent.left
            right: parent.right; bottom: parent.bottom
        }
        contentHeight: configCol.implicitHeight + 48
        clip: true

        Column {
            id: configCol
            anchors { top: parent.top; left: parent.left; right: parent.right; margins: 32 }
            spacing: 22
            topPadding: 24

            Text {
                text: "Config wordt opgeslagen in de daemon-config. Tracking-drempels worden live opnieuw opgebouwd; camera- en detectieparameters vragen meestal een herstart van de daemon/camera."
                color: "#8b949e"
                font.pixelSize: 13
                wrapMode: Text.WordWrap
                width: parent.width
            }

            Repeater {
                model: settingsScreen.configSections

                PanelBg {
                    width: parent.width
                    implicitHeight: sectionCol.implicitHeight + 32

                    Column {
                        id: sectionCol
                        anchors { fill: parent; margins: 16 }
                        spacing: 12

                        Text {
                            text: modelData.title
                            color: "#ffffff"
                            font.pixelSize: 18
                            font.weight: Font.Medium
                        }
                        Text {
                            text: modelData.description
                            color: "#8b949e"
                            font.pixelSize: 13
                            wrapMode: Text.WordWrap
                            width: parent.width
                        }

                        Repeater {
                            model: modelData.fields
                            ConfigField {
                                field: modelData
                            }
                        }
                    }
                }
            }

            Rectangle {
                width: 190; height: 46; radius: 8
                color: configSaveArea.containsMouse ? "#1a7f3c" : "#2ea44f"
                Text { anchors.centerIn: parent; text: "Config opslaan"; color: "#ffffff"; font.pixelSize: 16; font.weight: Font.Medium }
                MouseArea {
                    id: configSaveArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        backend.setConfig(settingsScreen.editConfig)
                        backend.requestConfig()
                        root.navigateBack()
                    }
                }
            }
        }
    }
}

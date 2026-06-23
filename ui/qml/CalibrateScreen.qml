import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: calibScreen
    anchors.fill: parent

    property int currentStep: 0
    property string calibPromptMsg: ""
    property int calibPromptStep: 0
    property string activeRun: ""   // "" | scale_ready | scale | map

    property string methodChoice: ""   // "" | "choose" | "movement" | "anchor"
    property bool anchorOverlayReady: false
    property var mapMarkerIds: []
    property int anchorIdA: -1
    property int anchorIdB: -1
    property real anchorDistCm: 0
    property string pendingAfterScale: ""
    property string lastScaleFile: ""
    property real lastScaleHeightM: 0


    function startAnchorFlow() {
        var pts = backend.currentMapPoints
        var ids = []
        for (var i = 0; i < pts.length; i++) {
            ids.push(pts[i].id !== undefined ? pts[i].id : i)
        }
        mapMarkerIds = ids
        anchorOverlayReady = false
        anchorIdA = ids.length > 0 ? ids[0] : -1
        anchorIdB = ids.length > 1 ? ids[1] : -1
        anchorDistCm = 0
        methodChoice = "anchor"
        backend.anchorPreview()
    }

    function parseDistanceCm(text) {
        var normalized = (text || "").replace(/,/g, ".")
        var value = Number(normalized)
        return isFinite(value) ? value : 0
    }

    function confirmAnchor() {
        if (anchorIdA < 0 || anchorIdB < 0 || anchorIdA === anchorIdB || anchorDistCm <= 0)
            return
        backend.scaleAnchor(anchorIdA, anchorIdB, anchorDistCm / 100.0)
        methodChoice = ""
    }

    function stepLabel(i) {
        return i === 0 ? "1. Schaalcalibratie" : "2. Sterrenkaart"
    }

    function requestNewScaleMeasurement() {
        calibPromptMsg = ""
        calibPromptStep = 0
        activeRun = "scale_ready"
    }

    function requestStandaloneScaleMeasurement() {
        pendingAfterScale = ""
        requestNewScaleMeasurement()
    }

    function beginScaleMeasurement() {
        calibPromptMsg = ""
        calibPromptStep = 0
        activeRun = "scale"
        backend.calibrateScale("")
    }

    function startMapScanNow() {
        calibPromptMsg = ""
        calibPromptStep = 0
        backend.clearScanOverlay()
        activeRun = "map"
        backend.buildMap("")
    }

    function requestNewMapScan() {
        methodChoice = "map_scale"
    }

    function useScaleForNewMap(fileName) {
        if (!fileName || fileName.length === 0)
            return
        pendingAfterScale = "map"
        methodChoice = ""
        backend.calibrateScale(fileName)
    }

    function makeScaleForNewMap() {
        pendingAfterScale = "map"
        methodChoice = ""
        requestNewScaleMeasurement()
    }

    function closeRunOverlay() {
        activeRun = ""
        calibPromptMsg = ""
        calibPromptStep = 0
        backend.clearScanOverlay()
    }

    function mapStatusText() {
        if (backend.scanFrame <= 0)
            return "Scan wordt gestart…"
        return "Frame " + backend.scanFrame
                + "  |  detecties " + backend.scanDetectedCount
                + "  |  bevestigd " + backend.scanConfirmedCount
                + " / " + backend.scanTotalCount
                + "  |  matches " + backend.scanMatchedCount
                + "  |  nieuw " + backend.scanNewCount
    }

    Rectangle {
        anchors.fill: parent
        color: "#0e1117"
    }

    Connections {
        target: backend

        function onCalibPromptReceived(step, msg) {
            calibScreen.activeRun = "scale"
            calibScreen.calibPromptStep = step
            calibScreen.calibPromptMsg = msg
        }

        function onScaleDone(file, height_m) {
            calibScreen.lastScaleFile = file
            calibScreen.lastScaleHeightM = height_m
            backend.requestFileList("scale_calibration")
            backend.requestFileList("star_maps")
            backend.requestConfig()
            if (calibScreen.pendingAfterScale === "map") {
                calibScreen.pendingAfterScale = ""
                calibScreen.closeRunOverlay()
                calibScreen.startMapScanNow()
            } else {
                calibScreen.closeRunOverlay()
            }
        }

        function onCalibrationDone(file, stars) {
            calibScreen.closeRunOverlay()
            backend.requestFileList("star_maps")
            backend.requestCurrentMap()
        }

        function onErrorOccurred(msg) {
            calibScreen.closeRunOverlay()
            calibScreen.methodChoice = ""
            calibScreen.anchorOverlayReady = false
        }

        function onScanOverlayUpdated() {
            mapOverlayCanvas.requestPaint()
        }
    }

    Component.onCompleted: {
        backend.requestFileList("scale_calibration")
        backend.requestFileList("star_maps")
        backend.requestCurrentMap()
        backend.requestConfig()
    }

    Rectangle {
        id: titleBar
        anchors { top: parent.top; left: parent.left; right: parent.right }
        height: 56
        color: "#161b22"

        RowLayout {
            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
                right: parent.right
                leftMargin: 16
                rightMargin: 16
            }

            Rectangle {
                width: 40
                height: 40
                radius: 6
                color: backArea.containsMouse ? "#30363d" : "transparent"
                Behavior on color { ColorAnimation { duration: 80 } }

                Text {
                    anchors.centerIn: parent
                    text: "←"
                    color: "#ffffff"
                    font.pixelSize: 22
                }

                MouseArea {
                    id: backArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        if (calibScreen.activeRun === "map")
                            backend.stopBuildMap()
                        root.navigateBack()
                    }
                }
            }

            Text {
                text: "Kalibratie"
                color: "#ffffff"
                font.pixelSize: 20
                font.weight: Font.Medium
                Layout.fillWidth: true
                leftPadding: 8
            }

            Text {
                text: backend.calibrationComplete ? "kaart actief" : "geen actieve kaart"
                color: backend.calibrationComplete ? "#2ea44f" : "#8b949e"
                font.pixelSize: 13
            }
        }
    }

    RowLayout {
        id: calibrationDashboard
        anchors {
            top: titleBar.bottom
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            margins: 20
        }
        spacing: 20

        Rectangle {
            Layout.fillHeight: true
            Layout.preferredWidth: Math.max(330, calibrationDashboard.width * 0.34)
            radius: 8
            color: "#161b22"
            border.color: "#30363d"
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 18
                spacing: 14

                Text {
                    text: "Huidige configuratie"
                    color: "#ffffff"
                    font.pixelSize: 20
                    font.weight: Font.Medium
                }

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: loadedMapCol.implicitHeight + 24
                    radius: 7
                    color: "#0e1117"
                    border.color: "#30363d"
                    border.width: 1

                    Column {
                        id: loadedMapCol
                        anchors { left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter; margins: 12 }
                        spacing: 6
                        Text { text: "Actieve map"; color: "#8b949e"; font.pixelSize: 13 }
                        Text {
                            width: parent.width
                            text: backend.currentMapName.length > 0 ? backend.currentMapName : "Geen map geladen"
                            color: "#ffffff"
                            font.pixelSize: 17
                            elide: Text.ElideRight
                        }
                        Text {
                            text: backend.currentMapPoints.length + " sterren geladen"
                            color: backend.currentMapPoints.length > 0 ? "#2ea44f" : "#8b949e"
                            font.pixelSize: 13
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: scaleStateCol.implicitHeight + 24
                    radius: 7
                    color: "#0e1117"
                    border.color: "#30363d"
                    border.width: 1

                    Column {
                        id: scaleStateCol
                        anchors { left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter; margins: 12 }
                        spacing: 6
                        Text { text: "Schaal voor nieuwe scans"; color: "#8b949e"; font.pixelSize: 13 }
                        Text {
                            text: calibScreen.lastScaleHeightM > 0
                                  ? calibScreen.lastScaleHeightM.toFixed(3) + " m actief"
                                  : (backend.configValues.calib_height !== undefined
                                     ? Number(backend.configValues.calib_height).toFixed(3) + " m actief"
                                     : "Nog niet gekozen")
                            color: "#ffffff"
                            font.pixelSize: 17
                        }
                        Text {
                            width: parent.width
                            text: calibScreen.lastScaleFile.length > 0
                                  ? calibScreen.lastScaleFile
                                  : "Wordt gevraagd wanneer je een nieuwe map bouwt"
                            color: "#8b949e"
                            font.pixelSize: 13
                            elide: Text.ElideRight
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    height: 44
                    radius: 7
                    color: newScaleMainArea.containsMouse ? "#30363d" : "#21262d"
                    border.color: "#30363d"
                    border.width: 1
                    Text { anchors.centerIn: parent; text: "Nieuwe 10 cm meting"; color: "#ffffff"; font.pixelSize: 14 }
                    MouseArea {
                        id: newScaleMainArea
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.PointingHandCursor
                        onClicked: calibScreen.requestStandaloneScaleMeasurement()
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    height: 44
                    radius: 7
                    opacity: backend.currentMapPoints.length > 0 ? 1.0 : 0.4
                    color: anchorMainArea.containsMouse && backend.currentMapPoints.length > 0 ? "#1c4a2e" : "#21262d"
                    border.color: "#2ea44f"
                    border.width: 1
                    Text {
                        anchors.centerIn: parent
                        text: "Schaalanker instellen"
                        color: backend.currentMapPoints.length > 0 ? "#2ea44f" : "#8b949e"
                        font.pixelSize: 14
                    }
                    MouseArea {
                        id: anchorMainArea
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: backend.currentMapPoints.length > 0 ? Qt.PointingHandCursor : Qt.ForbiddenCursor
                        onClicked: if (backend.currentMapPoints.length > 0) calibScreen.startAnchorFlow()
                    }
                }

                Item { Layout.fillHeight: true }
            }
        }

        Rectangle {
            Layout.fillHeight: true
            Layout.fillWidth: true
            radius: 8
            color: "#0e1117"
            border.color: "#30363d"
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 18
                spacing: 14

                RowLayout {
                    Layout.fillWidth: true
                    Text {
                        text: "Sterrenkaarten"
                        color: "#ffffff"
                        font.pixelSize: 20
                        font.weight: Font.Medium
                        Layout.fillWidth: true
                    }
                    Rectangle {
                        width: 156
                        height: 42
                        radius: 7
                        color: newMapMainArea.containsMouse ? "#1a7f3c" : "#2ea44f"
                        Text { anchors.centerIn: parent; text: "Nieuwe map bouwen"; color: "#ffffff"; font.pixelSize: 14; font.weight: Font.Medium }
                        MouseArea {
                            id: newMapMainArea
                            anchors.fill: parent
                            hoverEnabled: true
                            cursorShape: Qt.PointingHandCursor
                            onClicked: calibScreen.requestNewMapScan()
                        }
                    }
                }

                ListView {
                    id: mapList
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    model: backend.mapFiles
                    clip: true
                    spacing: 8

                    delegate: Rectangle {
                        width: mapList.width
                        height: 76
                        radius: 7
                        color: mapListArea.containsMouse ? "#21262d" : "#161b22"
                        border.color: backend.currentMapName === fileData.name ? "#2ea44f" : "#30363d"
                        border.width: 1
                        property var fileData: backend.mapFiles[index]
                        property bool confirmDelete: false

                        RowLayout {
                            anchors { fill: parent; leftMargin: 14; rightMargin: 14 }
                            spacing: 12

                            Column {
                                Layout.fillWidth: true
                                spacing: 5
                                Text {
                                    width: parent.width
                                    text: fileData ? fileData.name : ""
                                    color: "#ffffff"
                                    font.pixelSize: 15
                                    elide: Text.ElideRight
                                }
                                Text {
                                    text: fileData ? (fileData.stars + " sterren  |  " + fileData.date) : ""
                                    color: "#8b949e"
                                    font.pixelSize: 13
                                }
                            }

                            Column {
                                spacing: 5
                                Text {
                                    text: fileData ? fileData.scale_label : ""
                                    color: fileData && fileData.scale_status === "final" ? "#2ea44f" : "#ffd33d"
                                    font.pixelSize: 14
                                    horizontalAlignment: Text.AlignRight
                                    width: 110
                                }
                                Text {
                                    text: fileData && fileData.scale_height_m > 0
                                          ? fileData.scale_height_m.toFixed(3) + " m"
                                          : (fileData ? fileData.scale_source : "")
                                    color: "#8b949e"
                                    font.pixelSize: 12
                                    horizontalAlignment: Text.AlignRight
                                    width: 110
                                }
                            }

                            Rectangle {
                                width: 92
                                height: 38
                                radius: 7
                                color: loadMapArea.containsMouse ? "#30363d" : "#21262d"
                                border.color: "#30363d"
                                border.width: 1
                                Text { anchors.centerIn: parent; text: backend.currentMapName === fileData.name ? "Actief" : "Laad"; color: "#ffffff"; font.pixelSize: 13 }
                                MouseArea {
                                    id: loadMapArea
                                    anchors.fill: parent
                                    hoverEnabled: true
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: if (fileData) backend.buildMap(fileData.name)
                                }
                            }

                            Rectangle {
                                width: confirmDelete ? 92 : 38
                                height: 38
                                radius: 7
                                color: deleteArea.containsMouse
                                       ? (confirmDelete ? "#7d1515" : "#3d1515")
                                       : (confirmDelete ? "#5a1010" : "#21262d")
                                border.color: confirmDelete ? "#f85149" : "#30363d"
                                border.width: 1

                                Behavior on width { NumberAnimation { duration: 120 } }

                                Text {
                                    anchors.centerIn: parent
                                    text: confirmDelete ? "Zeker?" : "✕"
                                    color: confirmDelete ? "#f85149" : "#8b949e"
                                    font.pixelSize: 13
                                }
                                MouseArea {
                                    id: deleteArea
                                    anchors.fill: parent
                                    hoverEnabled: true
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        if (!fileData) return
                                        if (confirmDelete) {
                                            backend.deleteMap(fileData.name)
                                            confirmDelete = false
                                        } else {
                                            confirmDelete = true
                                        }
                                    }
                                }
                            }
                        }

                        MouseArea {
                            id: mapListArea
                            anchors.fill: parent
                            hoverEnabled: true
                            acceptedButtons: Qt.NoButton
                            onClicked: confirmDelete = false
                        }
                    }

                    Text {
                        anchors.centerIn: parent
                        visible: mapList.count === 0
                        text: "Geen opgeslagen kaarten gevonden."
                        color: "#8b949e"
                        font.pixelSize: 15
                    }
                }
            }
        }
    }

    // ── Schaal kiezen voor nieuwe map ─────────────────────────────────
    Rectangle {
        visible: calibScreen.methodChoice === "map_scale"
        anchors.fill: parent
        color: "#dd0d1117"
        z: 10

        Rectangle {
            anchors.centerIn: parent
            width: Math.min(parent.width - 48, 720)
            height: Math.min(parent.height - 80, 560)
            radius: 10
            color: "#161b22"
            border.color: "#30363d"
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 18
                spacing: 14

                Text {
                    text: "Schaal voor nieuwe map"
                    color: "#ffffff"
                    font.pixelSize: 21
                    font.weight: Font.Medium
                }

                Text {
                    Layout.fillWidth: true
                    text: "Kies welke hoogteschatting de map builder gebruikt, of maak eerst een nieuwe 10 cm meting. Een anker kan daarna dezelfde map definitief schalen."
                    color: "#8b949e"
                    font.pixelSize: 13
                    wrapMode: Text.WordWrap
                }

                ListView {
                    id: mapScaleList
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    model: backend.scaleFiles
                    clip: true
                    spacing: 7

                    delegate: Rectangle {
                        width: mapScaleList.width
                        height: 58
                        radius: 7
                        color: scaleChoiceArea.containsMouse ? "#21262d" : "#0e1117"
                        border.color: mapScaleList.currentIndex === index ? "#2ea44f" : "#30363d"
                        border.width: 1
                        property var fileData: backend.scaleFiles[index]

                        RowLayout {
                            anchors { fill: parent; leftMargin: 12; rightMargin: 12 }
                            Text {
                                text: fileData ? fileData.name : ""
                                color: "#ffffff"
                                font.pixelSize: 14
                                Layout.fillWidth: true
                                elide: Text.ElideRight
                            }
                            Text {
                                text: fileData ? fileData.height_m.toFixed(3) + " m" : ""
                                color: "#2ea44f"
                                font.pixelSize: 14
                            }
                            Text {
                                text: fileData ? fileData.date : ""
                                color: "#8b949e"
                                font.pixelSize: 12
                            }
                        }

                        MouseArea {
                            id: scaleChoiceArea
                            anchors.fill: parent
                            hoverEnabled: true
                            cursorShape: Qt.PointingHandCursor
                            onClicked: mapScaleList.currentIndex = index
                        }
                    }

                    Text {
                        anchors.centerIn: parent
                        visible: mapScaleList.count === 0
                        text: "Geen opgeslagen 10 cm metingen gevonden."
                        color: "#8b949e"
                        font.pixelSize: 14
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10
                    Button {
                        text: "Annuleer"
                        onClicked: calibScreen.methodChoice = ""
                    }
                    Item { Layout.fillWidth: true }
                    Button {
                        text: "Nieuwe 10 cm meting"
                        onClicked: calibScreen.makeScaleForNewMap()
                    }
                    Button {
                        text: "Gebruik geselecteerde"
                        enabled: mapScaleList.currentIndex >= 0
                        onClicked: {
                            var f = backend.scaleFiles[mapScaleList.currentIndex]
                            if (f)
                                calibScreen.useScaleForNewMap(f.name)
                        }
                    }
                }
            }
        }
    }

    // ── Methode kiezen ────────────────────────────────────────────────
    Rectangle {
        visible: calibScreen.methodChoice === "choose"
        anchors.fill: parent
        color: "#0d1117"
        radius: 8
        z: 10

        Column {
            anchors.centerIn: parent
            spacing: 20
            width: Math.min(parent.width - 40, 420)

            Text {
                text: "Schaalkalibratie — methode kiezen"
                color: "#ffffff"
                font.pixelSize: 20
                font.weight: Font.Medium
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Text {
                text: "Verplaatsing: beweeg de camera ~10 cm en meet de pixelverschuiving.\n" +
                      "Twee punten: meet de afstand tussen twee bekende sterren (vereist geladen kaart)."
                color: "#8b949e"
                font.pixelSize: 13
                wrapMode: Text.WordWrap
                width: parent.width
            }

            Rectangle {
                width: parent.width; height: 50; radius: 7
                color: movArea.containsMouse ? "#30363d" : "#21262d"
                border.color: "#30363d"; border.width: 1
                Text { anchors.centerIn: parent; text: "Verplaatsing (~10 cm)"; color: "#ffffff"; font.pixelSize: 15 }
                MouseArea {
                    id: movArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        calibScreen.methodChoice = ""
                        calibScreen.requestNewScaleMeasurement()
                    }
                }
            }

            Rectangle {
                width: parent.width; height: 50; radius: 7
                opacity: backend.currentMapPoints.length > 0 ? 1.0 : 0.4
                color: ancArea.containsMouse && backend.currentMapPoints.length > 0
                       ? "#1c4a2e" : "#21262d"
                border.color: "#2ea44f"; border.width: 1
                Text {
                    anchors.centerIn: parent
                    text: backend.currentMapPoints.length > 0
                          ? "Twee punten (anker op kaart)"
                          : "Twee punten — vereist geladen sterkaart"
                    color: backend.currentMapPoints.length > 0 ? "#2ea44f" : "#8b949e"
                    font.pixelSize: 15
                }
                MouseArea {
                    id: ancArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: backend.currentMapPoints.length > 0
                                 ? Qt.PointingHandCursor : Qt.ArrowCursor
                    onClicked: {
                        if (backend.currentMapPoints.length > 0)
                            calibScreen.startAnchorFlow()
                    }
                }
            }

            Text {
                text: "Annuleren"
                color: "#8b949e"
                font.pixelSize: 13
                anchors.horizontalCenter: parent.horizontalCenter
                MouseArea {
                    anchors.fill: parent
                    onClicked: calibScreen.methodChoice = ""
                }
            }
        }
    }

    // ── Anker-tool ────────────────────────────────────────────────────
    Rectangle {
        visible: calibScreen.methodChoice === "anchor"
        anchors.fill: parent
        color: "#0d1117"
        radius: 8
        z: 10

        Connections {
            target: backend
            function onAnchorPreviewDone(ok, matched) {
                calibScreen.anchorOverlayReady = true
                if (!ok)
                    anchorStatusText.text = "Lokalisatie mislukt — detections staan in beeld, maar kaartprojectie is niet betrouwbaar."
                else
                    anchorStatusText.text = matched + " sterren herkend. Kies twee en meet de afstand."
            }
            function onScaleAnchorDone(sf) {
                calibScreen.methodChoice = ""
                backend.requestFileList("scale_calibration")
            }
        }

        RowLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 20

            Rectangle {
                Layout.fillHeight: true
                Layout.preferredWidth: parent.width * 0.55
                color: "#161b22"
                radius: 6

                Image {
                    id: anchorPreviewImg
                    anchors.fill: parent
                    anchors.margins: 4
                    fillMode: Image.PreserveAspectFit
                    source: "image://frame/anchor-" + backend.frameCounter
                    cache: false

                    Connections {
                        target: backend
                        function onAnchorPreviewDone(ok, matched) {
                            anchorPreviewImg.source = "image://frame/anchor-" + backend.frameCounter
                        }
                    }
                }

                Text {
                    visible: !calibScreen.anchorOverlayReady
                    anchors.centerIn: parent
                    text: "Overlay aanvragen..."
                    color: "#8b949e"
                    font.pixelSize: 14
                }
            }

            Column {
                Layout.fillHeight: true
                Layout.fillWidth: true
                spacing: 16

                Text {
                    text: "Schaalkalibratie — twee punten"
                    color: "#ffffff"
                    font.pixelSize: 18
                    font.weight: Font.Medium
                }

                Text {
                    id: anchorStatusText
                    text: "Overlay wordt berekend..."
                    color: "#8b949e"
                    font.pixelSize: 13
                    wrapMode: Text.WordWrap
                    width: parent.width
                }

                Text { text: "Ster A (ID)"; color: "#aaaaaa"; font.pixelSize: 13 }
                ComboBox {
                    width: parent.width
                    model: calibScreen.mapMarkerIds
                    onCurrentIndexChanged: {
                        if (currentIndex >= 0 && currentIndex < calibScreen.mapMarkerIds.length)
                            calibScreen.anchorIdA = calibScreen.mapMarkerIds[currentIndex]
                    }
                }

                Text { text: "Ster B (ID)"; color: "#aaaaaa"; font.pixelSize: 13 }
                ComboBox {
                    width: parent.width
                    model: calibScreen.mapMarkerIds
                    currentIndex: Math.min(1, calibScreen.mapMarkerIds.length - 1)
                    onCurrentIndexChanged: {
                        if (currentIndex >= 0 && currentIndex < calibScreen.mapMarkerIds.length)
                            calibScreen.anchorIdB = calibScreen.mapMarkerIds[currentIndex]
                    }
                }

                Text { text: "Afstand tussen A en B (cm)"; color: "#aaaaaa"; font.pixelSize: 13 }
                RowLayout {
                    width: parent.width; spacing: 8
                    TextField {
                        Layout.fillWidth: true
                        placeholderText: "bijv. 35,8"
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        validator: RegularExpressionValidator { regularExpression: /^[0-9]+([,.][0-9]{0,2})?$/ }
                        onTextChanged: calibScreen.anchorDistCm = calibScreen.parseDistanceCm(text)
                    }
                    Text { text: "cm"; color: "#aaaaaa"; font.pixelSize: 14 }
                }

                Rectangle {
                    width: parent.width; height: 50; radius: 7
                    color: confirmArea.containsMouse ? "#1c4a2e" : "#21262d"
                    opacity: (calibScreen.anchorIdA >= 0 &&
                              calibScreen.anchorIdB >= 0 &&
                              calibScreen.anchorIdA !== calibScreen.anchorIdB &&
                              calibScreen.anchorDistCm > 0) ? 1.0 : 0.4
                    border.color: "#2ea44f"; border.width: 1
                    Text { anchors.centerIn: parent; text: "Bevestig anker"; color: "#2ea44f"; font.pixelSize: 15 }
                    MouseArea {
                        id: confirmArea
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.PointingHandCursor
                        onClicked: calibScreen.confirmAnchor()
                    }
                }

                Text {
                    text: "Annuleren"
                    color: "#8b949e"
                    font.pixelSize: 13
                    MouseArea {
                        anchors.fill: parent
                        onClicked: calibScreen.methodChoice = ""
                    }
                }
            }
        }
    }

    Rectangle {
        visible: activeRun.length > 0
        anchors.fill: parent
        color: "#dd0e1117"
        z: 20

        Rectangle {
            anchors.centerIn: parent
            width: Math.min(parent.width - 64, 1180)
            height: Math.min(parent.height - 96, 760)
            radius: 14
            color: "#161b22"
            border.color: "#30363d"
            border.width: 1

            Item {
                anchors.fill: parent
                anchors.margins: 20

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 18

                    Text {
                        text: activeRun === "scale_ready" ? "Hoogtekalibratie starten"
                             : activeRun === "scale" ? "Hoogtekalibratie"
                             : "Sterrenkaartscan actief"
                        color: "#ffffff"
                        font.pixelSize: 28
                        font.weight: Font.Bold
                    }

                    Text {
                        Layout.fillWidth: true
                        wrapMode: Text.WordWrap
                        color: "#8b949e"
                        font.pixelSize: 15
                        text: {
                            if (activeRun === "scale_ready")
                                return "Klik pas op Start wanneer de opstelling goed gericht is. Daarna vraagt de wizard je om stil te houden en vervolgens exact te verschuiven."
                            if (activeRun === "scale") {
                                if (calibPromptMsg.length > 0)
                                    return "Volg de instructie hieronder en druk telkens op Verder om de volgende capture uit te voeren."
                                return "De camera wordt vrijgemaakt voor de hoogtemeting. Dat duurt enkele seconden."
                            }
                            return "Loop met de Pi rond tot de sterrenkaart voldoende bevestigd is. Klik 'Sla op als kaart' om de bevestigde sterren op te slaan."
                        }
                    }

                    Item {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        visible: activeRun === "map"

                        Rectangle {
                            anchors.fill: parent
                            radius: 12
                            color: "#0b0f14"
                            border.color: "#30363d"
                            border.width: 1

                            Image {
                                id: mapPreview
                                anchors.fill: parent
                                anchors.margins: 10
                                source: "image://frame/" + backend.frameCounter
                                fillMode: Image.PreserveAspectFit
                                cache: false
                            }

                            Canvas {
                                id: mapOverlayCanvas
                                anchors.fill: mapPreview

                                function drawPoints(ctx, points, strokeColor, fillColor, radius) {
                                    var drawW = mapPreview.paintedWidth
                                    var drawH = mapPreview.paintedHeight
                                    var ox = (width - drawW) / 2
                                    var oy = (height - drawH) / 2

                                    for (var i = 0; i < points.length; ++i) {
                                        var p = points[i]
                                        var nx = p[0]
                                        var ny = p[1]
                                        var x = ox + nx * drawW
                                        var y = oy + ny * drawH
                                        ctx.beginPath()
                                        ctx.arc(x, y, radius, 0, Math.PI * 2)
                                        ctx.lineWidth = 2
                                        ctx.strokeStyle = strokeColor
                                        ctx.stroke()
                                        if (fillColor.length > 0) {
                                            ctx.fillStyle = fillColor
                                            ctx.fill()
                                        }
                                    }
                                }

                                onPaint: {
                                    var ctx = getContext("2d")
                                    ctx.clearRect(0, 0, width, height)
                                    drawPoints(ctx, backend.scanDetectedPoints, "#ffd33d", "", 7)
                                    drawPoints(ctx, backend.scanConfirmedPoints, "#2ea44f", "rgba(46,164,79,0.22)", 10)
                                }
                            }

                            Rectangle {
                                anchors {
                                    left: parent.left
                                    right: parent.right
                                    bottom: parent.bottom
                                    margins: 10
                                }
                                radius: 8
                                color: "#00000099"
                                implicitHeight: mapHudCol.implicitHeight + 16

                                Column {
                                    id: mapHudCol
                                    anchors {
                                        left: parent.left
                                        right: parent.right
                                        margins: 10
                                        verticalCenter: parent.verticalCenter
                                    }
                                    spacing: 4

                                    Text {
                                        text: calibScreen.mapStatusText()
                                        color: "#ffffff"
                                        font.pixelSize: 15
                                    }
                                    Text {
                                        text: "Geel = detectie in huidig frame, groen = al bevestigd voor de kaart"
                                        color: "#c9d1d9"
                                        font.pixelSize: 13
                                    }
                                }
                            }
                        }
                    }

                    Rectangle {
                        visible: activeRun === "scale" && calibPromptMsg.length > 0
                        Layout.fillWidth: true
                        radius: 10
                        color: "#0e1117"
                        border.color: "#30363d"
                        border.width: 1
                        implicitHeight: scalePromptCol.implicitHeight + 24

                        Column {
                            id: scalePromptCol
                            anchors {
                                fill: parent
                                margins: 12
                            }
                            spacing: 8

                            Text {
                                text: "Stap " + calibPromptStep
                                color: "#2ea44f"
                                font.pixelSize: 14
                                font.weight: Font.Bold
                            }

                            Text {
                                width: parent.width
                                text: calibPromptMsg
                                color: "#ffffff"
                                font.pixelSize: 21
                                wrapMode: Text.WordWrap
                            }
                        }
                    }

                    Text {
                        visible: activeRun === "scale" && calibPromptMsg.length === 0
                        Layout.fillWidth: true
                        wrapMode: Text.WordWrap
                        text: "Wacht tot de hoogtekalibratie de eerste instructie toont."
                        color: "#ffffff"
                        font.pixelSize: 16
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 12

                        Item { Layout.fillWidth: true }

                        Button {
                            visible: activeRun === "scale_ready"
                            text: "Annuleer"
                            onClicked: calibScreen.closeRunOverlay()
                        }

                        Button {
                            visible: activeRun === "scale_ready"
                            text: "Start"
                            onClicked: calibScreen.beginScaleMeasurement()
                        }

                        Button {
                            visible: activeRun === "scale" && calibPromptMsg.length > 0
                            text: "Verder"
                            onClicked: {
                                calibPromptMsg = ""
                                backend.calibConfirm()
                            }
                        }

                        Button {
                            visible: activeRun === "map"
                            text: "Sla op als kaart"
                            onClicked: backend.stopBuildMap()
                        }
                    }
                }
            }
        }
    }
}

// ui/qml/TrackScreen.qml
// Live tracking screen.
// Shows: fullscreen camera preview, pose panel (top-right),
//        comm panel (bottom-right), IMU panel (top-left), Stop button (bottom-left).
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: trackScreen
    anchors.fill: parent
    property bool stopRequested: false
    property bool lostPopupShown: false
    property bool trackingOn: false
    property bool imuOn: false
    property bool gridOn: true

    function applyToggles() {
        if (trackingOn && imuOn) {
            backend.startTrack()
        } else if (trackingOn && !imuOn) {
            backend.startTrackVisionOnly()
        } else if (!trackingOn && imuOn) {
            backend.startImuOnly()
        } else {
            backend.stopTracking()
        }
    }

    function stopAll() {
        trackingOn = false
        imuOn = false
        backend.stopTracking()
    }

    function leaveTracking(navigateHome) {
        if (!trackScreen.stopRequested) {
            trackScreen.stopRequested = true
            trackScreen.stopAll()
        }
        if (navigateHome) root.navigateBack()
    }

    Component.onCompleted: {
        backend.requestCurrentMap()
        trackingOn = true
        imuOn = true
        applyToggles()
    }

    Connections {
        target: backend
        function onPoseUpdated() { mapCanvas.requestPaint() }
        function onStarMapChanged() { mapCanvas.requestPaint() }
        function onStatusUpdated() {
            trackScreen.lostPopupShown =
                trackScreen.trackingOn && backend.trackingLost && !trackScreen.stopRequested
            if (backend.imuOnlyActive && !trackScreen.imuOn)
                trackScreen.imuOn = true
        }
        function onSocketConnectedChanged() {
            if (backend.socketConnected) backend.requestCurrentMap()
        }
    }

    // --- Camera preview (fullscreen background) ---
    Image {
        anchors.fill: parent
        source: "image://frame/" + backend.frameCounter
        fillMode: Image.PreserveAspectCrop
        cache: false

        Rectangle {
            anchors.fill: parent
            color: "#0e1117"
            visible: parent.status !== Image.Ready
        }
    }

    // Semi-transparent panel background helper component
    component PanelBg: Rectangle {
        color: "#b2161b22"   // ~70% opacity dark panel
        radius: 10
        border.color: "#30363d"
        border.width: 1
    }

    // --- Pose panel (top-right) ---
    PanelBg {
        id: posePanel
        anchors {
            top:   parent.top
            right: parent.right
            margins: 20
        }
        width: 220
        height: poseCol.implicitHeight + 24

        Column {
            id: poseCol
            anchors { top: parent.top; left: parent.left; right: parent.right; margins: 14 }
            spacing: 6
            topPadding: 4

            Text {
                text: trackScreen.trackingOn
                    ? (backend.trackingActive && !backend.trackingLost ? "● Tracking"
                                                                       : "● Lokaliseren…")
                    : ""
                color: backend.trackingActive && !backend.trackingLost ? "#2ea44f" : "#d29922"
                font.pixelSize: 13
                font.weight: Font.Medium
                visible: trackScreen.trackingOn
            }

            Text { text: "Positie"; color: "#aaaaaa"; font.pixelSize: 13; font.weight: Font.Medium }

            Repeater {
                model: [
                    { label: "X",     value: backend.posX.toFixed(4) + " m" },
                    { label: "Y",     value: backend.posY.toFixed(4) + " m" },
                    { label: "Z",     value: backend.posZ.toFixed(4) + " m" },
                ]
                RowLayout {
                    width: poseCol.width
                    Text { text: modelData.label; color: "#aaaaaa"; font.pixelSize: 14; Layout.preferredWidth: 28 }
                    Text { text: modelData.value; color: "#ffffff"; font.pixelSize: 18; font.weight: Font.Medium; font.family: "monospace" }
                }
            }

            Rectangle { width: parent.width; height: 1; color: "#30363d" }

            Text { text: "Ori\u00ebntering"; color: "#aaaaaa"; font.pixelSize: 13; font.weight: Font.Medium }

            Repeater {
                model: [
                    { label: "R",     value: backend.roll.toFixed(2)  + "\u00b0" },
                    { label: "P",     value: backend.pitch.toFixed(2) + "\u00b0" },
                    { label: "Y",     value: backend.yaw.toFixed(2) + "\u00b0" },
                ]
                RowLayout {
                    width: poseCol.width
                    Text { text: modelData.label; color: "#aaaaaa"; font.pixelSize: 14; Layout.preferredWidth: 28 }
                    Text { text: modelData.value; color: "#ffffff"; font.pixelSize: 18; font.weight: Font.Medium; font.family: "monospace" }
                }
            }
        }
    }

    // ── ESKF diagnostics panel ────────────────────────────────────────────────
    PanelBg {
        id: eskfDiagPanel
        anchors {
            top: posePanel.bottom
            right: parent.right
            margins: 20
            topMargin: 8
        }
        width: 220
        height: diagCol.implicitHeight + 16
        visible: trackScreen.imuOn

        Column {
            id: diagCol
            anchors { fill: parent; margins: 10 }
            spacing: 4

            Text { text: "ESKF"; color: "#aaaaaa"; font.pixelSize: 11; font.weight: Font.Medium }

            Repeater {
                model: [
                    { label: "Vel",  value: backend.eskfVelNorm.toFixed(3) + " m/s" },
                    { label: "Accβ", value: backend.eskfBaNorm.toFixed(3) + " m/s²" },
                    { label: "Gyrβ", value: backend.eskfBgNorm.toFixed(3) + " rad/s" },
                ]
                delegate: RowLayout {
                    width: diagCol.width
                    Text { text: modelData.label; color: "#aaaaaa"; font.pixelSize: 11; Layout.preferredWidth: 36 }
                    Text { text: modelData.value; color: "#ffffff"; font.pixelSize: 12; font.family: "monospace" }
                }
            }
        }
    }

    // ── Tracking controls — two independent toggles ───────────────────────
    PanelBg {
        id: controlPanel
        anchors { top: parent.top; left: parent.left; margins: 20 }
        width: 220
        height: ctrlCol.implicitHeight + 24

        Column {
            id: ctrlCol
            anchors { fill: parent; margins: 14 }
            spacing: 14

            Text {
                text: "Tracking"
                color: "#aaaaaa"
                font.pixelSize: 13
                font.weight: Font.Medium
            }

            Row {
                spacing: 12
                width: parent.width

                Switch {
                    id: starSwitch
                    checked: trackScreen.trackingOn
                    onToggled: {
                        trackScreen.trackingOn = checked
                        trackScreen.applyToggles()
                    }
                }
                Text {
                    text: "Stertracking"
                    color: "#ffffff"
                    font.pixelSize: 15
                    anchors.verticalCenter: parent.verticalCenter
                }
            }

            Row {
                spacing: 12
                width: parent.width

                Switch {
                    id: imuSwitch
                    checked: trackScreen.imuOn
                    onToggled: {
                        trackScreen.imuOn = checked
                        trackScreen.applyToggles()
                    }
                }
                Row {
                    spacing: 6
                    anchors.verticalCenter: parent.verticalCenter
                    Text {
                        text: "IMU"
                        color: "#ffffff"
                        font.pixelSize: 15
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Rectangle {
                        width: 8; height: 8; radius: 4
                        color: backend.imuOk ? "#2ea44f" : "#cf222e"
                        anchors.verticalCenter: parent.verticalCenter
                        visible: trackScreen.imuOn
                    }
                }
            }

            Row {
                spacing: 12
                width: parent.width

                Switch {
                    id: gridSwitch
                    checked: trackScreen.gridOn
                    onToggled: {
                        trackScreen.gridOn = checked
                        backend.setGridVisible(checked)
                    }
                }
                Text {
                    text: "Raster"
                    color: "#ffffff"
                    font.pixelSize: 15
                    anchors.verticalCenter: parent.verticalCenter
                }
            }

            Text {
                text: {
                    if (trackScreen.trackingOn && trackScreen.imuOn)
                        return "Visie + IMU fusie"
                    if (trackScreen.trackingOn)
                        return "Visie-only modus"
                    if (trackScreen.imuOn)
                        return "IMU-only modus"
                    return "Gestopt"
                }
                color: "#8b949e"
                font.pixelSize: 12
            }
        }
    }

    // --- Comm panel (bottom-right) ---
    PanelBg {
        id: commPanel
        anchors {
            bottom: parent.bottom
            right:  parent.right
            margins: 20
        }
        width: 220
        height: commCol.implicitHeight + 24

        Column {
            id: commCol
            anchors { top: parent.top; left: parent.left; right: parent.right; margins: 14 }
            spacing: 6
            topPadding: 4

            Text { text: "FreeD UDP"; color: "#aaaaaa"; font.pixelSize: 13; font.weight: Font.Medium }

            Text {
                text: backend.freedIp + ":" + backend.freedPort
                color: backend.freedEnabled ? "#ffffff" : "#8b949e"
                font.pixelSize: 14; font.family: "monospace"
            }

            Row {
                spacing: 8
                Rectangle {
                    width: 10; height: 10; radius: 5
                    anchors.verticalCenter: parent.verticalCenter
                    color: !backend.freedEnabled ? "#8b949e" : (backend.freedConnected ? "#2ea44f" : "#cf222e")
                }
                Text {
                    text: !backend.freedEnabled ? "Uit" : backend.freedHz.toFixed(1) + " Hz"
                    color: backend.freedEnabled ? "#ffffff" : "#8b949e"
                    font.pixelSize: 15; font.weight: Font.Medium
                    anchors.verticalCenter: parent.verticalCenter
                }
            }

            Text {
                text: backend.freedEnabled
                    ? "Latentie: " + backend.freedLatency.toFixed(1) + " ms"
                    : "Uitvoer gepauzeerd"
                color: "#aaaaaa"; font.pixelSize: 13
            }

            Text {
                text: "Vision: " + backend.visionHz.toFixed(1) + " Hz"
                color: "#aaaaaa"; font.pixelSize: 13; font.family: "monospace"
            }
            Text {
                text: "Pose: " + backend.poseHz.toFixed(1) + " Hz"
                color: "#aaaaaa"; font.pixelSize: 13; font.family: "monospace"
            }
        }
    }

    // --- Stop button (bottom-left) ---
    Rectangle {
        id: stopButton
        anchors {
            bottom: parent.bottom
            left:   parent.left
            margins: 20
        }
        width: 120; height: 46; radius: 8
        color: stopArea.containsMouse ? "#a21d2e" : "#cf222e"
        Behavior on color { ColorAnimation { duration: 100 } }

        Text {
            anchors.centerIn: parent
            text: "Stop"
            color: "#ffffff"; font.pixelSize: 18; font.weight: Font.Bold
        }
        MouseArea {
            id: stopArea
            anchors.fill: parent
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor
            onClicked: {
                trackScreen.leaveTracking(true)
            }
        }
    }

    // --- Star map card (bottom center) ---
    PanelBg {
        id: mapPanel
        anchors {
            left: stopButton.right
            right: commPanel.left
            bottom: parent.bottom
            leftMargin: 20
            rightMargin: 20
            bottomMargin: 20
        }
        height: 160
        visible: width > 280

        property var rollBuf:  new Array(200).fill(0)
        property var pitchBuf: new Array(200).fill(0)
        property var yawBuf:   new Array(200).fill(0)
        property int head: 0

        Connections {
            target: backend
            function onPoseUpdated() {
                mapPanel.rollBuf[mapPanel.head]  = backend.roll
                mapPanel.pitchBuf[mapPanel.head] = backend.pitch
                mapPanel.yawBuf[mapPanel.head]   = backend.yaw
                mapPanel.head = (mapPanel.head + 1) % 200
                eulerCanvas.requestPaint()
            }
        }

        Column {
            id: mapCol
            anchors { fill: parent; margins: 14 }
            spacing: 8

            RowLayout {
                width: parent.width
                spacing: 12

                Text {
                    text: "Star map"
                    color: "#aaaaaa"
                    font.pixelSize: 13
                    font.weight: Font.Medium
                }

                Text {
                    text: backend.currentMapName.length > 0
                        ? backend.currentMapName
                        : "Geen kaart geladen"
                    color: "#ffffff"
                    font.pixelSize: 13
                    font.family: "monospace"
                    elide: Text.ElideMiddle
                    Layout.fillWidth: true
                }

                Text {
                    text: backend.posX.toFixed(2) + ", " + backend.posY.toFixed(2) + " m"
                    color: "#aaaaaa"
                    font.pixelSize: 12
                    font.family: "monospace"
                }
            }

            Row {
                width: parent.width
                height: parent.height - y

                Canvas {
                    id: eulerCanvas
                    width: parent.width / 2
                    height: parent.height

                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.clearRect(0, 0, width, height)

                        ctx.fillStyle = "#8b949e"
                        ctx.font = "11px sans-serif"
                        ctx.fillText("Roll / Pitch / Yaw  [10 s]", 4, 12)

                        var N = 200
                        var maxDeg = 90

                        function drawLine(buf, color) {
                            ctx.beginPath()
                            ctx.strokeStyle = color
                            ctx.lineWidth = 1.2
                            for (var i = 0; i < N; i++) {
                                var idx = (mapPanel.head + i) % N
                                var x = i / (N - 1) * width
                                var y = height / 2 - (buf[idx] / maxDeg) * (height / 2 - 16)
                                if (i === 0) ctx.moveTo(x, y)
                                else         ctx.lineTo(x, y)
                            }
                            ctx.stroke()
                        }

                        ctx.beginPath()
                        ctx.strokeStyle = "#30363d"
                        ctx.lineWidth = 0.5
                        ctx.moveTo(0, height / 2)
                        ctx.lineTo(width, height / 2)
                        ctx.stroke()

                        drawLine(mapPanel.rollBuf,  "#e06c75")
                        drawLine(mapPanel.pitchBuf, "#98c379")
                        drawLine(mapPanel.yawBuf,   "#61afef")

                        ctx.font = "10px sans-serif"
                        ctx.fillStyle = "#e06c75"; ctx.fillText("R", 4, height - 4)
                        ctx.fillStyle = "#98c379"; ctx.fillText("P", 18, height - 4)
                        ctx.fillStyle = "#61afef"; ctx.fillText("Y", 32, height - 4)
                    }
                }

                Canvas {
                    id: mapCanvas
                    width: parent.width / 2
                    height: parent.height
                    antialiasing: true

                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.reset()
                        ctx.clearRect(0, 0, width, height)

                        var points = backend.currentMapPoints
                        if (!points || points.length === 0) {
                            ctx.fillStyle = "#8b949e"
                            ctx.font = "13px sans-serif"
                            ctx.fillText("Geen star map data", 12, Math.round(height / 2))
                            return
                        }

                        var minX = backend.posX
                        var maxX = backend.posX
                        var minY = backend.posY
                        var maxY = backend.posY
                        for (var i = 0; i < points.length; ++i) {
                            var p = points[i]
                            minX = Math.min(minX, p.x)
                            maxX = Math.max(maxX, p.x)
                            minY = Math.min(minY, p.y)
                            maxY = Math.max(maxY, p.y)
                        }

                        var spanX = Math.max(0.1, maxX - minX)
                        var spanY = Math.max(0.1, maxY - minY)
                        var pad = 12
                        var usableW = Math.max(1, width - pad * 2)
                        var usableH = Math.max(1, height - pad * 2)
                        var scale = Math.min(usableW / spanX, usableH / spanY)
                        var drawW = spanX * scale
                        var drawH = spanY * scale
                        var ox = Math.round((width - drawW) / 2)
                        var oy = Math.round((height - drawH) / 2)

                        function sx(x) { return ox + (x - minX) * scale }
                        function sy(y) { return oy + drawH - (y - minY) * scale }

                        ctx.strokeStyle = "#30363d"
                        ctx.lineWidth = 1
                        ctx.strokeRect(ox, oy, drawW, drawH)

                        ctx.fillStyle = "#f0f6fc"
                        for (var j = 0; j < points.length; ++j) {
                            var sp = points[j]
                            ctx.beginPath()
                            ctx.arc(sx(sp.x), sy(sp.y), 2.2, 0, Math.PI * 2)
                            ctx.fill()
                        }

                        var px = sx(backend.posX)
                        var py = sy(backend.posY)
                        var yawRad = (backend.yaw - 90) * Math.PI / 180.0
                        var arrowLen = 18
                        var ax = px + Math.cos(yawRad) * arrowLen
                        var ay = py + Math.sin(yawRad) * arrowLen

                        ctx.strokeStyle = "#ff4d5a"
                        ctx.lineWidth = 2
                        ctx.beginPath()
                        ctx.moveTo(px, py)
                        ctx.lineTo(ax, ay)
                        ctx.stroke()

                        ctx.fillStyle = "#ff4d5a"
                        ctx.beginPath()
                        ctx.arc(px, py, 5, 0, Math.PI * 2)
                        ctx.fill()

                        var side = 5
                        ctx.save()
                        ctx.translate(ax, ay)
                        ctx.rotate(yawRad)
                        ctx.beginPath()
                        ctx.moveTo(7, 0)
                        ctx.lineTo(-side, -side)
                        ctx.lineTo(-side, side)
                        ctx.closePath()
                        ctx.fill()
                        ctx.restore()
                    }
                }
            }
        }
    }

    // --- Tracking lost warning overlay ---
    Rectangle {
        visible: trackScreen.imuOn && !backend.imuOk
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: 80
        width: lostText.implicitWidth + 32
        height: 42; radius: 8
        color: "#d29922"

        Text {
            id: lostText
            anchors.centerIn: parent
            text: "IMU niet beschikbaar"
            color: "#0e1117"; font.pixelSize: 15; font.weight: Font.Medium
        }
    }

    Rectangle {
        visible: trackScreen.lostPopupShown
        anchors.fill: parent
        color: "#990e1117"
        z: 50

        Rectangle {
            anchors.centerIn: parent
            width: Math.min(parent.width - 64, 460)
            height: lostPopupCol.implicitHeight + 40
            radius: 14
            color: "#161b22"
            border.color: "#d29922"
            border.width: 1

            Column {
                id: lostPopupCol
                anchors {
                    left: parent.left
                    right: parent.right
                    top: parent.top
                    margins: 20
                }
                spacing: 14

                Text {
                    text: "Tracking verloren"
                    color: "#ffffff"
                    font.pixelSize: 24
                    font.weight: Font.Bold
                }

                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    text: "Er worden minstens een seconde niet genoeg sterren gezien voor betrouwbare tracking. De laatst geldige positie blijft zichtbaar; tracking herneemt automatisch zodra de sterren opnieuw betrouwbaar gevonden worden."
                    color: "#aaaaaa"
                    font.pixelSize: 15
                }

                Rectangle {
                    width: parent.width
                    height: 42
                    radius: 8
                    color: "#21262d"
                    anchors.horizontalCenter: parent.horizontalCenter
                    border.color: "#30363d"
                    border.width: 1

                    Text {
                        anchors.centerIn: parent
                        text: "Wachten op herlocalisatie..."
                        color: "#d29922"
                        font.pixelSize: 14
                        font.weight: Font.Bold
                    }
                }
            }
        }
    }
}

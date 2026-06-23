cmake_minimum_required(VERSION 3.16)
project(startracker-ui CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick Qml Network)

qt_add_executable(startracker-ui
    main.cpp
    FrameProvider.cpp
    BackendModel.cpp
    SharedMemoryReader.cpp
    SocketClient.cpp
)

qt_add_qml_module(startracker-ui
    URI StarTrackerUI
    VERSION 1.0
    QML_FILES
        qml/main.qml
        qml/DetectScreen.qml
        qml/CalibrateScreen.qml
        qml/TrackScreen.qml
        qml/SettingsScreen.qml
        qml/DetectorDebugScreen.qml
)

target_include_directories(startracker-ui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared
)

target_link_libraries(startracker-ui PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Quick
    Qt6::Qml
    Qt6::Network
    $<$<PLATFORM_ID:Linux>:rt>
    pthread
)

cmake_minimum_required(VERSION 3.16)
project(startracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Zelfde Pi-vriendelijke flags als cpp_bench
add_compile_options(-O2 -pipe)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ── Object libraries per module (vermijdt .a overhead, snelle links) ─────────

add_library(st_util OBJECT
    src/util/Intrinsics.cpp
    src/util/ConfigLoader.cpp)
target_include_directories(st_util PRIVATE src)
target_link_libraries(st_util PRIVATE ${OpenCV_LIBS})

add_library(st_detector OBJECT
    src/detector/StarDetector.cpp
    src/detector/StarDetectorLight.cpp)
target_include_directories(st_detector PRIVATE src)
target_link_libraries(st_detector PRIVATE ${OpenCV_LIBS})

add_library(st_localiser OBJECT
    src/localiser/StarIndex.cpp
    src/localiser/Localiser.cpp
    src/localiser/StarIndex3D.cpp
    src/localiser/Localiser3D.cpp)
target_include_directories(st_localiser PRIVATE src)
target_link_libraries(st_localiser PRIVATE ${OpenCV_LIBS})

add_library(st_tracker OBJECT
    src/tracker/Tracker.cpp
    src/tracker/Tracker3D.cpp)
target_include_directories(st_tracker PRIVATE src)
target_link_libraries(st_tracker PRIVATE ${OpenCV_LIBS})

add_library(st_io OBJECT
    src/io/CameraReader.cpp)
target_include_directories(st_io PRIVATE src)
target_link_libraries(st_io PRIVATE ${OpenCV_LIBS})

add_library(st_mapper OBJECT
    src/mapper/MapBuilder.cpp)
target_include_directories(st_mapper PRIVATE src)
target_link_libraries(st_mapper PRIVATE ${OpenCV_LIBS})

# ── Pose estimator (solvePnP / IPPE) ─────────────────────────────────────────
add_library(st_pose OBJECT
    src/pose/PoseEstimator.cpp)
target_include_directories(st_pose PRIVATE src)
target_link_libraries(st_pose PRIVATE ${OpenCV_LIBS})

# ── IPC layer (shared memory + socket server) ─────────────────────────────────
add_library(st_ipc OBJECT
    src/ipc/SharedMemoryServer.cpp
    src/ipc/SocketServer.cpp)
target_include_directories(st_ipc PRIVATE src shared)
target_link_libraries(st_ipc PRIVATE ${OpenCV_LIBS})

# ── IMU layer (BNO085 SHTP reader, FreeD UDP encoder) ────────────────────────
add_library(st_imu OBJECT
    src/imu/ImuReader.cpp
    src/ipc/FreeD.cpp)
target_include_directories(st_imu PRIVATE src)

# ── ESKF (Error-State Kalman Filter, Eigen3) ──────────────────────────────────
add_library(st_eskf OBJECT
    src/imu/eskf/ESKF.cpp)
target_include_directories(st_eskf PRIVATE src)
target_link_libraries(st_eskf PRIVATE Eigen3::Eigen)

# ── Worker classes (Core 1 / 2 / 3) ──────────────────────────────────────────
add_library(st_workers OBJECT
    src/workers/SharedState.cpp
    src/workers/CameraWorker.cpp
    src/workers/TrackingWorker.cpp
    src/workers/IMUWorker.cpp)
target_include_directories(st_workers PRIVATE src shared)
target_link_libraries(st_workers PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)

# ── Eindexecutable ────────────────────────────────────────────────────────────

add_executable(startracker
    src/main.cpp
    $<TARGET_OBJECTS:st_util>
    $<TARGET_OBJECTS:st_detector>
    $<TARGET_OBJECTS:st_localiser>
    $<TARGET_OBJECTS:st_tracker>
    $<TARGET_OBJECTS:st_io>
    $<TARGET_OBJECTS:st_mapper>
    $<TARGET_OBJECTS:st_ipc>
    $<TARGET_OBJECTS:st_imu>
    $<TARGET_OBJECTS:st_pose>
    $<TARGET_OBJECTS:st_eskf>
    $<TARGET_OBJECTS:st_workers>)

target_include_directories(startracker PRIVATE src shared)
# rt is Linux-only (POSIX shared memory via /dev/shm); not needed on macOS.
target_link_libraries(startracker PRIVATE ${OpenCV_LIBS} pthread Eigen3::Eigen
    $<$<PLATFORM_ID:Linux>:rt>)

# ── Debug-tool: live detectie-visualisatie ────────────────────────────────────
add_executable(debug_detect src/debug_detect.cpp)
target_include_directories(debug_detect PRIVATE src)
target_link_libraries(debug_detect PRIVATE
    st_detector st_io st_util
    ${OpenCV_LIBS})

add_executable(capture_frames src/capture_frames.cpp)
target_include_directories(capture_frames PRIVATE src)
target_link_libraries(capture_frames PRIVATE
    st_io st_util
    ${OpenCV_LIBS})

add_executable(bench_detect src/bench_detect.cpp)
target_include_directories(bench_detect PRIVATE src)
target_link_libraries(bench_detect PRIVATE
    st_detector st_util
    ${OpenCV_LIBS})


# ── Frontend (Qt6 kiosk UI) ───────────────────────────────────────────────────
option(BUILD_UI "Build the Qt6 frontend UI" ON)
if(BUILD_UI)
    add_subdirectory(ui)
endif()

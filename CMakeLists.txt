cmake_minimum_required(VERSION 3.16)
project(obstacle_avoidance VERSION 2.0.0 LANGUAGES CXX)

# ── C++ standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json for clangd / IDE tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ── Compiler flags ────────────────────────────────────────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-g -O0 -fsanitize=address,undefined)
        add_link_options(-fsanitize=address,undefined)
    else()
        add_compile_options(-O2)
    endif()
elseif(MSVC)
    add_compile_options(/W4 /WX-)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────

find_package(MAVSDK REQUIRED)

find_package(OpenCV 4 REQUIRED
    COMPONENTS core imgproc video highgui videoio)

find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    )
    FetchContent_MakeAvailable(nlohmann_json)
    message(STATUS "nlohmann/json fetched via FetchContent")
else()
    message(STATUS "nlohmann/json found: ${nlohmann_json_VERSION}")
endif()

find_package(Threads REQUIRED)

# ── Sources ───────────────────────────────────────────────────────────────────

set(SOURCES
    # Entry point
    src/main.cpp

    # Core (non-template headers only; .cpp files listed here)
    src/core/mavsdk_connection.cpp

    # Navigation
    src/navigation/rotation_tracker.cpp

    # Retained internal implementations (wrapped by services)
    src/camera/camera_interface.cpp
    src/detection/object_detector.cpp

    # Services
    src/services/camera_service.cpp
    src/services/detection_service.cpp
    src/services/depth_service.cpp
    src/services/telemetry_service.cpp
    src/services/sync_service.cpp
    src/services/avoidance_planner_service.cpp
    src/services/pid_service.cpp
    src/services/command_service.cpp
    src/services/flight_manager_service.cpp
)

add_executable(obstacle_avoidance ${SOURCES})

# ── Include dirs ──────────────────────────────────────────────────────────────
target_include_directories(obstacle_avoidance PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

# ── Link libraries ────────────────────────────────────────────────────────────
target_link_libraries(obstacle_avoidance PRIVATE
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
    Threads::Threads
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS obstacle_avoidance DESTINATION bin)
install(FILES config/params.json DESTINATION etc/obstacle_avoidance)

# ── Summary ───────────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "=== obstacle_avoidance v2 (micro-service) ===")
message(STATUS "  OpenCV  : ${OpenCV_VERSION}")
message(STATUS "  Build   : ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++     : ${CMAKE_CXX_STANDARD}")
message(STATUS "=============================================")

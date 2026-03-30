cmake_minimum_required(VERSION 3.24)
project(simulate_u1406330 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SIMULATOR_BUILD_TESTS "Build simulator tests" ON)
option(SIMULATOR_BUILD_VISUAL_APP "Build the SDL3 visualizer" ON)

if(EXISTS "${CMAKE_SOURCE_DIR}/vcpkg_installed/x64-windows/share/sdl3/SDL3Config.cmake")
    list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/vcpkg_installed/x64-windows/share")
endif()

add_library(sim_core
    src/Scenario.cpp
    src/SceneCsv.cpp
    src/Simulation.cpp
)
target_include_directories(sim_core PUBLIC include)
target_compile_features(sim_core PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(sim_core PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(sim_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

find_package(SDL3 CONFIG QUIET)

if(SIMULATOR_BUILD_VISUAL_APP)
    if(SDL3_FOUND)
        add_executable(simulator src/main.cpp)
        target_link_libraries(simulator PRIVATE sim_core SDL3::SDL3)
        if(WIN32)
            get_filename_component(SDL3_ROOT "${SDL3_DIR}/../.." ABSOLUTE)
            set(SDL3_RUNTIME_DLL "${SDL3_ROOT}/bin/SDL3.dll")
            add_custom_command(
                TARGET simulator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${SDL3_RUNTIME_DLL}"
                    $<TARGET_FILE_DIR:simulator>
            )
        endif()
    else()
        message(WARNING "SDL3 was not found. The simulator visualizer target will not be built.")
    endif()
endif()

if(SDL3_FOUND)
    add_library(sim_color_tools
        src/ImageSampling.cpp
        src/SceneColorize.cpp
    )
    target_include_directories(sim_color_tools PUBLIC include)
    target_link_libraries(sim_color_tools PUBLIC sim_core SDL3::SDL3)

    add_executable(scene_colorize src/SceneColorizeMain.cpp)
    target_link_libraries(scene_colorize PRIVATE sim_color_tools)
    if(WIN32)
        get_filename_component(SDL3_ROOT "${SDL3_DIR}/../.." ABSOLUTE)
        set(SDL3_RUNTIME_DLL "${SDL3_ROOT}/bin/SDL3.dll")
        add_custom_command(
            TARGET scene_colorize POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SDL3_RUNTIME_DLL}"
                $<TARGET_FILE_DIR:scene_colorize>
        )
    endif()
endif()

add_executable(physics_validation src/ValidationMain.cpp)
target_link_libraries(physics_validation PRIVATE sim_core)

if(SIMULATOR_BUILD_TESTS)
    enable_testing()
    add_executable(physics_tests tests/SimulationTests.cpp)
    target_link_libraries(physics_tests PRIVATE sim_core)
    add_test(NAME physics.deterministic_replay COMMAND physics_tests deterministic_replay)
    add_test(NAME physics.container_integrity COMMAND physics_tests container_integrity)
    add_test(NAME physics.container_is_square COMMAND physics_tests container_is_square)
    add_test(NAME physics.balls_settle_in_box COMMAND physics_tests balls_settle_in_box)
    add_test(NAME physics.fast_wall_containment COMMAND physics_tests fast_wall_containment)
    add_test(NAME physics.csv_roundtrip COMMAND physics_tests csv_roundtrip)
    add_test(NAME physics.csv_preserves_walls COMMAND physics_tests csv_preserves_walls)
    add_test(
        NAME physics.csv_roundtrip_preserves_scene_metadata
        COMMAND physics_tests csv_roundtrip_preserves_scene_metadata
    )
    if(SDL3_FOUND)
        add_executable(physics_image_tests tests/ImageSamplingTests.cpp)
        target_link_libraries(physics_image_tests PRIVATE sim_color_tools)
        add_test(NAME physics.image_sampling COMMAND physics_image_tests)
    endif()
    add_test(
        NAME physics.validation_container
        COMMAND physics_validation
                --scenario container
                --balls 250
                --seed 1337
                --restitution 0.25
                --steps 300
                --dump-every 0
                --max-penetration 12
                --require-no-escape-every-frame
    )
endif()

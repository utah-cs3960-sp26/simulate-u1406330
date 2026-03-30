cmake_minimum_required(VERSION 3.24)
project(simulate_u1406330 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SIMULATOR_BUILD_TESTS "Build simulator tests" ON)
option(SIMULATOR_BUILD_VISUAL_APP "Build the SDL3 visualizer" ON)

add_library(sim_core
    src/Scenario.cpp
    src/Simulation.cpp
)
target_include_directories(sim_core PUBLIC include)
target_compile_features(sim_core PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(sim_core PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(sim_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

if(SIMULATOR_BUILD_VISUAL_APP)
    find_package(SDL3 CONFIG QUIET)
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

add_executable(physics_validation src/ValidationMain.cpp)
target_link_libraries(physics_validation PRIVATE sim_core)

if(SIMULATOR_BUILD_TESTS)
    enable_testing()
    add_executable(physics_tests tests/SimulationTests.cpp)
    target_link_libraries(physics_tests PRIVATE sim_core)
    add_test(NAME physics.deterministic_replay COMMAND physics_tests deterministic_replay)
    add_test(NAME physics.restitution_energy_and_packing COMMAND physics_tests restitution_energy_and_packing)
    add_test(NAME physics.container_integrity COMMAND physics_tests container_integrity)
    add_test(NAME physics.gap_integrity COMMAND physics_tests gap_integrity)
    add_test(NAME physics.fast_wall_containment COMMAND physics_tests fast_wall_containment)
    add_test(
        NAME physics.validation_container
        COMMAND physics_validation
                --scenario container
                --balls 1000
                --seed 1337
                --restitution 0.25
                --steps 600
                --dump-every 200
                --max-penetration 0.02
                --require-no-escape-every-frame
    )
endif()

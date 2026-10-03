set(A320_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../unreal/A320Sim/Content/JSBSim")

set(A320_SOUNDS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../unreal/A320Sim/Content/Sounds")

add_executable(a320_unit_tests test_main.cpp test_geometry.cpp test_systems.cpp test_audio.cpp)
target_link_libraries(a320_unit_tests PRIVATE a320core_static libJSBSim)
target_compile_options(a320_unit_tests PRIVATE ${A320_WARNINGS})
target_compile_definitions(a320_unit_tests PRIVATE A320_SOUNDS_DIR="${A320_SOUNDS_DIR}")
add_test(NAME unit COMMAND a320_unit_tests)

# Flight tests drive the real JSBSim A320 through the C API, the same entry point Unreal uses.
add_executable(a320_flight_tests test_main.cpp test_flight.cpp test_autopilot.cpp ../src/CApi.cpp)
target_link_libraries(a320_flight_tests PRIVATE a320core_static libJSBSim)
target_compile_options(a320_flight_tests PRIVATE ${A320_WARNINGS})
target_compile_definitions(a320_flight_tests PRIVATE A320_DATA_DIR="${A320_DATA_DIR}")
add_test(NAME flight COMMAND a320_flight_tests)

cmake_minimum_required(VERSION 3.10)

# The units under test are compiled directly into the test binary rather than
# depending on the irTactileSerialBridge target, so this suite never needs
# irtactile-shm, shmlog or windows.h. That is why the stream-facing signatures
# take plain values (samplesPerBlock, a rate, a TX queue size) instead of a
# Reader and a SerialPort.
add_executable(irTactileSerialBridge_tests
    doctest_main.cpp
    SerialProtocol_test.cpp
    BridgeConfig_test.cpp
    WireParams_test.cpp
    Throttle_test.cpp
    TuningFrame_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/SerialProtocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/BridgeConfig.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/WireParams.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/TuningFrame.cpp
)

target_include_directories(irTactileSerialBridge_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(irTactileSerialBridge_tests PRIVATE doctest::doctest)
target_compile_features(irTactileSerialBridge_tests PRIVATE cxx_std_20)

# Warning flags and NOMINMAX come from the root add_compile_options /
# add_compile_definitions, which propagate into this subdirectory.

list(APPEND CMAKE_MODULE_PATH "${doctest_SOURCE_DIR}/scripts/cmake")
include(doctest)
doctest_discover_tests(irTactileSerialBridge_tests)

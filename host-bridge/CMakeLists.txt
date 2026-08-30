cmake_minimum_required(VERSION 3.10)
project(irTactileSerialBridge)

option(HOST_BRIDGE_BUILD_TESTS "Build the host-bridge test suite (doctest)" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to Release as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(MSVC)
    add_compile_options(/W3 /Zc:preprocessor /utf-8)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()

# -----------------------------------------------------------------------------
# irtactile-shm dependency
#
# The shared-memory stream contract lives in the public irtactile-shm
# repository (bindings/cpp): header-only, zero link deps. FetchContent tracks
# main - no tagged release exists yet - with a local checkout escape hatch.
# -----------------------------------------------------------------------------
set(IRTACTILE_SHM_LOCAL_DIR "" CACHE PATH "Path to a local irtactile-shm checkout")
if(IRTACTILE_SHM_LOCAL_DIR)
    add_subdirectory(${IRTACTILE_SHM_LOCAL_DIR}/bindings/cpp irtactile-shm)
else()
    include(FetchContent)
    # SOURCE_SUBDIR is required: the repo root is spec/ + bindings/ and carries
    # no CMakeLists.txt of its own, so without it MakeAvailable would populate
    # the content and silently define no targets.
    FetchContent_Declare(irtactile_shm
        GIT_REPOSITORY https://github.com/jSchmidt-design/irtactile-shm.git
        GIT_TAG        main
        SOURCE_SUBDIR  bindings/cpp)
    FetchContent_MakeAvailable(irtactile_shm)
endif()

# -----------------------------------------------------------------------------
# shmlog dependency
#
# Every runtime diagnostic the bridge emits goes through shmlog's LOG_* macros,
# so a bridge running alongside irTactile lands in the same merged log as the
# engine. Fetched like irtactile-shm above, with the same escape hatch.
#
# The backend is a build-time choice and decides where the lines go:
#   shm    (default) - into the named partition, for a collector to drain;
#                      nothing appears on this process's console.
#   stdout           - one line per entry on stdout, for running the bridge by
#                      hand with no collector attached.
#   null             - compiled to a discard.
# SHMLOG_BACKEND is a cache variable in shmlog, so it is FORCEd here: a stale
# value from an earlier configure would otherwise win over this one silently.
# -----------------------------------------------------------------------------
set(HOST_BRIDGE_LOG_BACKEND shm CACHE STRING
    "shmlog backend for the bridge: shm (collector), stdout (console), or null")
set_property(CACHE HOST_BRIDGE_LOG_BACKEND PROPERTY STRINGS shm stdout null)
set(SHMLOG_BACKEND ${HOST_BRIDGE_LOG_BACKEND} CACHE STRING "" FORCE)

set(SHMLOG_LOCAL_DIR "" CACHE PATH "Path to a local shmlog checkout")
if(SHMLOG_LOCAL_DIR)
    add_subdirectory(${SHMLOG_LOCAL_DIR} shmlog)
else()
    include(FetchContent)
    # main, not a tag: shmlog carries no tagged release yet. Pin one as soon as
    # it does - the bridge and the collector draining its partition have to
    # agree on kFormatVersion.
    FetchContent_Declare(shmlog
        GIT_REPOSITORY https://github.com/jSchmidt-design/shmlog.git
        GIT_TAG        main)
    FetchContent_MakeAvailable(shmlog)
endif()

add_executable(irTactileSerialBridge
    src/main.cpp
    src/BridgeConfig.cpp
    src/BridgeConfig.h
    src/SerialProtocol.cpp
    src/SerialProtocol.h
    src/SerialPort.cpp
    src/SerialPort.h
    src/Throttle.h
    src/WireParams.cpp
    src/WireParams.h
)

target_include_directories(irTactileSerialBridge PRIVATE src)
target_link_libraries(irTactileSerialBridge PRIVATE irtactile::stream-shm shmlog::logger)
target_compile_features(irTactileSerialBridge PRIVATE cxx_std_20)

# -----------------------------------------------------------------------------
# belt-tune
#
# Interactive tool that sets the runtime-tunable encoder force curve (full-scale
# pull count + gamma) over the serial link. It links only the serial and framing
# code - no irtactile-shm, no shmlog. The two shared sources are compiled
# directly rather than through a static library, matching tests/CMakeLists.txt.
#
# --target belt-tune still triggers the FetchContent calls above, which run
# unconditionally at configure time. The reduced footprint is at the source and
# link level, not the configure level.
# -----------------------------------------------------------------------------
add_executable(belt-tune
    src/BeltTuneMain.cpp
    src/TuningFrame.cpp
    src/TuningFrame.h
    src/SerialPort.cpp
    src/SerialPort.h
)
target_include_directories(belt-tune PRIVATE src)
target_compile_features(belt-tune PRIVATE cxx_std_20)

# -----------------------------------------------------------------------------
# Tests
#
# Everything the bridge decides without touching the stream or the port:
# SerialProtocol (the wire-format encoder both sides of the link depend on
# byte-for-byte, see controller-firmware/protocol.md), the rate-ladder and wire arithmetic in
# WireParams, the command line in BridgeConfig, and Throttle. All pure, so they
# are tested natively rather than on a board or against a live shm section.
# -----------------------------------------------------------------------------
if(HOST_BRIDGE_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG        v2.4.11)
    set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
    set(DOCTEST_NO_INSTALL ON  CACHE BOOL "" FORCE)
    # doctest v2.4.11 declares cmake_minimum_required(VERSION 3.0); allow it to
    # configure under newer CMake that removed compatibility with < 3.5.
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    FetchContent_MakeAvailable(doctest)

    enable_testing()
    add_subdirectory(tests)
endif()

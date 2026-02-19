cmake_minimum_required(VERSION 3.16)
project(franka_direct CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

# ── Conda environment (provides gRPC, Protobuf) ───────────────────────────────
set(CONDA_ENV /root/miniconda3/envs/polymetis-local)
list(PREPEND CMAKE_PREFIX_PATH ${CONDA_ENV})

# ── gRPC & Protobuf ───────────────────────────────────────────────────────────
find_package(Protobuf REQUIRED)
find_package(gRPC     REQUIRED)

# Locate the grpc_cpp_plugin executable
if(TARGET gRPC::grpc_cpp_plugin)
    get_target_property(GRPC_CPP_PLUGIN gRPC::grpc_cpp_plugin LOCATION)
else()
    find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin
        HINTS ${CONDA_ENV}/bin /usr/local/bin /usr/bin)
endif()
message(STATUS "grpc_cpp_plugin: ${GRPC_CPP_PLUGIN}")

# ── libfranka ─────────────────────────────────────────────────────────────────
# Try the polymetis third-party build first, then fall back to system install.
set(POLYMETIS_ROOT /app/droid/fairo/polymetis/polymetis)

set(FRANKA_BASE ${POLYMETIS_ROOT}/src/clients/franka_panda_client/third_party/libfranka)

set(FRANKA_INCLUDE_HINTS
    ${FRANKA_BASE}/include
    ${CONDA_ENV}/include
    /usr/local/include
)

set(FRANKA_LIB_HINTS
    ${FRANKA_BASE}/build          # in-source build location
    ${CONDA_ENV}/lib
    /usr/local/lib
    /usr/lib
)

find_path(FRANKA_INCLUDE_DIR franka/robot.h HINTS ${FRANKA_INCLUDE_HINTS})
find_library(FRANKA_LIB franka HINTS ${FRANKA_LIB_HINTS} NO_DEFAULT_PATH)
if(NOT FRANKA_LIB)
    find_library(FRANKA_LIB franka)  # system fallback
endif()

if(NOT FRANKA_INCLUDE_DIR OR NOT FRANKA_LIB)
    message(FATAL_ERROR
        "Could not find libfranka.\n"
        "  Searched headers in: ${FRANKA_INCLUDE_HINTS}\n"
        "  Searched libs in:    ${FRANKA_LIB_HINTS}\n"
        "Set FRANKA_INCLUDE_DIR and FRANKA_LIB manually if needed.")
endif()
message(STATUS "libfranka headers: ${FRANKA_INCLUDE_DIR}")
message(STATUS "libfranka library: ${FRANKA_LIB}")

# ── Protobuf / gRPC code generation ──────────────────────────────────────────
set(PROTO_DIR  ${CMAKE_SOURCE_DIR}/proto)
set(PROTO_FILE ${PROTO_DIR}/franka_control.proto)

get_target_property(PROTOC protobuf::protoc LOCATION)

set(PROTO_SRCS ${CMAKE_BINARY_DIR}/franka_control.pb.cc)
set(PROTO_HDRS ${CMAKE_BINARY_DIR}/franka_control.pb.h)
set(GRPC_SRCS  ${CMAKE_BINARY_DIR}/franka_control.grpc.pb.cc)
set(GRPC_HDRS  ${CMAKE_BINARY_DIR}/franka_control.grpc.pb.h)

add_custom_command(
    OUTPUT  ${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS} ${GRPC_HDRS}
    COMMAND ${PROTOC}
        --proto_path=${PROTO_DIR}
        --cpp_out=${CMAKE_BINARY_DIR}
        --grpc_out=${CMAKE_BINARY_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating C++ protobuf/gRPC stubs"
)

# ── franka_server executable ──────────────────────────────────────────────────
add_executable(franka_server
    src/franka_server.cpp
    ${PROTO_SRCS}
    ${GRPC_SRCS}
)

target_include_directories(franka_server PRIVATE
    ${CMAKE_BINARY_DIR}                   # generated pb headers
    ${FRANKA_INCLUDE_DIR}
    ${FRANKA_BASE}/common/include         # research_interface/* headers
)

target_link_libraries(franka_server PRIVATE
    ${FRANKA_LIB}
    gRPC::grpc++
    protobuf::libprotobuf
    pthread
)

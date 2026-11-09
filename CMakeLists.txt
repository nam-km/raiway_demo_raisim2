cmake_minimum_required(VERSION 3.18)
# raisim 2's headers need C++20, which the system g++ 9 cannot compile
find_program(GXX10 g++-10)
if(GXX10 AND NOT CMAKE_CXX_COMPILER AND NOT CMAKE_TOOLCHAIN_FILE MATCHES "Emscripten")
    set(CMAKE_CXX_COMPILER ${GXX10})
endif()
project(raiway_demo_raisim2 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(RAISIM2LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../raisim2Lib" CACHE PATH "raisim2Lib checkout: its raisim SDK and examples/include")

# under emcmake the build is the browser page instead, against the RaiSim WebAssembly SDK
if(EMSCRIPTEN)
    set(RAISIM_WASM_SDK "${RAISIM2LIB_DIR}/wasm/sdk" CACHE PATH "The unpacked webassembly-sdk-<version> of a raisim2Lib release")
    set(RAIWAY_WEB_URL "" CACHE STRING "Address the page is published at, for the card a shared link shows")
    find_package(rayrai CONFIG REQUIRED PATHS "${RAISIM_WASM_SDK}" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    add_executable(raiway src/raiway_web.cpp)
    set_target_properties(raiway PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/site")
    rayrai_add_web_app(raiway TITLE "Raiway on RaiSim"
        DESCRIPTION "A wheel-legged robot drives its perceptive RL policy over stairs, blocks, holes and hurdles, simulated by RaiSim in your browser."
        URL "${RAIWAY_WEB_URL}"
        ASSETS "${CMAKE_CURRENT_SOURCE_DIR}/rsc/raiway@/assets/raiway")
    target_compile_definitions(raiway PRIVATE RAIWAY_RSC="/assets/raiway")
    target_compile_options(raiway PRIVATE -O3)
    # -O2 at link optimizes the 27 MB module in about 2/3 of -O3's time, for a 1 % larger file
    target_link_options(raiway PRIVATE -O2)
    # GitHub Pages serves index.html at the site's address
    add_custom_target(raiway_index ALL "${CMAKE_COMMAND}" -E copy_if_different raiway.html index.html
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/site" VERBATIM)
    add_dependencies(raiway_index raiway)
    return()
endif()

list(APPEND CMAKE_PREFIX_PATH "${RAISIM2LIB_DIR}/raisim")
find_package(raisim CONFIG REQUIRED)

add_executable(raiway_demo src/raiway_demo.cpp)
target_include_directories(raiway_demo PRIVATE "${RAISIM2LIB_DIR}/examples/include")
target_link_libraries(raiway_demo PRIVATE raisim::raisim pthread)
# held arrow keys are read from the X display; without X11 the sliders alone drive
find_package(X11)
if(X11_FOUND)
    target_compile_definitions(raiway_demo PRIVATE RAIWAY_X11)
    target_link_libraries(raiway_demo PRIVATE X11::X11)
endif()
target_compile_options(raiway_demo PRIVATE $<$<CONFIG:Release>:-O3>)

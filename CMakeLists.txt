cmake_minimum_required(VERSION 3.10)

project(evalAd5933 VERSION 1.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

option(EVAL_AD5933_BUILD_TESTS "Build unit tests" ON)

# Path to cyusb_linux source tree (expected sibling of this repo by default).
set(LIB_CYUSB_DIR "${CMAKE_SOURCE_DIR}/../cyusb_linux" CACHE PATH
    "Path to the cyusb_linux source tree")

# Where the firmware ships once installed.
set(AD5933_FW_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/share/ad5933/AD5933_34FW.hex"
    CACHE STRING "Installed firmware path baked into the library")

# --- Library ----------------------------------------------------------------

add_library(ad5933 SHARED lib/libad5933.cpp)
target_include_directories(ad5933 PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${LIB_CYUSB_DIR}/include)
target_compile_definitions(ad5933 PRIVATE
    AD5933_DEFAULT_FW_PATH="${AD5933_FW_INSTALL_PATH}")

find_library(LIBCYUSB cyusb HINTS ${LIB_CYUSB_DIR}/lib)
if(NOT LIBCYUSB)
    message(WARNING "libcyusb not found in ${LIB_CYUSB_DIR}/lib; link will fail.")
endif()
target_link_libraries(ad5933 PRIVATE ${LIBCYUSB})

# --- Executable -------------------------------------------------------------

add_executable(eval_ad5933_program src/main.cpp)
target_include_directories(eval_ad5933_program PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(eval_ad5933_program PRIVATE ad5933)

# --- Tests (pure math, no hardware) ----------------------------------------

if(EVAL_AD5933_BUILD_TESTS)
    enable_testing()
    add_executable(ad5933_math_tests tests/test_math.cpp)
    target_include_directories(ad5933_math_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
    add_test(NAME ad5933_math_tests COMMAND ad5933_math_tests)
endif()

# --- Install ---------------------------------------------------------------

install(TARGETS ad5933 eval_ad5933_program
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)
install(FILES include/libad5933.h
              include/ad5933types.h
              include/ad5933math.h
        DESTINATION include)
install(FILES cypressfw/AD5933_34FW.hex
        DESTINATION share/ad5933)
if(LIBCYUSB)
    install(FILES ${LIBCYUSB} DESTINATION lib)
endif()
if(EXISTS ${LIB_CYUSB_DIR}/include/cyusb.h)
    install(FILES ${LIB_CYUSB_DIR}/include/cyusb.h DESTINATION include)
endif()

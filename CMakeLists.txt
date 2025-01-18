cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MyProject VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set cyusb_linux repository dir
set(LIB_CYUSB_DIR ../cyusb_linux) 

# Add the source files for your library
add_library(ad5933 SHARED lib/libad5933.cpp)

# Specify the include directories for the library
target_include_directories(ad5933 PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(ad5933 PUBLIC ${LIB_CYUSB_DIR}/include)

# Find the other library
# Assuming the library is located at /path/to/other/library
find_library(LIBCYUSB cyusb ${LIB_CYUSB_DIR}/lib)

# Link the other library to your library
target_link_libraries(ad5933 PRIVATE ${LIBCYUSB})

# Add the executable for your main program
add_executable(eval_ad5933_program src/main.cpp)

# Link the MyLibrary library to your main program
target_link_libraries(eval_ad5933_program PRIVATE ad5933)

# Specify the include directories for the main program
target_include_directories(eval_ad5933_program PUBLIC 
                           ${PROJECT_SOURCE_DIR}/include)

# Add GoogleTest
#add_subdirectory(googletest)

# Enable testing
#enable_testing()
# Add the test executable
#add_executable(MyLibraryTest tests/test_mylibrary.cpp)

# Specify the include directories for the test executable
#target_include_directories(MyLibraryTest PUBLIC
#                           ${PROJECT_SOURCE_DIR}/include)

# Link gtest and gtest_main
#target_link_libraries(MyLibraryTest gtest gtest_main)

# Link your library to the test executable
#arget_link_libraries(MyLibraryTest MyLibrary)

# Add a test
#add_test(NAME MyLibraryTest COMMAND MyLibraryTest)

# Optionally, specify the installation rules
install(TARGETS ad5933 DESTINATION lib)
install(FILES include/libad5933.h DESTINATION include)
install(FILES include/types.h DESTINATION include)
install(FILES ${LIB_CYUSB_DIR}/lib/libcyusb.so DESTINATION lib)
install(FILES ${LIB_CYUSB_DIR}/include/cyusb.h DESTINATION include)


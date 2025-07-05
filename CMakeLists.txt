

cmake_minimum_required(VERSION 3.10)

project(RoutingKit)

enable_testing()

find_package(GTest REQUIRED)
find_package(ZLIB REQUIRED)

cmake_minimum_required(VERSION 3.10)

project(RoutingKit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler options (debug by default, can be changed by build type)
add_compile_options(-Wall -O0 -ggdb -march=native -ffast-math -D_GLIBCXX_DEBUG)


include_directories(${CMAKE_SOURCE_DIR}/include)

# Source files
file(GLOB SRC_FILES "src/*.cpp")
file(GLOB HEADER_FILES "src/*.h")


# Split source files into main and non-main files
set(MAIN_SRC_FILES)
set(NONMAIN_SRC_FILES)
foreach(SRC ${SRC_FILES})
    file(READ ${SRC} SRC_CONTENT)
    string(FIND "${SRC_CONTENT}" "int main" MAIN_POS)
    if(NOT ${MAIN_POS} EQUAL -1)
        list(APPEND MAIN_SRC_FILES ${SRC})
    else()
        list(APPEND NONMAIN_SRC_FILES ${SRC})
    endif()
endforeach()


# Create a static library from all non-main source files
add_library(routingkit_common STATIC ${NONMAIN_SRC_FILES} ${HEADER_FILES})
target_include_directories(routingkit_common PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(routingkit_common PUBLIC ZLIB::ZLIB)


# Create executables and link with the common library
set(TEST_EXECUTABLES)
foreach(SRC ${MAIN_SRC_FILES})
    get_filename_component(EXE_NAME ${SRC} NAME_WE)
    add_executable(${EXE_NAME} ${SRC})
    target_link_libraries(${EXE_NAME} PRIVATE routingkit_common m pthread ZLIB::ZLIB)
endforeach()

file(GLOB ALL_CPP_FILES CONFIGURE_DEPENDS "src/*.cpp")
set(GTEST_TESTS)
foreach(FILE ${ALL_CPP_FILES})
    file(READ "${FILE}" CONTENTS)
    string(FIND "${CONTENTS}" "#include <gtest" FOUND)
    if(NOT ${FOUND} EQUAL -1)
        list(APPEND GTEST_TESTS ${FILE})
    endif()
endforeach()
foreach(TEST_SRC ${GTEST_TESTS})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_link_libraries(${TEST_NAME} PRIVATE routingkit_common GTest::gtest GTest::gtest_main pthread m ZLIB::ZLIB)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()

# Libraries (static and shared)
if(EXISTS "${CMAKE_SOURCE_DIR}/include")
    file(GLOB LIB_DIRS RELATIVE "${CMAKE_SOURCE_DIR}/include" "${CMAKE_SOURCE_DIR}/include/*")
    foreach(LIB ${LIB_DIRS})
        file(GLOB LIB_HEADERS "${CMAKE_SOURCE_DIR}/include/${LIB}/*.h")
        file(GLOB LIB_SOURCES "${CMAKE_SOURCE_DIR}/src/${LIB}.cpp")
        if(LIB_SOURCES)
            add_library(${LIB}_static STATIC ${LIB_SOURCES} ${LIB_HEADERS})
            add_library(${LIB}_shared SHARED ${LIB_SOURCES} ${LIB_HEADERS})
            set_target_properties(${LIB}_shared PROPERTIES OUTPUT_NAME ${LIB})
            set_target_properties(${LIB}_static PROPERTIES OUTPUT_NAME ${LIB})
            target_include_directories(${LIB}_static PUBLIC ${CMAKE_SOURCE_DIR}/include)
            target_include_directories(${LIB}_shared PUBLIC ${CMAKE_SOURCE_DIR}/include)
        endif()
    endforeach()
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

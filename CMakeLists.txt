
project (SimEdit)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

##################################
# Build options
##################################

option(DEBUG "Enable debug compilation" OFF)

message(STATUS "Options - "
    " -DDEBUG='${DEBUG}' "
)

if (DEBUG)
    SET(EXECUTABLE simeditd)
    SET(CORELIB cored)
else()
    SET(EXECUTABLE simedit)
    SET(CORELIB core)
endif()

##################################
# External libraries
##################################

set(LIBS "glog")
set(LIBPATHS)

find_package(TBB REQUIRED)
list(APPEND LIBS ${TBB_LIBRARIES})
list(APPEND LIBPATHS ${TBB_LIBRARY_DIRS})

find_package(Ceres REQUIRED)
list(APPEND LIBS ${CERES_LIBRARIES})

find_package(Eigen REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME ON)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED system iostreams filesystem timer chrono)
list(APPEND LIBS ${Boost_LIBRARIES})

# for Cholmod
find_package(LAPACK REQUIRED)
list(APPEND LIBS ${LAPACK_LIBRARIES})

find_package(SuiteSparse REQUIRED)
list(APPEND LIBS ${SuiteSparse_LIBRARIES})

##################################
# Build dependencies
##################################

add_subdirectory("dependencies")

set(CORE_INCPATHS "src")
list(APPEND CORE_INCPATHS 
    ${TBB_INCLUDE_DIR} 
    ${EIGEN_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS} 
    ${CERES_INCLUDE_DIRS}
    ${SuiteSparse_INCLUDE_DIRS}
)


##################################
# Build core library
##################################

set(CORE_SOURCES
    src/gravity.cpp
    src/particle.cpp
    src/test.cpp
    src/util.cpp
    src/vectors.cpp
)

# Build flags
set(LINK_FLAGS "")
set(AS_FLAGS "-Wall -std=c++14 -Wfatal-errors" )
if (DEBUG)
    set(AS_FLAGS "${AS_FLAGS} -g -DDEBUG" )
else()
    set(AS_FLAGS "${AS_FLAGS} -O3" )
endif()

add_library(${CORELIB} OBJECT ${CORE_SOURCES})
set_target_properties(${CORELIB} PROPERTIES COMPILE_FLAGS ${AS_FLAGS})
target_include_directories(${CORELIB} PUBLIC ${CORE_INCPATHS})
list(APPEND COMPONENTS $<TARGET_OBJECTS:${CORELIB}>)

##################################
# Add git revision
##################################

if (EXISTS "${CMAKE_SOURCE_DIR}/.git/index")
    include(GetGitRevisionDescription)
    get_git_head_revision(BRANCH HASH)
    git_date(DATE_LONG)
    string(LENGTH ${DATE_LONG} L)
    if(L LESS 10)
        set(DATA "unknown")
    else()
        string(SUBSTRING ${DATE_LONG} 0 10 DATE)
    endif()
else()
    set(HASH "unknown")
    set(BRANCH "unknown")
    set(DATE "unknown")
endif()
if (DEBUG)
    set(BUILDS "debug")
else()
    set(BUILDS "release")
endif()
string(SUBSTRING ${HASH} 0 7 HASH_SHORT)

# build version file
file(STRINGS src/version.txt VERSION_FILE)
list(GET VERSION_FILE 0 VERSION_NUMBER)
list(GET VERSION_FILE 1 VERSION_NAME)
set(VERSIONCPP ${CMAKE_CURRENT_BINARY_DIR}/version.cpp)
list(APPEND COMPONENTS ${VERSIONCPP})
configure_file(src/gitid_template.cpp ${VERSIONCPP})

##################################
# Build executable
##################################

add_executable(${EXECUTABLE} src/main.cpp ${COMPONENTS})

set_target_properties(${EXECUTABLE} PROPERTIES COMPILE_FLAGS ${AS_FLAGS})
#set_target_properties(${EXECUTABLE} PROPERTIES LINK_FLAGS ${ALINK_FLAGS})
target_include_directories(${EXECUTABLE} PUBLIC "src")
target_link_libraries(${EXECUTABLE} ${LIBS})


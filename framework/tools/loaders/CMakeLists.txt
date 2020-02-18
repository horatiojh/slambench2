cmake_minimum_required(VERSION 2.8)



######################################################################################
#################        COMMONS
######################################################################################

include_directories(. ${PROJECT_SOURCE_DIR}/shared/include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")


if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 8.0)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=catch-value=")
endif()
	
######################################################################################
################# DYNAMIC LIBRARY SYSTEM
######################################################################################

add_executable(benchmark_loader benchmark_loader.cpp)
TARGET_INCLUDE_DIRECTORIES(benchmark_loader PUBLIC ${SLAMBENCH_INCLUDE_DIR})
target_link_libraries(benchmark_loader PRIVATE pthread ${SLAMBENCH_LIBRARIES})

find_package(Pangolin QUIET)
find_package(TOON QUIET)




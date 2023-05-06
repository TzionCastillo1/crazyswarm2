cmake_minimum_required(VERSION 3.5)

project(crazyflie_gazebo)

find_package(ament_cmake REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

add_library(<FILE_NAME>
	SHARED
	src/<FILE_NAME>.cc
)

target_include_directories(
	<FILE_NAME> PRIVATE include
)

target_link_libraries(<FILE_NAME> PRIVATE
	gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
	TARGETS <FILE_NAME>
	DESTINATION share/${PROJECT_NAME}
)

install(
	DIRECTORY worlds/
	DESTINATION share/${PROJECT_NAME}/worlds
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)

project(crazyflie_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

install(
	DIRECTORY
		meshes/
	DESTINATION share/${PROJECT_NAME}/meshes
)

install(
	DIRECTORY
		crazyflie/
	DESTINATION share/${PROJECT_NAME}/crazyflie
)

install(
	DIRECTORY
		worlds/
	DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()

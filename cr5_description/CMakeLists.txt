cmake_minimum_required(VERSION 3.14)
project(cr5_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes xacro config launch
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()
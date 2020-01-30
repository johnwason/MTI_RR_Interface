    
cmake_minimum_required(VERSION 3.3)
project(mti2D_RR)

set (CMAKE_CXX_STANDARD 11)




find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteur_standard_robdef_cpp REQUIRED)
include_directories(${RobotRaconteur_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
link_directories(EthernetScanner/EthernetScanner_x64/)


ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS 
	com.robotraconteur.geometry.robdef
	com.robotraconteur.geometryf.robdef
	com.robotraconteur.identifier.robdef
	com.robotraconteur.resource.robdef
	com.robotraconteur.image.robdef
	com.robotraconteur.uuid.robdef
	com.robotraconteur.datetime.robdef
	com.robotraconteur.device.robdef
	com.robotraconteur.param.robdef
	com.robotraconteur.sensordata.robdef
	com.robotraconteur.units.robdef
	com.robotraconteur.datatype.robdef
	com.robotraconteur.sensor.robdef
	com.robotraconteur.pointcloud.robdef
	com.robotraconteur.pointcloud.sensor.robdef
	MASTER_HEADER
	IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../robotraconteur_standard_robdef_cpp/robdef/group1
	)
if (MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

add_executable(mti2D_RR
	${RR_THUNK_HDRS}
	${RR_THUNK_SRCS}
	mti2D_RR_bridge.cpp
	mti2D_RR_bridge.h
	mti2D_RR_interface.h
	EthernetScanner/EthernetScannerSDK.h
	EthernetScanner/EthernetScannerSDKDefine.h
	)



target_link_libraries(mti2D_RR RobotRaconteurCore ${Boost_LIBRARIES} ${RobotRaconteurCore_Extra_libraries} )
target_link_libraries(mti2D_RR EthernetScanner.lib)


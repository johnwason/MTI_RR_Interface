    
cmake_minimum_required(VERSION 3.3)
project(mti2D_RR)

set (CMAKE_CXX_STANDARD 11)

find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteurCompanion REQUIRED)
link_directories(EthernetScanner/EthernetScanner_x64/)

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



target_link_libraries(mti2D_RR RobotRaconteurCompanion RobotRaconteurCore EthernetScanner.lib)

include(GNUInstallDirs)

install(TARGETS mti2D_RR RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${CMAKE_SOURCE_DIR}/EthernetScanner/EthernetScanner_x64/EthernetScanner.dll DESTINATION ${CMAKE_INSTALL_BINDIR})

set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_DIRECTORY ${PROJECT_BINARY_DIR}/package)

include(CPack)

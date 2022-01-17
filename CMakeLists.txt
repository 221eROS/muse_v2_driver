cmake_minimum_required(VERSION 3.0.2)
project(muse_v2_driver)

find_package(catkin REQUIRED
  diagnostic_msgs
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  message_generation
)

add_message_files(FILES
		  CommandList.msg
		  Transmission.msg
		  RosLog.msg
	         )

add_service_files(FILES
		  RawDataStreaming.srv
		  Battery.srv
		  StopTransmission.srv
		  Shutdown.srv
		  Disconnect.srv
		  GetConfigurationParams.srv
		  SetConfigurationParams.srv
		  GetCalibrationParams.srv
		  MemoryManagement.srv
		  )

generate_messages(DEPENDENCIES
					std_msgs
					geometry_msgs
					diagnostic_msgs
                  )

find_package(lib221e CONFIG REQUIRED)
include_directories(${lib221e_INCLUDE_DIRS})

find_library(MUSE_V2_LIB NAMES MuseV2 PATHS ${lib221e_INCLUDE_DIRS})
message(STATUS ${MUSE_V2_LIB})

find_library(SERIAL_LIB NAMES Connection PATHS ${lib221e_INCLUDE_DIRS})
message(STATUS ${SERIAL_LIB})

find_package(Boost REQUIRED COMPONENTS date_time)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS
              roscpp 
			  message_runtime
			  std_msgs
			  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${lib221e_INCLUDE_DIRS}
)

set(SOURCES src/internal/MuseV2.cpp
	    src/internal/Stream.cpp
	    src/internal/Miscellaneous.cpp
        src/internal/Configuration.cpp
	    src/internal/Calibration.cpp
	    src/internal/Memory.cpp
    )

add_executable(muse_v2 "src/muse_v2.cpp" ${SOURCES})
add_dependencies(muse_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2 ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_stop_transmission "src/stop_transmission.cpp" ${SOURCES})
add_dependencies(muse_v2_stop_transmission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_stop_transmission ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_shutdown "src/shutdown.cpp" ${SOURCES})
add_dependencies(muse_v2_shutdown ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_shutdown ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_battery "src/battery.cpp" ${SOURCES})
add_dependencies(muse_v2_battery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_battery ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_get_configuration_params "src/get_configuration_params.cpp" ${SOURCES})
add_dependencies(muse_v2_get_configuration_params ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_get_configuration_params ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_set_configuration_params "src/set_configuration_params.cpp" ${SOURCES})
add_dependencies(muse_v2_set_configuration_params ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_set_configuration_params ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_get_calibration_params "src/get_calibration_params.cpp" ${SOURCES})
add_dependencies(muse_v2_get_calibration_params ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_get_calibration_params ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_logger "src/logger.cpp" ${SOURCES})
add_dependencies(muse_v2_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_logger ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_start_transmission "src/start_transmission.cpp" ${SOURCES})
add_dependencies(muse_v2_start_transmission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_start_transmission ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 

add_executable(muse_v2_raw_data_subscriber "src/raw_data_subscriber.cpp" ${SOURCES})
add_dependencies(muse_v2_raw_data_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(muse_v2_raw_data_subscriber ${catkin_LIBRARIES} ${MUSE_V2_LIB} ${SERIAL_LIB} ${Boost_LIBRARIES}) 
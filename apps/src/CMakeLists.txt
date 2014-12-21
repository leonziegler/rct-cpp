cmake_minimum_required(VERSION 2.6)
            
# --- generate executable
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/core/src ${CMAKE_SOURCE_DIR}/rsb/src ${CMAKE_SOURCE_DIR}/ros/src ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/../../core/src)
ADD_EXECUTABLE(rct-static-publisher rct/RctStaticPublisher.cpp)
TARGET_LINK_LIBRARIES(rct-static-publisher ${RCT_NAME} ${RCT_NAME_RSB} ${RCT_NAME_ROS} ${RSB_LIBRARIES} ${RST_LIBRARIES} ${Log4cxx_LIBRARIES})
SET_TARGET_PROPERTIES(rct-static-publisher PROPERTIES
                                 VERSION ${RCT_VERSION}
                                 SOVERSION ${RCT_API_VERSION})

# --- install target
INSTALL(TARGETS rct-static-publisher
        RUNTIME DESTINATION ${BINDIR})

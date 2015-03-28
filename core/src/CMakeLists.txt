cmake_minimum_required(VERSION 2.6)

# --- generate executable
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/rsb/src ${CMAKE_BINARY_DIR}/rsb/src ${CMAKE_SOURCE_DIR}/ros/src ${CMAKE_CURRENT_SOURCE_DIR})
ADD_LIBRARY(${RCT_NAME} SHARED rct/TransformerFactory.cpp rct/impl/TransformerTF2.cpp rct/TransformReceiver.cpp rct/TransformPublisher.cpp)
TARGET_LINK_LIBRARIES(${RCT_NAME} ${RSB_LIBRARIES} ${RSBXML_LIBRARIES} ${tf2-minimal_LIBRARIES})
SET_TARGET_PROPERTIES(${RCT_NAME} PROPERTIES
                                 VERSION ${RCT_VERSION}
                                 SOVERSION ${RCT_API_VERSION})

# --- install target
INSTALL(TARGETS ${RCT_NAME}
        LIBRARY DESTINATION ${LIBDIR})
INSTALL(DIRECTORY . DESTINATION ${INCLUDEDIR}
          FILES_MATCHING 
          PATTERN "./rct/*.h" 
          PATTERN "./rct/impl/*.h" 
)

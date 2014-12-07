cmake_minimum_required(VERSION 2.6)
            
# --- generate executable
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
ADD_LIBRARY(${RCT_NAME} SHARED rct/TransformerTF2.cpp)
TARGET_LINK_LIBRARIES(${RCT_NAME} ${RSB_LIBRARIES} ${RSBXML_LIBRARIES})
SET_TARGET_PROPERTIES(${RCT_NAME} PROPERTIES
                                 VERSION ${RSBAM_VERSION}
                                 SOVERSION ${RSBAM_API_VERSION})

# --- install target
INSTALL(TARGETS ${RCT_NAME}
        LIBRARY DESTINATION ${LIBDIR})
INSTALL(DIRECTORY . DESTINATION ${INCLUDEDIR}
          FILES_MATCHING 
          PATTERN "./rsbam/*.h" 
          PATTERN "./rsbam/comm/rsb/*.h" 
          PATTERN "./rsbam/comm/proto/*.h" 
          PATTERN "./rsbam/model/*.h" )

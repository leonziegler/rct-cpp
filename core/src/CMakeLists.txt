cmake_minimum_required(VERSION 2.6)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/rct/rctConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/rct/rctConfig.h)
            
# --- generate executable
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/rsb/src ${CMAKE_SOURCE_DIR}/ros/src ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
ADD_LIBRARY(${RCT_NAME} SHARED rct/TransformerFactory.cpp rct/impl/TransformerTF2.cpp rct/Transformer.cpp)
TARGET_LINK_LIBRARIES(${RCT_NAME} ${RSB_LIBRARIES} ${RSBXML_LIBRARIES} ${Log4cxx_LIBRARIES} ${tf2_LIBRARIES})
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

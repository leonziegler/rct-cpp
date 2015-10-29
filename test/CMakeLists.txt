ENABLE_TESTING()

INCLUDE_DIRECTORIES(BEFORE ../core/src
                           ${CMAKE_CURRENT_BINARY_DIR}
                           ${CMAKE_CURRENT_BINARY_DIR}/../core/src
                           ${GMOCK_INCLUDE_DIRS}
                           ${CMAKE_CURRENT_BINARY_DIR}
                           ${CMAKE_CURRENT_SOURCE_DIR})

ADD_DEFINITIONS(${GMOCK_CFLAGS})

# --- setup generic test infrastructure ---

SET(TEST_RESULT_DIR ${CMAKE_BINARY_DIR}/testresults)
SET(TEST_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(AVAILABLE_TESTS)

# --- core test ---
                      
SET(CORE_TEST_SOURCES rcttest_core.cpp
                      rct/FrameTreeTest.cpp
                      )

ADD_EXECUTABLE(rcttest_core ${CORE_TEST_SOURCES})
TARGET_LINK_LIBRARIES(rcttest_core
                      ${RCT_NAME}
                      ${RCT_NAME_RSB}
                      ${Boost_LIBRARIES}
                      ${GMOCK_LIBRARIES})

ADD_TEST(rcttest_core ${EXECUTABLE_OUTPUT_PATH}/${CORE_TEST_NAME} "--gtest_output=xml:${TEST_RESULT_DIR}/")
LIST(APPEND AVAILABLE_TESTS rcttest_core)


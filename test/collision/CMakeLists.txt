include_directories(${CMAKE_SOURCE_DIR} ${BOOST_INCLUDE_DIRS})
link_directories(${Boost_INCLUDE_DIRS})

add_executable(wall-ball Wall-Ball.cpp)
target_link_libraries(wall-ball
                      collision
                      ${BOOST_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(wall-ball wall-ball)

add_executable(segment Segment.cpp)
target_link_libraries(segment
                      collision
                      ${BOOST_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(segment segment)

add_executable(ball-ball Ball-Ball.cpp)
target_link_libraries(ball-ball
                      collision
                      ${BOOST_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(ball-ball ball-ball)

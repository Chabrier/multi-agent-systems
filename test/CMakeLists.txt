INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
##
## Unity tests
##
ADD_EXECUTABLE(Dummy Dummy.cpp)
TARGET_LINK_LIBRARIES(Dummy multi-agent-systems ${Boost_LIBRARIES})
add_test(Test_Dummy Dummy)



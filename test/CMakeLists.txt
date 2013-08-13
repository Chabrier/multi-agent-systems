LINK_DIRECTORIES(${VLE_LIBRARY_DIRS} ${CMAKE_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${VLE_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)
##
## Unity tests
##
ADD_EXECUTABLE(Agent Agent_test.cpp)
ADD_EXECUTABLE(Scheduler Scheduler_test.cpp)

TARGET_LINK_LIBRARIES(Agent
    ${Boost_LIBRARIES} ${VLE_LIBRARIES})
TARGET_LINK_LIBRARIES(Scheduler
    ${Boost_LIBRARIES} ${VLE_LIBRARIES})
add_test(Agent_test Agent)
add_test(Scheduler_test Scheduler)

add_subdirectory(dynamics)
add_subdirectory(collision)

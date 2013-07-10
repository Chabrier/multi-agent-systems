include_directories(${CMAKE_SOURCE_DIR}/src ${VLE_INCLUDE_DIRS})
link_directories(${VLE_LIBRARY_DIRS} ${CMAKE_SOURCE_DIR}/lib)

ADD_LIBRARY(Ball MODULE Ball.cpp)
TARGET_LINK_LIBRARIES(Ball ${VLE_LIBRARIES} mas)
INSTALL(TARGETS Ball
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(SimpleWall MODULE SimpleWall.cpp)
TARGET_LINK_LIBRARIES(SimpleWall ${VLE_LIBRARIES} mas)
INSTALL(TARGETS SimpleWall
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(ExecuterWallBall MODULE ExecuterWallBall.cpp)
TARGET_LINK_LIBRARIES(ExecuterWallBall ${VLE_LIBRARIES} mas)
INSTALL(TARGETS ExecuterWallBall
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

include_directories(${CMAKE_SOURCE_DIR}/src ${VLE_INCLUDE_DIRS})
link_directories(${VLE_LIBRARY_DIRS} ${CMAKE_SOURCE_DIR}/lib)

#~ ADD_LIBRARY(Ball MODULE Ball.cpp)
#~ TARGET_LINK_LIBRARIES(Ball ${VLE_LIBRARIES} mas)
#~ INSTALL(TARGETS Ball
    #~ RUNTIME DESTINATION plugins/simulator
    #~ LIBRARY DESTINATION plugins/simulator)
#~
#~ ADD_LIBRARY(SimpleWall MODULE SimpleWall.cpp)
#~ TARGET_LINK_LIBRARIES(SimpleWall ${VLE_LIBRARIES} mas)
#~ INSTALL(TARGETS SimpleWall
    #~ RUNTIME DESTINATION plugins/simulator
    #~ LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(WallG MODULE WallG.cpp)
TARGET_LINK_LIBRARIES(WallG ${VLE_LIBRARIES} mas collision)
INSTALL(TARGETS WallG
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(ExecuterWallBall MODULE ExecuterWallBall.cpp)
TARGET_LINK_LIBRARIES(ExecuterWallBall ${VLE_LIBRARIES} mas collision)
INSTALL(TARGETS ExecuterWallBall
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(BallG MODULE BallG.cpp)
TARGET_LINK_LIBRARIES(BallG ${VLE_LIBRARIES} mas collision)
INSTALL(TARGETS BallG
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(ExecuterWallBall2 MODULE ExecuterWallBall2.cpp)
TARGET_LINK_LIBRARIES(ExecuterWallBall2 ${VLE_LIBRARIES} mas collision)
INSTALL(TARGETS ExecuterWallBall2
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(Sky MODULE Sky.cpp)
TARGET_LINK_LIBRARIES(Sky ${VLE_LIBRARIES} mas collision)
INSTALL(TARGETS Sky
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(Bird MODULE Bird.cpp)
TARGET_LINK_LIBRARIES(Bird ${VLE_LIBRARIES} mas collision)
INSTALL(TARGETS Bird
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)

ADD_LIBRARY(God MODULE God.cpp)
TARGET_LINK_LIBRARIES(God ${VLE_LIBRARIES} mas collision)
INSTALL(TARGETS God
    RUNTIME DESTINATION plugins/simulator
    LIBRARY DESTINATION plugins/simulator)
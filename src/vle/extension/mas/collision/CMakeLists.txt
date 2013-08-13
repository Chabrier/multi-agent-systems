set(HEADERS Circle.hpp Segment.hpp Types.hpp Vector.hpp)
set(SRC Segment.cpp Vector2d.cpp Circle.cpp)

include_directories(${CMAKE_SOURCE_DIR} ${BOOST_INCLUDE_DIRS})
link_directories(${Boost_INCLUDE_DIRS})

add_library(collision SHARED ${SRC})

target_link_libraries(collision ${BOOST_LIBRARIES})
install(FILES ${HEADERS} DESTINATION include)

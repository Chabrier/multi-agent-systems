#ifndef TYPES_HPP
#define TYPES_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include "boost/tuple/tuple.hpp"
#include <vle/extension/mas/collision/Vector2d.hpp>

typedef boost::geometry::model::d2::point_xy<double> Point;

namespace bg = boost::geometry;

typedef struct {
    Point object1CollisionPosition;
    Point object2CollisionPosition;
    Point collisionPoint;
} CollisionPoints;

#endif

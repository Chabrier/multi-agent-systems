#include <vle/extension/mas/collision/Types.hpp>
#include <vle/extension/mas/collision/Circle.hpp>
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Wall-Ball
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <iostream>
#define DELTA 0.001

BOOST_AUTO_TEST_CASE( inCollisionTests )
{

    for(double s = 0.0; s <= 10.0; s+=DELTA) {
        Segment segment(Point(0.0,0.0),Point(0.0,10.0));
        Circle circle(Point(s,2.0+s),1.0);
        Vector2d direction(0.0,1+DELTA);
        BOOST_ASSERT(!circle.inCollision(segment,direction));
    }

    for(double s = 0.0; s <= 10.0; s+=DELTA) {
        Segment segment(Point(0.0,0.0),Point(0.0,10.0));
        Circle circle(Point(s,2.0+s),1.0);
        Circle circle2(Point(s,-2.0-s),1.0);
        Vector2d direction(0.0,-1+DELTA);
        Vector2d direction2(0.0,1+DELTA);
        BOOST_ASSERT(!circle.inCollision(segment,direction));
        BOOST_ASSERT(!circle2.inCollision(segment,direction2));
    }

    for(double s = -5.0; s <= 15.0; s+=DELTA) {
        Segment segment(Point(0.0,0.0),Point(0.0,10.0));
        Circle circle(Point(1,s),1.0);
        Circle circle2(Point(-1,s),1.0);
        Vector2d direction(-1-DELTA,0.0);
        Vector2d direction2(1+DELTA,0.0);
        if(s >= -1.0 && s <= 11.0) {
            BOOST_ASSERT(circle.inCollision(segment,direction));
            BOOST_ASSERT(circle2.inCollision(segment,direction2));
        } else {
            BOOST_ASSERT(!circle.inCollision(segment,direction));
            BOOST_ASSERT(!circle2.inCollision(segment,direction2));
        }
    }

    for(double s = -10.0; s <= 15; s+=DELTA) {
        Segment segment(Point(0.0,0.0),Point(0.0,10.0));
        Circle circle(Point(1.0,s),1.0);
        Circle circle2(Point(-1.0,s),1.0);
        Vector2d direction(-1.0,1.0);
        Vector2d direction2(1.0,1.0);

        if(s >= -2.0 && s <= 10.0) {
            BOOST_ASSERT(circle.inCollision(segment,direction));
            BOOST_ASSERT(circle2.inCollision(segment,direction2));
        } else {
            BOOST_ASSERT(!circle.inCollision(segment,direction));
            BOOST_ASSERT(!circle2.inCollision(segment,direction2));
        }
    }
}

BOOST_AUTO_TEST_CASE( collisionPointsTest )
{
    for(double s = -1.0; s <= 11.0; s+=DELTA) {
        Segment segment(Point(0.0,0.0),Point(0.0,10.0));
        Circle circle(Point(1,s),1.0);
        Vector2d direction(-1.0,1.0);
        Vector2d n_direction = direction;
        n_direction.normalize();

        Point intersection, collision;
        CollisionPoints cp;
        cp = circle.collisionPoints(segment,direction);
        intersection = cp.collisionPoint;
        collision = cp.object1CollisionPosition;

        BOOST_CHECK_CLOSE(intersection.x(), 0.0, 0.0001 );
        BOOST_CHECK_CLOSE(intersection.y(), s + 1.0, 0.0001 );
        BOOST_CHECK_CLOSE(bg::distance(collision,intersection),1.0,0.0001);
        BOOST_CHECK_CLOSE(collision.x(), intersection.x() + -1*n_direction.x(), 0.0001 );
        BOOST_CHECK_CLOSE(collision.y(), intersection.y() + -1*n_direction.y(), 0.0001 );
    }
}

BOOST_AUTO_TEST_CASE( newDirection )
{
    for(double s = -1.0; s <= 11.0; s+=DELTA) {
        Segment segment(Point(0.0,0.0),Point(0.0,10.0));
        Circle circle(Point(1,s),1.0);
        Vector2d new_direction, direction(-1-s,1.0+s);

        new_direction = circle.newDirection(segment,direction);

        BOOST_CHECK_CLOSE(new_direction.x(), 1.0+s, 0.0001 );
        BOOST_CHECK_CLOSE(new_direction.y(), 1.0+s, 0.0001 );
    }

    {
        Segment segment(Point(0.0,0.0),Point(10.0,10.0));
        Circle circle(Point(0.0,1.0),0.1);
        Vector2d new_direction, direction(-1.0,-1.0);

        new_direction = circle.newDirection(segment,direction);

        BOOST_CHECK_CLOSE(new_direction.x(), -1.0, 0.0001 );
        BOOST_CHECK_CLOSE(new_direction.y(), -1.0, 0.0001 );
    }

    {
        Segment segment(Point(0.0,10.0),Point(10.0,0.0));
        Circle circle(Point(0.0,0.0),0.1);
        Vector2d new_direction, direction(20.0,20.0);

        new_direction = circle.newDirection(segment,direction);

        BOOST_CHECK_CLOSE(new_direction.x(), -20.0, 0.0001 );
        BOOST_CHECK_CLOSE(new_direction.y(), -20.0, 0.0001 );
    }
}

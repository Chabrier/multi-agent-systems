#include <vle/extension/mas/collision/Types.hpp>
#include <vle/extension/mas/collision/Circle.hpp>
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Ball-Ball
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <iostream>
#define DELTA 0.001

BOOST_AUTO_TEST_CASE( inCollisionTests )
{
    /* Collision already happend */
    {
    Circle c1(Point(1.0,1.0),1.0);
    Circle c2(Point(2.0,1.0),1.0);
    Vector2d d1(1.0,0.0);
    Vector2d d2(-1.0,0.0);

    BOOST_ASSERT(!c1.inCollision(d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(d1,c2,-d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,-d2));
    }

    /* Collision will happend, first test must be true */
    {
    Circle c1(Point(1.0,1.0),0.1);
    Circle c2(Point(2.0,1.0),0.1);
    Vector2d d1(1.0,0.0);
    Vector2d d2(-1.0,0.0);

    BOOST_ASSERT(c1.inCollision(d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(d1,c2,-d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,-d2));
    }

    /* Collision will happend, first test must be true */
    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,1.0),0.5);
    Vector2d d1(1.0,0.0);
    Vector2d d2(-1.0,0.0);

    BOOST_ASSERT(c1.inCollision(d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(d1,c2,-d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,-d2));
    }

    /* Collision will not happend, all test false */
    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,1.0+DELTA),0.5);
    Vector2d d1(1.0,0.0);
    Vector2d d2(-1.0,0.0);

    BOOST_ASSERT(!c1.inCollision(d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(d1,c2,-d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,-d2));
    }

    /* Collision will happend, first test true */
    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,0.0),0.5);
    Vector2d d1(1.0,1.0);
    Vector2d d2(-1.0,1.0);

    BOOST_ASSERT(c1.inCollision(d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(d1,c2,-d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,-d2));
    }

    /* Collision will not happend, speed is too high */
    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,0.0),0.5);
    Vector2d d1(1.0,1.0);
    Vector2d d2(-2.1,2.1);

    BOOST_ASSERT(!c1.inCollision(d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(d1,c2,-d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,-d2));
    }

    /* Collision will happend, speed ok */
    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,0.0),0.5);
    Vector2d d1(1.0,1.0);
    Vector2d d2(-2.0,2.0);

    BOOST_ASSERT(c1.inCollision(d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,d2));
    BOOST_ASSERT(!c1.inCollision(d1,c2,-d2));
    BOOST_ASSERT(!c1.inCollision(-d1,c2,-d2));
    }
}

BOOST_AUTO_TEST_CASE( collisionPoints )
{
    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,0.0),0.5);
    Vector2d d1(1.0,1.0);
    Vector2d d2(-1.0,1.0);

    Point inter, collision;
    CollisionPoints cp = c1.collisionPoints(d1,c2,d2);
    inter = cp.collisionPoint;
    collision = cp.object1CollisionPosition;

    BOOST_CHECK_CLOSE(collision.x(), 1.0, 0.0001 );
    BOOST_CHECK_CLOSE(collision.y(), 1.0, 0.0001 );
    BOOST_CHECK_CLOSE(bg::distance(collision,inter),0.5,0.0001);
    BOOST_CHECK_CLOSE(inter.x(), 1.5, 0.0001 );
    BOOST_CHECK_CLOSE(inter.y(), 1.0, 0.0001 );
    }

    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,0.0),0.5);
    Vector2d d1(1.0,0.0);
    Vector2d d2(-1.0,0.0);

    Point inter, collision;
    CollisionPoints cp = c1.collisionPoints(d1,c2,d2);
    inter = cp.collisionPoint;
    collision = cp.object1CollisionPosition;

    BOOST_CHECK_CLOSE(collision.x(), 1.0, 0.0001 );
    BOOST_CHECK_CLOSE(collision.y(), 0.0, 0.0001 );
    BOOST_CHECK_CLOSE(bg::distance(collision,inter),0.5,0.0001);
    BOOST_CHECK_CLOSE(inter.x(), 1.5, 0.0001 );
    BOOST_CHECK_CLOSE(inter.y(), 0.0, 0.0001 );
    }

    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,0.0),0.5);
    Vector2d d1(1.0,0.0);
    Vector2d d2(-2.0,0.0);

    Point inter, collision;
    CollisionPoints cp = c1.collisionPoints(d1,c2,d2);
    inter = cp.collisionPoint;
    collision = cp.object1CollisionPosition;

    BOOST_CHECK_CLOSE(collision.x(), 2.0/3.0, 0.0001 );
    BOOST_CHECK_CLOSE(collision.y(), 0.0, 0.0001 );
    BOOST_CHECK_CLOSE(bg::distance(collision,inter),0.5,0.0001);
    BOOST_CHECK_CLOSE(inter.x(), 2.0/3.0 + 0.5, 0.0001 );
    BOOST_CHECK_CLOSE(inter.y(), 0.0, 0.0001 );
    }

    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,3.0),0.5);
    Vector2d d1(1.0,1.0);
    Vector2d d2(0.0,0.0);

    Point inter, collision;

    CollisionPoints cp = c1.collisionPoints(d1,c2,d2);
    inter = cp.collisionPoint;
    collision = cp.object1CollisionPosition;

    double k = sqrt((((sqrt(0.5)-0.5)*(sqrt(0.5)-0.5))/2.0));

    BOOST_CHECK_CLOSE(collision.x(), 2.0+2*k, 0.0001 );
    BOOST_CHECK_CLOSE(collision.y(), 2.0+2*k, 0.0001 );
    BOOST_CHECK_CLOSE(bg::distance(collision,inter),0.5,0.0001);
    BOOST_CHECK_CLOSE(inter.x(), 2.5+k, 0.0001 );
    BOOST_CHECK_CLOSE(inter.y(), 2.5+k, 0.0001 );
    }
}

BOOST_AUTO_TEST_CASE( newDirection )
{/*
    {
    Circle c1(Point(0.0,0.0),0.5);
    Circle c2(Point(3.0,0.0),0.5);
    Vector2d d1(1.0,0.0);
    Vector2d d2(-1.0,0.0);

    Point inter, collision;

    boost::tie(inter,collision) = c1.collisionPoints(d1,c2,d2);
    Circle tmp_c1(Point(0.0,0.0),0.5), tmp_c2(Point(0.0,0.0),0.5);

    BOOST_CHECK_CLOSE(collision.x(), 2.0, 0.0001 );
    BOOST_CHECK_CLOSE(collision.y(), 2.0+, 0.0001 );
    BOOST_CHECK_CLOSE(bg::distance(collision,inter),0.5,0.0001);
    BOOST_CHECK_CLOSE(inter.x(), 2.5+k, 0.0001 );
    BOOST_CHECK_CLOSE(inter.y(), 2.5+k, 0.0001 );
    }*/
}

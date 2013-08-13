#include <vle/extension/mas/collision/Collision.hpp>
#include <vle/extension/mas/collision/Types.hpp>
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Intersection
#include <boost/test/unit_test.hpp>

#define DELTA 0.001

BOOST_AUTO_TEST_CASE( Intersection )
{
    {
    Point p1(0.0,0.0), p2(1.0,1.0), p3(0.0,1.0), p4(1.0,0.0);

    Point result = Collision::intersection(p1,p2,p3,p4);

    BOOST_REQUIRE_EQUAL(result.x(),0.5);
    BOOST_REQUIRE_EQUAL(result.y(),0.5);
    }

    {
    Point p1(0.0,0.0), p2(0.0,3.0), p3(-1.0,1.0), p4(1.0,1.0);

    Point result = Collision::intersection(p1,p2,p3,p4);

    BOOST_REQUIRE_EQUAL(result.x(),0.0);
    BOOST_REQUIRE_EQUAL(result.y(),1.0);
    }
}

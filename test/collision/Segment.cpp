#include <vle/extension/mas/collision/Types.hpp>
#include <vle/extension/mas/collision/Circle.hpp>
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Segment
#include <boost/test/unit_test.hpp>

#define DELTA 0.001

BOOST_AUTO_TEST_CASE( inCollisionTests )
{
    {
    Segment segment(Point(0.0,0.0),Point(0.0,10.0));
    segment.extends(Segment::END1,5.0);
    BOOST_REQUIRE_EQUAL(segment.getEnd1().y(), -5.0);
    segment.extends(Segment::END2,5.0);
    BOOST_REQUIRE_EQUAL(segment.getEnd2().y(), 15.0);
    }
    {
    Segment segment(Point(0.0,0.0),Point(1.0,1.0));
    segment.extends(Segment::END1,sqrt(2));
    BOOST_REQUIRE_EQUAL(segment.getEnd1().x(), -1.0);
    BOOST_REQUIRE_EQUAL(segment.getEnd1().y(), -1.0);

    segment.extends(Segment::END2,sqrt(2));
    BOOST_REQUIRE_EQUAL(segment.getEnd2().x(), 2.0);
    BOOST_REQUIRE_EQUAL(segment.getEnd2().y(), 2.0);
    }

}

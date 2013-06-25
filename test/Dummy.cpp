#define BOOST_TEST_MAIN
#define BOOST_AUTO_TEST_MAIN
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE package_test
#include <boost/test/unit_test.hpp>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <Dummy.hpp>
BOOST_AUTO_TEST_CASE(test_de_test)
{
    BOOST_REQUIRE(0 == 0);
    if( 2 == 3)
    {
        BOOST_REQUIRE(0 == 0);
    }
    foo(0);
}


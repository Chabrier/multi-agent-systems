#define BOOST_TEST_MAIN
#define BOOST_AUTO_TEST_MAIN
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE package_test
#include <boost/test/unit_test.hpp>
#include <boost/test/auto_unit_test.hpp>

#include <vle/extension/mas/Scheduler.hpp>

namespace vemas = vle::extension::mas;
struct A{
    A()
    {
        BOOST_TEST_MESSAGE( "setup fixture" );
    }
    ~A() { BOOST_TEST_MESSAGE( "teardown fixture" ); }

    vemas::Scheduler<int> s;
};

BOOST_FIXTURE_TEST_SUITE( heap_test, A)
BOOST_AUTO_TEST_CASE(sort_test)
{
    s.add_event(999);
    s.add_event(1);
    s.add_event(66);
    s.add_event(99);
    s.add_event(33);
    s.add_event(4);
    s.add_event(9);

    BOOST_REQUIRE_EQUAL((int)s.next_event(), 1);
    s.remove_next_event();
    BOOST_REQUIRE_EQUAL((int)s.next_event(), 4);
    s.remove_next_event();
    BOOST_REQUIRE_EQUAL((int)s.next_event(), 9);
    s.remove_next_event();
    BOOST_REQUIRE_EQUAL((int)s.next_event(), 33);
    s.remove_next_event();
    BOOST_REQUIRE_EQUAL((int)s.next_event(), 66);
    s.remove_next_event();
    BOOST_REQUIRE_EQUAL((int)s.next_event(), 99);
    s.remove_next_event();
    BOOST_REQUIRE_EQUAL((int)s.next_event(), 999);
}

BOOST_AUTO_TEST_CASE(empty_heap_test)
{
    if(!s.empty())
        BOOST_FAIL("empty() must return false!");

    try{
        s.next_event();
        BOOST_FAIL("Exception must be raised..");
    }catch(const std::exception& e){
        BOOST_TEST_MESSAGE("Exception successfully catched : ");
    }

    try{
        s.remove_next_event();
        BOOST_FAIL("Exception must be raised..");
    }catch(const std::exception& e){
        BOOST_TEST_MESSAGE("Exception successfully catched : ");
    }
}
BOOST_AUTO_TEST_SUITE_END();

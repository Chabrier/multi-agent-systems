#define BOOST_TEST_MAIN
#define BOOST_AUTO_TEST_MAIN
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE package_test
#include <boost/test/unit_test.hpp>
#include <boost/test/auto_unit_test.hpp>

#include <vle/extension/mas/Scheduler.hpp>

namespace vemas = vle::extension::mas;
struct A {
    A() {
        BOOST_TEST_MESSAGE("setup fixture");
    }
    ~A() {
        BOOST_TEST_MESSAGE("teardown fixture");
    }

    vemas::Scheduler<int> s;
};

BOOST_FIXTURE_TEST_SUITE(heap_test, A)
BOOST_AUTO_TEST_CASE(sort_test)
{
    s.addEffect(999);
    s.addEffect(1);
    s.addEffect(66);
    s.addEffect(99);
    s.addEffect(33);
    s.addEffect(4);
    s.addEffect(9);

    BOOST_REQUIRE_EQUAL((int)s.nextEffect(), 1);
    s.removeNextEffect();
    BOOST_REQUIRE_EQUAL((int)s.nextEffect(), 4);
    s.removeNextEffect();
    BOOST_REQUIRE_EQUAL((int)s.nextEffect(), 9);
    s.removeNextEffect();
    BOOST_REQUIRE_EQUAL((int)s.nextEffect(), 33);
    s.removeNextEffect();
    BOOST_REQUIRE_EQUAL((int)s.nextEffect(), 66);
    s.removeNextEffect();
    BOOST_REQUIRE_EQUAL((int)s.nextEffect(), 99);
    s.removeNextEffect();
    BOOST_REQUIRE_EQUAL((int)s.nextEffect(), 999);
}

BOOST_AUTO_TEST_CASE(empty_heap_test)
{
    if (!s.empty())
        BOOST_FAIL("empty() must return false!");

    try {
        s.nextEffect();
        BOOST_FAIL("Exception must be raised..");
    } catch (const std::exception& e) {
        BOOST_TEST_MESSAGE("Exception successfully catched : ");
    }

    try {
        s.removeNextEffect();
        BOOST_FAIL("Exception must be raised..");
    } catch (const std::exception& e) {
        BOOST_TEST_MESSAGE("Exception successfully catched : ");
    }
}
BOOST_AUTO_TEST_SUITE_END();

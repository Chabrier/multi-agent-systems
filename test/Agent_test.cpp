#define BOOST_TEST_MAIN
#define BOOST_AUTO_TEST_MAIN
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE package_test
#include <boost/shared_ptr.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <vle/value/Map.hpp>
#include <vle/devs/Dynamics.hpp>
#include <vle/vpz/AtomicModel.hpp>

#include <vle/extension/mas/Agent.hpp>

namespace vd = vle::devs;
namespace vemas = vle::extension::mas;


struct A {
    A() {
        //~ BOOST_TEST_MESSAGE( "setup fixture" );
        //~ vle::vpz::AtomicModel model("", 0);
        //~ vd::PackageId packageid;
        //~ vd::DynamicsInit dynInit(model, packageid);
//~
        //~ a.reset(new vemas::Agent(dynInit, vle::value::Map()));
    }
    ~A() {
        BOOST_TEST_MESSAGE("teardown fixture");
    }

    //~ boost::shared_ptr<vemas::Agent> a;
};

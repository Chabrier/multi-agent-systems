/*
 * This file is part of VLE, a framework for multi-modeling, simulation
 * and analysis of complex dynamical systems.
 * http://www.vle-project.org
 *
 * Copyright (c) 2013 INRA http://www.inra.fr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "GodProxy.hpp"
#include "GodLua.hpp"

namespace mas
{
namespace test
{
namespace dynamics
{

const char GodProxy::className[] = "GodProxy";

Lunar<GodProxy>::RegType GodProxy::methods[] = {
    LUNAR_DECLARE_METHOD(GodProxy, helloGod),
    LUNAR_DECLARE_METHOD(GodProxy, createSky),
    LUNAR_DECLARE_METHOD(GodProxy, createBird),
    LUNAR_DECLARE_METHOD(GodProxy, inANeigborhood),
    {0,0}
};

GodProxy::GodProxy(lua_State *L) {}

GodProxy::GodProxy() {}

void GodProxy::setGod ( God* pgod) {mGod = pgod;}

int GodProxy::helloGod(lua_State *L) {
    double nbi = luaL_checknumber(L, 1);
    std::string si (luaL_checkstring(L, 2));
    std::cout << "ADD " << nbi << " " << si << " " << lua_gettop(L) << std::endl;
    std::cout << "This was just to test!!!"<< std::endl;
    return 0;
}

int GodProxy::createSky(lua_State *L) {

    std::cout << "sky created by Lua!...i hope" << std::endl;

    mGod->createSky(luaL_checknumber(L, 1),
                    luaL_checknumber(L, 2),
                    luaL_checknumber(L, 3),
                    luaL_checknumber(L, 4));

    return 0;
}

int GodProxy::createBird(lua_State *L) {

    std::cout << "bird created by Lua!...i hope" << std::endl;

    mGod->createBird(luaL_checknumber(L, 1),
                     luaL_checknumber(L, 2),
                     luaL_checknumber(L, 3),
                     luaL_checknumber(L, 4),
                     luaL_checknumber(L, 5));

    return 0;
}

int GodProxy::inANeigborhood(lua_State *L) {

    std::cout << "check Neigborhood from Lua!...i hope" << std::endl;

    lua_pushboolean(L,
                    mGod->inANeigborhood(luaL_checknumber(L, 1),
                                         luaL_checknumber(L, 2)));


    return 1;
}

}
}
} // namespace vle example

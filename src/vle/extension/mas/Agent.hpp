/*
 * This file is part of VLE, a framework for multi-modeling, simulation
 * and analysis of complex dynamical systems
 * http://www.vle-project.org
 *
 * Copyright (c) 2013 INRA http://www.inra.fr
 *
 * See the AUTHORS or Authors.txt file for copyright owners and contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef AGENT_HPP
#define AGENT_HPP

#include <vle/devs/Time.hpp>
#include <vle/devs/Dynamics.hpp>

#include <vle/extension/mas/Scheduler.hpp>
#include <vle/extension/mas/Events.hpp>

using namespace boost;
namespace vd = vle::devs;
namespace vv = vle::value;

namespace vle {
namespace extension {
namespace mas {

class Agent{
public:
    virtual void init() = 0;
protected:
    /*! \var mEventsToSend
    *\brief Vector containing event to send
    * Events are sent to "agent_output port" and deleted when DEVS output
    * function is called.
    *\see http://www.vle-project.org/wiki/Mod%C3%A8le_atomique_fonction_de_sortie
    */

    std::vector<Event> mEventsToSend;
    vd::Time mLastUpdate;
private:
};
}}}// namespace vle extension mas

#endif

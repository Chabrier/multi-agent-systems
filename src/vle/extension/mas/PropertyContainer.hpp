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
#ifndef PROPERTY_CONTAINER
#define PROPERTY_CONTAINER

#include <vle/value/Value.hpp>
#include <unordered_map>

namespace vle {
namespace extension {
namespace mas {

namespace vv = vle::value;

class PropertyContainer
{
/* Public types */
public:
    typedef std::shared_ptr<vv::Value> value_ptr;
    typedef std::unordered_map<std::string, value_ptr> property_map;

/* Public functions */
public:
    inline const property_map& getInformations() const
    {return mInformations;}

    inline property_map& getInformations()
    {return mInformations;}

/* Private members */
private:
    property_map mInformations;
};

}
}
}// namespace vle extension mas

#endif


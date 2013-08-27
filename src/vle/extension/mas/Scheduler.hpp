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
#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

#include <stdexcept>
#include <algorithm>

namespace vle {
namespace extension {
namespace mas {

template <typename T>
class Scheduler
{
public:
    typedef typename std::vector<T> Elements;

    /* Modifiers */
    /** @brief Add element*/
    inline void addEffect(const T& t)
    {
        if(!exists(t)) {
            mElements.push_back(t);
            std::sort(mElements.begin(),mElements.end());
        } else {
            throw std::logic_error("Scheduler already contains this element");
        }
    }

    /** @brief Remove minimal element*/
    inline void removeNextEffect()
    {
        if (mElements.empty())
            throw std::logic_error("Scheduler is empty");
        mElements.erase(mElements.begin());
    }

    inline void update(const T& t)
    {
        if (!exists(t))
            throw std::logic_error("Scheduler doesn't contain this element");

        std::replace(mElements.begin(), mElements.end(), t, t);
        std::sort(mElements.begin(),mElements.end());
    }

    /* Observers */
    /** @brief Check if scheduler is empty
     *  @return boolean true if empty, false otherwise*/
    inline bool empty()const
    {return mElements.empty();}

    /** @brief Get number of elements
     *  @return size_t number of elements*/
    inline size_t size() const
    {return mElements.size();}

    inline bool exists(const T& t)
    {return std::find(mElements.begin(),mElements.end(),t) != mElements.end();}

    /* Element access */
    /** @brief Get next elements of scheduler */
    inline const T& nextEffect() const
    {
        if (mElements.empty())
            throw std::logic_error("Scheduler is empty");
        return mElements.at(0);
    }

    const Elements& elements() const {return mElements;}
protected:
private:
    Elements mElements;
};

}
}
}// namespace vle extension mas

#endif

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
    typedef typename std::vector<T> Heap;

    /* Modifiers */
    /** @brief Add element*/
    inline void addEffect(const T& t)
    {
        if(!exists(t)) {
            mHeap.push_back(t);
            std::sort(mHeap.begin(),mHeap.end());
        } else {
            throw std::logic_error("Scheduler already contains this element");
        }
    }

    /** @brief Remove minimal element*/
    inline void removeNextEffect()
    {
        if (mHeap.empty())
            throw std::logic_error("Scheduler is empty");
        mHeap.erase(mHeap.begin());
    }

    inline void update(const T& t)
    {
        if (!exists(t))
            throw std::logic_error("Scheduler doesn't contain this element");

        std::replace(mHeap.begin(), mHeap.end(), t, t);
        std::sort(mHeap.begin(),mHeap.end());
    }

    /* Observers */
    /** @brief Check if scheduler is empty
     *  @return boolean true if empty, false otherwise*/
    inline bool empty()const
    {return mHeap.empty();}

    /** @brief Get number of elements
     *  @return size_t number of elements*/
    inline size_t size() const
    {return mHeap.size();}

    inline bool exists(const T& t)
    {return std::find(mHeap.begin(),mHeap.end(),t) != mHeap.end();}

    /* Element access */
    /** @brief Get next elements of scheduler */
    inline const T& nextEffect() const
    {
        if (mHeap.empty())
            throw std::logic_error("Scheduler is empty");
        return mHeap.at(0);
    }

    const Heap& elements() const {return mHeap;}
protected:
private:
    Heap mHeap;
};

}
}
}// namespace vle extension mas

#endif

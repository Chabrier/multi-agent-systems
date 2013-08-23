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

#include <boost/heap/fibonacci_heap.hpp>

namespace vle
{
namespace extension
{
namespace mas
{

template <typename T>
class Scheduler
{
public:
    typedef typename boost::heap::fibonacci_heap
                     <T,boost::heap::compare<std::greater<T> > >
                     Heap;

    /* Modifiers */
    /** @brief Add element*/
    inline void addEffect(const T& t)
    {
        mHeap.push(t);
    }

    /** @brief Remove minimal element*/
    void removeNextEffect()
    {
        if (mHeap.empty())
            throw std::out_of_range("Scheduler is empty");
        mHeap.pop();

    }

    /* Observers */
    /** @brief Check if scheduler is empty
     *  @return boolean true if empty, false otherwise*/
    inline bool empty()const
    {
        return mHeap.empty();
    }

    /** @brief Get number of elements
     *  @return size_t number of elements*/
    inline size_t size() const
    {
        return mHeap.size();
    }

    /* Element access */
    /** @brief Get next elements of scheduler */
    const T& nextEffect() const
    {
        if (mHeap.empty())
            throw std::out_of_range("Scheduler is empty");
        return mHeap.top();
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

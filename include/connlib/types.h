/*******************************************************************************
 *
 * Copyright (c) 2017, Peter Andreas Entschev
 * All rights reserved.
 *
 * BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#pragma once

#include <cfloat>
#include <cstddef>
#include <cmath>
#include <limits>
#include <memory>
#include <list>

namespace connlib
{


struct Node
{
    Node()
        : x(std::numeric_limits<float>::quiet_NaN()),
          y(std::numeric_limits<float>::quiet_NaN()),
          index(-1)
    {}

    Node(const float x, const float y)
        : x(x), y(y), index(-1)
    {}

    float x;
    float y;
    size_t index;
};

typedef std::shared_ptr<Node> NodePtr;
typedef std::list<Node> NodeList;
typedef std::list<NodePtr> NodePtrList;

struct Edge
{
    Edge()
        : node1(nullptr),
          node2(nullptr),
          length(std::numeric_limits<double>::quiet_NaN())
    {}

    Edge(const NodePtr& node1, const NodePtr& node2, const double length)
        : node1(node1), node2(node2), length(length)
    {}

    static bool equal(const Edge& e1, const Edge& e2, double eps = 1e-3)
    {
        return (std::abs(e1.node1->x - e2.node1->x) < eps &&
                std::abs(e1.node1->y - e2.node1->y) < eps &&
                std::abs(e1.node2->x - e2.node2->x) < eps &&
                std::abs(e1.node2->y - e2.node2->y) < eps &&
                std::abs(e1.length - e2.length) < eps);
    }

    static bool greater(const Edge& e1, const Edge& e2)
    {
        if (e1.node1->x < e2.node1->x) return true;
        if (e1.node1->y < e2.node1->y) return true;
        if (e1.node2->x < e2.node2->x) return true;
        if (e1.node2->y < e2.node2->y) return true;
        if (e1.length < e2.length) return true;
        return false;
    }

    NodePtr node1;
    NodePtr node2;
    double length;
};

typedef std::list<Edge> EdgeList;

typedef enum
{
    CONNLIB_SUCCESS = 0,
    CONNLIB_NEGATIVE_COORDINATE,
    CONNLIB_NAN_COORDINATE,
    CONNLIB_INF_COORDINATE
} ConnLibStatus;

} // namespace lib

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

#include <connlib/utils.h>

namespace connlib
{

bool EdgeLengthGreater::operator()(const Edge& e1, const Edge& e2)
{
    return e1.length > e2.length;
}

EdgeEqual::EdgeEqual(double eps)
    : m_eps(eps)
{
}

bool EdgeEqual::operator()(const Edge& e1, const Edge& e2)
{
    return (std::abs(e1.node1->x - e2.node1->x) < m_eps &&
            std::abs(e1.node1->y - e2.node1->y) < m_eps &&
            std::abs(e1.node2->x - e2.node2->x) < m_eps &&
            std::abs(e1.node2->y - e2.node2->y) < m_eps &&
            std::abs(e1.length - e2.length) < m_eps);
}

bool compareEdgeList(EdgeList& list1, EdgeList& list2, double eps)
{
    if (list1.size() != list2.size()) return false;

    list1.sort(Edge::greater);
    list2.sort(Edge::greater);

    return std::equal(list1.begin(), list1.end(), list2.begin(),
                      EdgeEqual(1e-3f));
}

void addNode(NodeList& list, const float x, const float y)
{
    list.push_back(Node(x, y));
}

void addEdge(EdgeList& list, const float n1X, const float n1Y,
        const float n2X, const float n2Y, const float length)
{
    NodePtr node1(std::make_shared<connlib::Node>(n1X, n1Y));
    NodePtr node2(std::make_shared<connlib::Node>(n2X, n2Y));
    Edge edge(node1, node2, length);
    list.push_back(edge);
}

} // namespace connlib

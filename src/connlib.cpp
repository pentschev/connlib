/*******************************************************************************
 *
 * Copyright (c) 2018, Peter Andreas Entschev
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

#include <connlib/connlib.h>

#include <cfloat>
#include <cmath>
#include <queue>
#include <utility>
#include <iostream>

namespace connlib
{

ConnLibSolver::ConnLibSolver()
    : m_totalLength(FLT_MAX)
{
}

void ConnLibSolver::clear()
{
    m_connections.clear();
    m_nodes.clear();
    m_totalLength = FLT_MAX;
}

ConnLibStatus ConnLibSolver::checkInput(const NodeList& nodes)
{
    for (NodeList::const_iterator it = nodes.begin(); it != nodes.end(); it++)
    {
        Node n = *it;

        if (n.x < 0.0f || n.y < 0.0f)
            return CONNLIB_NEGATIVE_COORDINATE;
        if (std::isnan(n.x) || std::isnan(n.y))
            return CONNLIB_NAN_COORDINATE;
        if (std::isinf(n.x) || std::isinf(n.y))
            return CONNLIB_INF_COORDINATE;
    }

    return CONNLIB_SUCCESS;
}

void ConnLibSolver::initializeConnections(
        std::vector<EdgeList>& adjacencyList, const NodeList& nodes)
{
    size_t N = nodes.size();
    adjacencyList.resize(N);

    for (NodeList::const_iterator it = nodes.begin(); it != nodes.end(); it++)
    {
        NodePtr node(std::make_shared<Node>(*it));
        node->index = std::distance(nodes.begin(), it);
        m_nodes.push_back(node);
    }

    for (NodePtrList::const_iterator it = m_nodes.begin(); it != m_nodes.end(); it++)
    {
        if (std::next(it, 1) == m_nodes.end()) break;

        NodePtr n1 = *it;
        for (NodePtrList::const_iterator it2 = std::next(it, 1); it2 != m_nodes.end(); it2++)
        {
            NodePtr n2 = *it2;
            double length = calculateLength(n1, n2);
            adjacencyList[n1->index].push_back(Edge(n1, n2, length));
            adjacencyList[n2->index].push_back(Edge(n2, n1, length));
        }
    }
}

ConnLibStatus ConnLibSolver::operator()(const NodeList& nodes)
{
    clear();

    if (nodes.size() < 2)
    {
        m_totalLength = 0.0f;
        return CONNLIB_SUCCESS;
    }

    ConnLibStatus status = checkInput(nodes);
    if (status != CONNLIB_SUCCESS) return status;

    size_t N = nodes.size();

    std::vector<EdgeList> adjacencyList;
    initializeConnections(adjacencyList, nodes);

    std::vector<Edge> parentEdge(N, Edge());
    std::vector<bool> visited(N, false);
    std::vector<double> bestLength(N, FLT_MAX);

    std::priority_queue<Edge, std::vector<Edge>, EdgeGreater> q;

    NodePtr frontNode = m_nodes.front();
    q.push(Edge(nullptr, frontNode, 0.0f));
    bestLength[frontNode->index] = 0.0f;

    /// Iterate through all the nodes via priority_queue
    while (!q.empty())
    {
        Edge edge = q.top();
        q.pop();

        NodePtr srcNode = edge.node2;
        size_t idx = srcNode->index;
        visited[idx] = true;

        for (EdgeList::iterator it = adjacencyList[idx].begin();
                it != adjacencyList[idx].end(); it++)
        {
            NodePtr dstNode = it->node2;
            size_t dstIdx = dstNode->index;
            double length = it->length;

            if (!visited[dstIdx] && length < bestLength[dstIdx])
            {
                bestLength[dstIdx] = length;

                Edge edge = Edge(srcNode, dstNode, length);
                parentEdge[dstIdx] = edge;
                q.push(edge);
            }
        }
    }

    // Calculate total length
    m_totalLength = 0.f;
    for (std::vector<double>::iterator it = bestLength.begin();
            it != bestLength.end(); it++)
        m_totalLength += *it;

    // Create connection list
    // First parent is only a reference for the first node, ignore it
    for (std::vector<Edge>::iterator it = parentEdge.begin()+1;
            it != parentEdge.end(); it++)
        m_connections.push_back(*it);

    return CONNLIB_SUCCESS;
}

EdgeList ConnLibSolver::getConnections()
{
    return m_connections;
}

double ConnLibSolver::getTotalLength()
{
    return m_totalLength;
}

double ConnLibSolver::calculateLength(const NodePtr& n1, const NodePtr& n2)
{
    double x = static_cast<double>(n1->x) - static_cast<double>(n2->x);
    double y = static_cast<double>(n1->y) - static_cast<double>(n2->y);
    return sqrt(x*x + y*y);
}

} // namespace connlib

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
#include <connlib/utils.h>

#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

TEST(InputCheck, NegativeCoordinate)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;
    nodeList.push_back(connlib::Node(0.0f,  1.0f));
    nodeList.push_back(connlib::Node(0.0f, -1.0f));

    EXPECT_EQ(connlib::CONNLIB_NEGATIVE_COORDINATE, cls(nodeList));
}

TEST(InputCheck, InfCoordinate)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    float inf = std::numeric_limits<float>::infinity();
    nodeList.push_back(connlib::Node(0.0f, 1.0f));
    nodeList.push_back(connlib::Node(inf, 0.0f));

    EXPECT_EQ(connlib::CONNLIB_INF_COORDINATE, cls(nodeList));
}

TEST(InputCheck, NaNCoordinate)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    float nan = std::numeric_limits<float>::quiet_NaN();
    nodeList.push_back(connlib::Node(0.0f, 1.0f));
    nodeList.push_back(connlib::Node(nan, 0.0f));

    EXPECT_EQ(connlib::CONNLIB_NAN_COORDINATE, cls(nodeList));
}

TEST(FindPath, EmptyList)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    EXPECT_EQ(true, cls.getConnections().empty());
    EXPECT_EQ(0.0f, cls.getTotalLength());
}

TEST(FindPath, 1Node)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;
    nodeList.push_back(connlib::Node(0.0f, 1.0f));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    EXPECT_EQ(true, cls.getConnections().empty());
    EXPECT_EQ(0.0f, cls.getTotalLength());
}


TEST(FindPath, 2Nodes)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    nodeList.push_back(connlib::Node(0.0f, 0.0f));
    nodeList.push_back(connlib::Node(0.0f, 1.0f));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    EXPECT_EQ(1.0f, cls.getTotalLength());
}

TEST(FindPath, 3Nodes)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    nodeList.push_back(connlib::Node(0.0f, 0.0f));
    nodeList.push_back(connlib::Node(0.0f, 1.0f));
    nodeList.push_back(connlib::Node(1.0f, 1.0f));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    EXPECT_EQ(2.0f, cls.getTotalLength());
}

TEST(FindPath, 100NodesVertical)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    size_t totalNodes = 100;
    for (size_t n = 0; n < totalNodes; n++)
        nodeList.push_back(connlib::Node(0.0f, static_cast<float>(n)));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    float expectedLength = totalNodes - 1.f;
    ASSERT_NEAR(expectedLength, cls.getTotalLength(), 1e-3f);

    connlib::EdgeList edgeList = cls.getConnections();
    EXPECT_EQ(totalNodes-1, edgeList.size());

    connlib::EdgeList expectedEdgeList;

    for(connlib::EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); it++)
    {
        float idx = static_cast<float>(std::distance(edgeList.begin(), it));

        connlib::addEdge(expectedEdgeList, 0.0f, idx, 0.0f, idx+1, 1.0f);

        ASSERT_NEAR(expectedEdgeList.back().node1->x, it->node1->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->x, it->node2->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node1->y, it->node1->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->y, it->node2->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().length, it->length, 1e-3f);
    }

    EXPECT_EQ(expectedEdgeList.size(), edgeList.size());

    ASSERT_TRUE(connlib::compareEdgeList(edgeList, expectedEdgeList));
}

TEST(FindPath, 100NodesHorizontal)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    size_t totalNodes = 100;
    for (size_t n = 0; n < totalNodes; n++)
        nodeList.push_back(connlib::Node(static_cast<float>(n), 0.0f));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    float expectedLength = totalNodes - 1.f;
    ASSERT_NEAR(expectedLength, cls.getTotalLength(), 1e-3f);

    connlib::EdgeList edgeList = cls.getConnections();
    EXPECT_EQ(totalNodes-1, edgeList.size());

    connlib::EdgeList expectedEdgeList;

    for(connlib::EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); it++)
    {
        float idx = static_cast<float>(std::distance(edgeList.begin(), it));

        connlib::addEdge(expectedEdgeList, idx, 0.0f, idx+1, 0.0f, 1.0f);

        ASSERT_NEAR(expectedEdgeList.back().node1->x, it->node1->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->x, it->node2->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node1->y, it->node1->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->y, it->node2->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().length, it->length, 1e-3f);
    }

    EXPECT_EQ(expectedEdgeList.size(), edgeList.size());

    ASSERT_TRUE(connlib::compareEdgeList(edgeList, expectedEdgeList));
}

TEST(FindPath, 100NodesDiagonal)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    size_t totalNodes = 100;
    for (size_t n = 0; n < totalNodes; n++)
        nodeList.push_back(connlib::Node(static_cast<float>(n),
                                         static_cast<float>(n)));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    float expectedLength = totalNodes * sqrt(2.f) - sqrt(2.f);
    ASSERT_NEAR(expectedLength, cls.getTotalLength(), 1e-3f);

    connlib::EdgeList edgeList = cls.getConnections();
    EXPECT_EQ(totalNodes-1, edgeList.size());

    connlib::EdgeList expectedEdgeList;

    for(connlib::EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); it++)
    {
        float idx = static_cast<float>(std::distance(edgeList.begin(), it));

        connlib::addEdge(expectedEdgeList, idx, idx, idx+1, idx+1, sqrt(2.0f));

        ASSERT_NEAR(expectedEdgeList.back().node1->x, it->node1->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->x, it->node2->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node1->y, it->node1->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->y, it->node2->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().length, it->length, 1e-3f);
    }

    EXPECT_EQ(expectedEdgeList.size(), edgeList.size());

    ASSERT_TRUE(connlib::compareEdgeList(edgeList, expectedEdgeList));
}

TEST(FindPath, 100NodesDiagonalInverted)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    size_t totalNodes = 100;
    for (size_t n = 0; n < totalNodes; n++)
        nodeList.push_back(connlib::Node(static_cast<float>(totalNodes-n-1),
                                         static_cast<float>(totalNodes-n-1)));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    float expectedLength = totalNodes * sqrt(2.f) - sqrt(2.f);
    ASSERT_NEAR(expectedLength, cls.getTotalLength(), 1e-3f);

    connlib::EdgeList edgeList = cls.getConnections();
    EXPECT_EQ(totalNodes-1, edgeList.size());

    connlib::EdgeList expectedEdgeList;

    for(connlib::EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); it++)
    {
        float idx = static_cast<float>(
                totalNodes - std::distance(edgeList.begin(), it) - 1);

        connlib::addEdge(expectedEdgeList, idx, idx, idx-1, idx-1, sqrt(2.0f));

        ASSERT_NEAR(expectedEdgeList.back().node1->x, it->node1->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->x, it->node2->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node1->y, it->node1->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->y, it->node2->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().length, it->length, 1e-3f);
    }

    EXPECT_EQ(expectedEdgeList.size(), edgeList.size());

    ASSERT_TRUE(connlib::compareEdgeList(edgeList, expectedEdgeList));
}

TEST(FindPath, 500NodesDiagonal)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    size_t totalNodes = 500;
    for (size_t n = 0; n < totalNodes; n++)
        nodeList.push_back(connlib::Node(static_cast<float>(n),
                                         static_cast<float>(n)));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    double expectedLength = totalNodes * sqrt(2.f) - sqrt(2.f);
    ASSERT_NEAR(expectedLength, cls.getTotalLength(), 1e-3f);

    connlib::EdgeList edgeList = cls.getConnections();
    EXPECT_EQ(totalNodes-1, edgeList.size());

    connlib::EdgeList expectedEdgeList;

    for(connlib::EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); it++)
    {
        float idx = static_cast<float>(std::distance(edgeList.begin(), it));

        connlib::addEdge(expectedEdgeList, idx, idx, idx+1, idx+1, sqrt(2.0f));

        ASSERT_NEAR(expectedEdgeList.back().node1->x, it->node1->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->x, it->node2->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node1->y, it->node1->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->y, it->node2->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().length, it->length, 1e-3f);
    }

    EXPECT_EQ(expectedEdgeList.size(), edgeList.size());

    ASSERT_TRUE(connlib::compareEdgeList(edgeList, expectedEdgeList));
}

TEST(FindPath, 1000NodesDiagonal)
{
    connlib::ConnLibSolver cls;
    connlib::NodeList nodeList;

    size_t totalNodes = 1000;
    for (size_t n = 0; n < totalNodes; n++)
        nodeList.push_back(connlib::Node(static_cast<float>(n),
                                         static_cast<float>(n)));

    EXPECT_EQ(connlib::CONNLIB_SUCCESS, cls(nodeList));

    double expectedLength = totalNodes * sqrt(2.f) - sqrt(2.f);
    ASSERT_NEAR(expectedLength, cls.getTotalLength(), 1e-3f);

    connlib::EdgeList edgeList = cls.getConnections();
    EXPECT_EQ(totalNodes-1, edgeList.size());

    connlib::EdgeList expectedEdgeList;

    for(connlib::EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); it++)
    {
        float idx = static_cast<float>(std::distance(edgeList.begin(), it));

        connlib::addEdge(expectedEdgeList, idx, idx, idx+1, idx+1, sqrt(2.0f));

        ASSERT_NEAR(expectedEdgeList.back().node1->x, it->node1->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->x, it->node2->x, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node1->y, it->node1->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().node2->y, it->node2->y, 1e-3f);
        ASSERT_NEAR(expectedEdgeList.back().length, it->length, 1e-3f);
    }

    EXPECT_EQ(expectedEdgeList.size(), edgeList.size());

    ASSERT_TRUE(connlib::compareEdgeList(edgeList, expectedEdgeList));
}

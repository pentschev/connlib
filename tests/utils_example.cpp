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

#include <iomanip>
#include <iostream>

#include <algorithm>

int main()
{
    // Allocate a list of nodes
    connlib::NodeList nodeList;

    // Populate nodes list. First argument is the list object, second and third
    // arguments are x and y coordinates, respectively.
    addNode(nodeList, 0.0f, 0.0f);
    addNode(nodeList, 1.0f, 1.0f);
    addNode(nodeList, 2.0f, 2.0f);

    // Instantiate connection solver
    connlib::ConnLibSolver cls;

    // Compute connections
    connlib::ConnLibStatus status = cls(nodeList);

    // Check for success
    if (status != connlib::CONNLIB_SUCCESS)
        return status;

    // Get the resulting edge list and total length
    connlib::EdgeList edgeList = cls.getConnections();
    double totalLength = cls.getTotalLength();

    // Returns error if total length is wrong
    if (totalLength != 2.0f*sqrt(2.0f)) return -1;

    // Allocates a list of expected edge results
    connlib::EdgeList expectedEdgeList;
    addEdge(expectedEdgeList, 0.0f, 0.0f, 1.0f, 1.0f, sqrt(2.0f));
    addEdge(expectedEdgeList, 1.0f, 1.0f, 2.0f, 2.0f, sqrt(2.0f));

    // Populate expected edges list. The first argument is the list of edges,
    // second and third are the x and y coordinates of the source node, fourth
    // and fifth are the x and y coordinates of the destination node, sixth is
    // the length of the connection.
    connlib::compareEdgeList(edgeList, expectedEdgeList);

    // Compares two edge lists, returns true if they match, false otherwise.
    // First two arguments are the lists, last argument is the maximum error
    // allowed for each coordinate and length.
    bool isEqual = connlib::compareEdgeList(edgeList, expectedEdgeList, 1e-3f);
    if (!isEqual) return -1;

    return status;
}

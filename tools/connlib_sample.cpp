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
    connlib::ConnLibSolver cls;

    connlib::NodeList nodeList;

    std::cout << std::fixed << std::setprecision(6);

    size_t numNodes;

    std::cin >> numNodes;
    for (size_t n = 0; n < numNodes; n++)
    {
        float x, y;
        std::cin >> x >> y;
        nodeList.push_back(connlib::Node(x, y));
    }

    connlib::ConnLibStatus status = cls(nodeList);
    if (status != connlib::CONNLIB_SUCCESS)
        return status;

    connlib::EdgeList edgeList = cls.getConnections();
    connlib::EdgeList edgeList2 = cls.getConnections();
    float totalLength = cls.getTotalLength();

    std::cout << cls.getTotalLength() << std::endl;
    for(connlib::EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); it++)
    {
        std::cout << it->node1->x << " " << it->node1->y << " "
            << it->node2->x << " " << it->node2->y << " "
            << it->length << std::endl;
    }

    return status;
}

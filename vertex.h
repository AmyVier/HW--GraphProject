/**
 *  Vertex class - Vertex of edges, allowing user to perform
 *  traversal algorithms
 *
 * @author Amy Vier and Elaine Hsu
 * @date 29 Apr 2024
 *
 * ---------------------
 * Vertices can be connected to each other through edges with a certain cost/distance.
 *
 * ---------------------
 * An edge cannot contain more than one destination
 * A vertex can be connected to other vertices via weighted, directed edge.
 * A vertex cannot connect to itself or have multiple edges to the same vertex
 */

#ifndef VERTEX_H
#define VERTEX_H

#include <map>
#include <string>

using namespace std;

class Vertex
{
    friend class Graph;

    // constructor
    Vertex(string value) {
        this->value = value;
    }

    struct Edge
    {
        friend class Graph;

        // constructor
        Edge(int distance, Vertex *destination)
        {
            this->distance = distance;
            this->destination = destination;
        }

    private:
        // distance from current vertex to vertex of destination
        int distance = 0;

        // destination vertex
        Vertex *destination = nullptr;
    };

private:
    string value;              // string name/value of vertex
    map<string, Edge *> edges; // all edges of vertex
};

#endif // VERTEX_H
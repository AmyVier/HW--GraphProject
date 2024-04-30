// Amy Vier and Elaine Hsu
#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>
#include <stack>

using namespace std;

// Vertex constructor
Vertex::Vertex(string value)
{
  this->value = value;
}

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges)
{
  this->directionalEdges = directionalEdges;
}

// destructor
Graph::~Graph()
{
  for (auto vertex : vertices)
  {
    for (auto edge : vertex.second->edges)
    {
      delete edge.second;
    }
    delete vertex.second;
  }
}

// @return total number of vertices
int Graph::verticesSize() const
{
  return vertices.size();
}

// @return total number of edges
int Graph::edgesSize() const
{
  int numberOfEdges = 0;
  for (auto vertex : vertices)
  {
    numberOfEdges += vertex.second->edges.size();
  }

  if (!directionalEdges)
  {
    numberOfEdges = numberOfEdges / 2;
  }

  return numberOfEdges;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const
{
  Vertex *vertexFind; // vertex found

  if (vertices.count(label))
  {
    vertexFind = vertices.at(label);
  }
  else
  {
    return -1;
  }

  return vertexFind->edges.size();
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label)
{
  if (vertices.count(label))
  {
    return false;
  }
  else
  {
    vertices.insert({label, new Vertex(label)});
  }

  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const
{
  return vertices.count(label);
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const
{
  Vertex *vertexFind;     // vertex found
  string edgeString = ""; // string representing edges and weights

  // if vertex does not exist
  if (vertices.count(label))
  {
    vertexFind = vertices.at(label);
  }
  else
  {
    return "";
  }

  // iterate through edges
  for (auto vertex : vertexFind->edges)
  {
    edgeString = edgeString + vertex.second->destination->value +
                 "(" + to_string(vertex.second->distance) + "),";
  }

  // get rid of last comma
  if (edgeString.size() > 0)
  {
    edgeString = edgeString.substr(0, edgeString.size() - 1);
  }

  return edgeString;
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight)
{
  Vertex *fromVertex;
  Vertex *toVertex;

  // vertex cannot connect to itself
  if (from == to)
  {
    return false;
  }

  // add if either vertices does not exist
  if (!(vertices.count(from)))
  {
    add(from);
  }

  if (!(vertices.count(to)))
  {
    add(to);
  }

  fromVertex = vertices[from];
  toVertex = vertices[to];

  // if edge already exists, return false
  if (fromVertex->edges.count(to))
  {
    return false;
  }

  // connect
  fromVertex->edges.insert({to, new Vertex::Edge(weight, toVertex)});

  // if not directional, add one more connection of opposite direction
  if (!(directionalEdges))
  {
    toVertex->edges.insert({from, new Vertex::Edge(weight, fromVertex)});
  }

  return true;
}

bool Graph::disconnect(const string &from, const string &to)
{
  Vertex *fromVertex;
  Vertex *toVertex;

  // get vertices if found, return false if not found
  if ((from != to) && (vertices.count(from) && vertices.count(to)))
  {
    fromVertex = vertices[from];
    toVertex = vertices[to];
  }
  else
  {
    return false;
  }

  // if connection does not exist, return false, if exists, disconnect
  if (fromVertex->edges.count(to))
  {
    delete fromVertex->edges.at(to);
    fromVertex->edges.erase(to);
  }
  else
  {
    return false;
  }

  // if not directional get rid of connection of opposite direction
  if (!(directionalEdges))
  {
    delete toVertex->edges.at(from);
    toVertex->edges.erase(from);
  }

  return true;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label))
{
  set<string> visited;     // keep track of visited vertices
  stack<Vertex *> toVisit; // keep tack of vertices to visit
  Vertex *currentVertex;

  // check if vertex exists
  if (vertices.count(startLabel))
  {
    toVisit.push(vertices[startLabel]);
    visited.insert(startLabel);
  }
  else
  {
    return;
  }

  while (!(toVisit.empty()))
  {
    // go to new vertex
    currentVertex = toVisit.top();
    toVisit.pop();
    visit(currentVertex->value);

    // iterate over map of edges in reverse order to visit edges in
    // alphabetical order
    map<string, Vertex::Edge *>::reverse_iterator it;

    // iterate over edges
    for (it = currentVertex->edges.rbegin(); it != currentVertex->edges.rend(); it++)
    {
      // add to stack if not visited
      if (!(visited.count(it->second->destination->value)))
      {
        visited.insert(it->second->destination->value);
        toVisit.push(it->second->destination);
      }
    }
  }
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label))
{
  set<string> visited;     // keep track of visited vertices
  queue<Vertex *> toVisit; // keep track of vertices to visit
  Vertex *currentVertex;

  // check if vertex exists
  if (vertices.count(startLabel))
  {
    toVisit.push(vertices[startLabel]);
    visited.insert(startLabel);
  }
  else
  {
    return;
  }

  while (!(toVisit.empty()))
  {
    // go to new vertex
    currentVertex = toVisit.front();
    toVisit.pop();
    visit(currentVertex->value);

    // iterate over edges
    for (auto edge : currentVertex->edges)
    {
      // add to queue if not visited
      if (!(visited.count(edge.second->destination->value)))
      {
        visited.insert(edge.second->destination->value);
        toVisit.push(edge.second->destination);
      }
    }
  }
}

// store the weights in a map
// store the previous label in a map
// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const
{
  map<string, int> weights;     // store weights
  map<string, string> previous; // store previous vertices/path

  // priority queue of the path cost from the start vertex to
  // the vertex stored, the previous vertex of the vertex stored,
  // and vertex stored. Sorted by the least cost.
  // priority_queue < {path cost, {previous vertex, vertex}} >
  priority_queue<pair<int, pair<string, Vertex *>>,
                 vector<pair<int, pair<string, Vertex *>>>,
                 greater<pair<int, pair<string, Vertex *>>>>
      toVisit;

  Vertex *currentVertex;
  int path = 0;                      // path cost
  string previousLabel = startLabel; // previous vertex

  // check if vertex exists
  if (vertices.count(startLabel))
  {
    toVisit.push(make_pair(0, make_pair(startLabel, vertices.at(startLabel))));
  }
  else
  {
    return make_pair(weights, previous);
  }

  while (!(toVisit.empty()))
  {
    currentVertex = toVisit.top().second.second; // go to vertex
    path = toVisit.top().first;                  // get path of vertex
    previousLabel = toVisit.top().second.first;  // get previous vertex of vertex
    toVisit.pop();

    // explore new vertex if the vertex is unexplored or the new path of the
    // vertex is less than the old stored path of the vertex
    if (!(weights.count(currentVertex->value)) ||
        weights.at(currentVertex->value) > path)
    {
      // store weight and previous node into maps
      if (!(weights.count(currentVertex->value)))
      {
        weights.insert({currentVertex->value, path});
        previous.insert({currentVertex->value, previousLabel});
      }
      else if (weights.at(currentVertex->value) > path)
      {
        previous[currentVertex->value] = previousLabel;
        weights[currentVertex->value] = path;
      }

      // iterate over edges
      for (auto edge : currentVertex->edges)
      {
        // explore edges if the edges or unexplored of the cost path is
        // less than the cost path stored
        if (!(weights.count(edge.second->destination->value)) ||
            (path + edge.second->distance <
             weights.at(edge.second->destination->value)))
        {
          toVisit.push(make_pair(path + edge.second->distance,
                                 make_pair(currentVertex->value, edge.second->destination)));
        }
      }
    }
  }

  // remove start vertext looping back to itself ( [start: 0] and [start: start])
  weights.erase(startLabel);
  previous.erase(startLabel);

  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const
{

  return -1;
}

// minimum spanning tree using Krustal's algorithm
int Graph::mstKruskal(const string &startLabel,
                      void visit(const string &from, const string &to,
                                 int weight)) const
{
  // priority queue of edges sorted by the least cost
  // priority queue < {cost, {from vertex, to vertex}}
  priority_queue<pair<int, pair<string, string>>,
                 vector<pair<int, pair<string, string>>>,
                 greater<pair<int, pair<string, string>>>>
      edges;

  // vector of culsters of vertices that are connected to one another
  // containing the cost of all edges connecting each vertex in the cluster
  // and indexes of which clusters the cluster is connected to
  // vector < {cost of all edges in cluster, {indexes of all clusters the
  // cluster is connected to, all vertices in the cluster}}
  vector<pair<int, pair<set<int>, set<string>>>> minimumSpanningTree;

  // cluster index the start vertex is connected to
  int startVertexConnection = -1;

  // total edge cost
  int total = 0;

  // store clusters that contain vertices connected to start vertex
  queue<int> minimumSpanningTreeTraversal;

  // indexes of visited clusters
  set<int> visited;

  // check if vertex exists and if graph is not directional
  if (!(vertices.count(startLabel)) || directionalEdges)
  {
    return -1;
  }

  // store edges by least cost
  for (auto vertex : vertices)
  {
    for (auto edge : vertex.second->edges)
    {
      edges.push(make_pair(edge.second->distance,
                           make_pair(vertex.first, edge.first)));
    }
  }

  // iterate through edges, storing vertices in clusters
  while (!(edges.empty()))
  {
    int fromIndex = -1; // cluster index where the from vertex is located
    int toIndex = -1;   // cluster index where the to vertex is located

    int distance = edges.top().first;             // cost of edge
    string fromVertex = edges.top().second.first; // from vertex
    string toVertex = edges.top().second.second;  // tto vertex
    edges.pop();

    // search through clusters to see if/where vertices exist within them
    for (int i = 0; i < minimumSpanningTree.size(); i++)
    {
      if (minimumSpanningTree[i].second.second.count(fromVertex))
      {
        fromIndex = i;
      }

      if (minimumSpanningTree[i].second.second.count(toVertex))
      {
        toIndex = i;
      }
    }

    // if both vertices are not added in the clusters
    if (fromIndex == -1 && toIndex == -1)
    {
      // create set and add vertices
      set<string> connectedVertices;
      connectedVertices.insert(fromVertex);
      connectedVertices.insert(toVertex);

      // create set of connected clusters
      set<int> otherConnectedVertices;

      // create and new cluster to vector of clusters
      minimumSpanningTree.push_back(make_pair(distance, make_pair(otherConnectedVertices, connectedVertices)));

      visit(fromVertex, toVertex, distance);
    }
    // if one of the vertices are added one of the clusters
    else if (fromIndex == -1 && toIndex != -1)
    {
      // insert vertex into cluster and add distance
      minimumSpanningTree[toIndex].second.second.insert(fromVertex);
      minimumSpanningTree[toIndex].first = minimumSpanningTree[toIndex].first + distance;

      visit(fromVertex, toVertex, distance);
    }
    else if (fromIndex != -1 && toIndex == -1)
    {
      // insert vertex into cluster and add distance
      minimumSpanningTree[fromIndex].second.second.insert(toVertex);
      minimumSpanningTree[fromIndex].first = minimumSpanningTree[fromIndex].first + distance;

      visit(fromVertex, toVertex, distance);
    }
    // if both vertices are both in different clusters that are not connected to one another
    else if (fromIndex != -1 && toIndex != -1 && fromIndex != toIndex && !(minimumSpanningTree[fromIndex].second.first.count(toIndex)))
    {
      // connect clusters
      minimumSpanningTree[fromIndex].second.first.insert(toIndex);
      minimumSpanningTree[toIndex].second.first.insert(fromIndex);

      // add cost to one of the clusters
      minimumSpanningTree[fromIndex].first = minimumSpanningTree[fromIndex].first + distance;

      visit(fromVertex, toVertex, distance);
    }
  }

  // find index of cluster containing starting vertex
  for (int i = 0; i < minimumSpanningTree.size(); i++)
  {
    if (minimumSpanningTree[i].second.second.count(startLabel))
    {
      startVertexConnection = i;
    }
  }

  // add cluster containing starting vertex to explore
  minimumSpanningTreeTraversal.push(startVertexConnection);
  visited.insert(startVertexConnection);

  // find all clusters connected to cluster containing starting vertex
  while (!(minimumSpanningTreeTraversal.empty()))
  {
    // explore cluster
    startVertexConnection = minimumSpanningTreeTraversal.front();
    minimumSpanningTreeTraversal.pop();

    // add cost of cluster to total cost
    total = total + minimumSpanningTree[startVertexConnection].first;

    // iterate through all unexplored connected clusters
    for (int connectedVertices : minimumSpanningTree[startVertexConnection].second.first)
    {
      if (!(visited.count(connectedVertices)))
      {
        visited.insert(connectedVertices);
        minimumSpanningTreeTraversal.push(connectedVertices);
      }
    }
  }

  return total;
}

// read a text file and create the graph
bool Graph::readFile(const string &filename)
{
  ifstream myfile(filename);
  if (!myfile.is_open())
  {
    cerr << "Failed to open " << filename << endl;
    return false;
  }
  int edges = 0;
  int weight = 0;
  string fromVertex;
  string toVertex;
  myfile >> edges;
  for (int i = 0; i < edges; ++i)
  {
    myfile >> fromVertex >> toVertex >> weight;
    connect(fromVertex, toVertex, weight);
  }
  myfile.close();
  return true;
}

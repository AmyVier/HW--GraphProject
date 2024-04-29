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
  Vertex *vertexFind;

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
  Vertex *vertexFind;
  string edgeString = "";

  if (vertices.count(label))
  {
    vertexFind = vertices.at(label);
  }
  else
  {
    return "";
  }

  for (auto vertex : vertexFind->edges)
  {
    edgeString = edgeString + vertex.second->destination->value + "(" + to_string(vertex.second->distance) + "),";
  }

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

  if (from == to)
  {
    return false;
  }

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
  set<string> visited;
  stack<Vertex *> toVisit;
  Vertex *currentVertex;

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
    currentVertex = toVisit.top();
    toVisit.pop();
    visit(currentVertex->value);
    map<string, Vertex::Edge *>::reverse_iterator it;

    for (it = currentVertex->edges.rbegin(); it != currentVertex->edges.rend(); it++)
    {
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
  set<string> visited;
  queue<Vertex *> toVisit;
  Vertex *currentVertex;

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
    currentVertex = toVisit.front();
    toVisit.pop();
    visit(currentVertex->value);

    for (auto edge : currentVertex->edges)
    {
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
  map<string, int> weights;
  map<string, string> previous;
  priority_queue<pair<int, pair<string, Vertex *>>, vector<pair<int, pair<string, Vertex *>>>, greater<pair<int, pair<string, Vertex *>>>> toVisit;
  Vertex *currentVertex;
  int path = 0;
  string previousLabel = startLabel;

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
    currentVertex = toVisit.top().second.second;
    path = toVisit.top().first;
    previousLabel = toVisit.top().second.first;
    toVisit.pop();

    if (!(weights.count(currentVertex->value)) || weights.at(currentVertex->value) > path)
    {
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
      for (auto edge : currentVertex->edges)
      {
        if (!(weights.count(edge.second->destination->value)) ||
            (path + edge.second->distance < weights.at(edge.second->destination->value)))
        {
          toVisit.push(make_pair(path + edge.second->distance, make_pair(currentVertex->value, edge.second->destination)));
        }
      }
    }
  }

  weights.erase(startLabel);
  previous.erase(startLabel);

  // TODO(student) Your code here
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const
{

  return -1;
}

// minimum spanning tree using Prim's algorithm
int Graph::mstKruskal(const string &startLabel,
                      void visit(const string &from, const string &to,
                                 int weight)) const
{
  return -1;
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

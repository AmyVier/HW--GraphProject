#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <limits.h>

using namespace std;

class Graph {
private:
    unordered_map<string, unordered_map<string, int>> adjacencyList; // To store vertices and their edges with weights
    bool directionalEdges;

public:
    // Constructor
    explicit Graph(bool directionalEdges = true) : directionalEdges(directionalEdges) {}

    // Destructor
    ~Graph() {
        adjacencyList.clear();
    }

    // Function to check if a vertex exists in the graph
    bool contains(const string &label) const {
        return adjacencyList.find(label) != adjacencyList.end();
    }

    // Function to add a vertex to the graph
    bool add(const string &label) {
        if (!contains(label)) {
            adjacencyList[label] = unordered_map<string, int>();
            return true;
        }
        return false;
    }

    // Function to connect two vertices with an edge
    bool connect(const string &from, const string &to, int weight = 0) {
        if (contains(from) && contains(to) && from != to) {
            adjacencyList[from][to] = weight;
            if (!directionalEdges) {
                adjacencyList[to][from] = weight;
            }
            return true;
        }
        return false;
    }

    // Function to disconnect two vertices
    bool disconnect(const string &from, const string &to) {
        if (contains(from) && contains(to) && adjacencyList[from].find(to) != adjacencyList[from].end()) {
            adjacencyList[from].erase(to);
            if (!directionalEdges) {
                adjacencyList[to].erase(from);
            }
            return true;
        }
        return false;
    }

    // Function to get the number of vertices in the graph
    int verticesSize() const {
        return adjacencyList.size();
    }

    // Function to get the number of edges in the graph
    int edgesSize() const {
        int count = 0;
        for (const auto &entry : adjacencyList) {
            count += entry.second.size();
        }
        return count;
    }

    // Function to get the degree of a vertex
    int vertexDegree(const string &label) const {
        if (contains(label)) {
            return adjacencyList.at(label).size();
        }
        return -1;
    }

    // Function to get edges as a string for a given vertex
    string getEdgesAsString(const string &label) const {
        if (contains(label)) {
            string result;
            for (const auto &edge : adjacencyList.at(label)) {
                result += edge.first + "(" + to_string(edge.second) + "), ";
            }
            result.pop_back(); // Remove trailing comma
            result.pop_back(); // Remove space
            return result;
        }
        return "";
    }

    // Function to read edges from a file
    bool readFile(const string &filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        int numEdges;
        file >> numEdges;

        for (int i = 0; i < numEdges; ++i) {
            string from, to;
            int weight;
            file >> from >> to >> weight;

            add(from);
            add(to);
            connect(from, to, weight);
        }

        file.close();
        return true;
    }

    // Depth-first traversal
    void dfs(const string &startLabel, void visit(const string &label)) {
        unordered_set<string> visited;
        dfsHelper(startLabel, visited, visit);
    }

private:
    // Helper function for depth-first traversal
    void dfsHelper(const string &label, unordered_set<string> &visited, void visit(const string &label)) {
        if (visited.find(label) != visited.end()) {
            return;
        }

        visited.insert(label);
        visit(label);

        for (const auto &neighbor : adjacencyList[label]) {
            dfsHelper(neighbor.first, visited, visit);
        }
    }

public:
    // Breadth-first traversal
    void bfs(const string &startLabel, void visit(const string &label)) {
        unordered_set<string> visited;
        queue<string> bfsQueue;

        bfsQueue.push(startLabel);
        visited.insert(startLabel);

        while (!bfsQueue.empty()) {
            string currentLabel = bfsQueue.front();
            bfsQueue.pop();
            visit(currentLabel);

            for (const auto &neighbor : adjacencyList[currentLabel]) {
                if (visited.find(neighbor.first) == visited.end()) {
                    bfsQueue.push(neighbor.first);
                    visited.insert(neighbor.first);
                }
            }
        }
    }

    // Dijkstra's algorithm
    pair<unordered_map<string, int>, unordered_map<string, string>> dijkstra(const string &startLabel) const {
        unordered_map<string, int> weights;
        unordered_map<string, string> previous;
        priority_queue<pair<int, string>, vector<pair<int, string>>, greater<pair<int, string>>> pq;

        for (const auto &vertex : adjacencyList) {
            weights[vertex.first] = INT_MAX;
            previous[vertex.first] = "";
        }

        weights[startLabel] = 0;
        pq.push({0, startLabel});

        while (!pq.empty()) {
            string currentLabel = pq.top().second;
            pq.pop();

            for (const auto &neighbor : adjacencyList.at(currentLabel)) {
                string neighborLabel = neighbor.first;
                int edgeWeight = neighbor.second;
                int totalWeight = weights[currentLabel] + edgeWeight;

                if (totalWeight < weights[neighborLabel]) {
                    weights[neighborLabel] = totalWeight;
                    previous[neighborLabel] = currentLabel;
                    pq.push({totalWeight, neighborLabel});
                }
            }
        }

        return {weights, previous};
    }

    // Prim's algorithm for Minimum Spanning Tree
    int mstPrim(const string &startLabel, void visit(const string &from, const string &to, int weight)) const {
        if (!contains(startLabel)) {
            return -1; // Start vertex not found
        }

        int totalWeight = 0;
        unordered_set<string> visited;
        priority_queue<pair<int, pair<string, string>>, vector<pair<int, pair<string, string>>>, greater<>> pq;

        pq.push({0, {startLabel, startLabel}});

        while (!pq.empty()) {
            string from = pq.top().second.first;
            string to = pq.top().second.second;
            int weight = pq.top().first;
            pq.pop();

            if (visited.find(to) != visited.end()) {
                continue;
            }

            visited.insert(to);
            totalWeight += weight;

            if (from != to) {
                visit(from, to, weight);
            }

            for (const auto &neighbor : adjacencyList.at(to)) {
                pq.push({neighbor.second, {to, neighbor.first}});
            }
        }

        return totalWeight;
    }
};

// Usage example:
int main() {
    Graph graph;
    graph.add("A");
    graph.add("B");
    graph.add("C");
    graph.connect("A", "B", 3);
    graph.connect("A", "C", 5);

    cout << "Vertices: " << graph.verticesSize() << endl;
    cout << "Edges: " << graph.edgesSize() << endl;

    cout << "Vertex Degree A: " << graph.vertexDegree("A") << endl;
    cout << "Vertex Degree B: " << graph.vertexDegree("B") << endl;

    cout << "Edges from A: " << graph.getEdgesAsString("A") << endl;

    graph.dfs("A", [](const string &label) {
        cout << label << " ";
    });
    cout << endl;

    graph.bfs("A", [](const string &label) {
        cout << label << " ";
    });
    cout << endl;

    auto dijkstraResult = graph.dijkstra("A");
    for (const auto &entry : dijkstraResult.first) {
        cout << "Shortest path to " << entry.first << ": " << entry.second << endl;
    }

    int mstWeight = graph.mstPrim("A", [](const string &from, const string &to, int weight) {
        cout << "Edge: " << from << " - " << to << " Weight: " << weight << endl;
    });

    cout << "MST Weight: " << mstWeight << endl;

    return 0;
}

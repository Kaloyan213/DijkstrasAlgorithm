#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <stack>

const int INF = std::numeric_limits<int>::max();

struct Edge {
    int destination;
    int weight;
};

std::vector<int> prev;
using Graph = std::vector<std::vector<Edge>>;


void addEdge(int from, int to, int weight, Graph& graph) {
    graph.at(from).push_back({ to, weight });
    graph.at(to).push_back({ from, weight });
}

auto dijkstra(int startNode, int endNode, const Graph& graph) {
    int numNodes = graph.size();
    std::vector <int> distance(numNodes, INF);
    distance.at(startNode) = 0;
    prev.assign(numNodes, -1);

    //creates a priority queue with pair for current distance and current node
    std::priority_queue<std::pair<int, int>, std::vector <std::pair<int, int>>> pq;

    pq.push({ 0, startNode });

    while (!pq.empty()) {
        int currentDistance = pq.top().first;
        int currentNode = pq.top().second;
        pq.pop();

        if (currentDistance > distance.at(currentNode)) {
            continue;
        }

        for (const auto& edge : graph.at(currentNode)) {
            int neighbour = edge.destination;
            int weight = edge.weight;

            if (distance.at(currentNode) + weight < distance.at(neighbour)) {
                distance.at(neighbour) = distance.at(currentNode) + weight;
                pq.push({ distance.at(neighbour), neighbour });
                prev.at(neighbour) = currentNode;
            }
        }
    }
    return distance;
}

void printShortestPath(int startNode, int endNode) {
    std::stack<int> path;
    int currentNode = endNode;

    while (currentNode != -1) {
        path.push(currentNode);
        currentNode = prev.at(currentNode);
    }

    std::cout << "Shortest path from " << startNode << " to " << endNode << ": ";
    while (!path.empty()) {
        std::cout << path.top();
        path.pop();
        if (!path.empty()) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
}


int main() {
    int numNodes = 9; // since we dont use push_back method for the vector 
    Graph graph(numNodes);


    // Define the graph (connections and weights)
    addEdge(0, 1, 4, graph);
    addEdge(0, 7, 8, graph);
    addEdge(1, 2, 8, graph);
    addEdge(1, 7, 11, graph);
    addEdge(2, 3, 7, graph);
    addEdge(2, 8, 2, graph);
    addEdge(2, 5, 4, graph);
    addEdge(3, 4, 9, graph);
    addEdge(3, 5, 14, graph);
    addEdge(4, 5, 10, graph);
    addEdge(5, 6, 2, graph);
    addEdge(6, 7, 1, graph);
    addEdge(6, 8, 6, graph);
    addEdge(7, 8, 7, graph);

    /*                            Node Weight
         Node 0 is connected to : ( 1,   4)    (7, 8)
         Node 1 is connected to : (0,4) (2, 8) (7, 11)
         Node 2 is connected to : (1,8) (3, 7) (8, 2) (5, 4)
         Node 3 is connected to : (2,7) (4, 9) (5, 14)
         Node 4 is connected to : (3,9) (5, 10)
         Node 5 is connected to : (2,4) (3, 14) (4, 10) (6, 2)
         Node 6 is connected to : (5,2) (7, 1) (8, 6)
         Node 7 is connected to : (0,8) (1, 11) (6, 1) (8, 7)
         Node 8 is connected to : (2,2) (6, 6) (7, 7)*/


    int startNode = 3;
    int endNode = 4;

    auto shortestDistances = dijkstra(startNode,endNode, graph);



    std::cout << "The shortest distance from " << startNode << " to " << endNode << " is " << shortestDistances.at(endNode) << std::endl;

    printShortestPath(startNode, endNode);
}

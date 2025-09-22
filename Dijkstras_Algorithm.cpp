#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <algorithm>

// it is a large value that represents "infinity" 
const int INF_VALUE = std::numeric_limits<int>::max();

// Class route planner uses dijkstra's algorithm and plans routes between locations.
class RoutePlanner {
private:
    std::vector<std::string> locationList;         // To stores all locations (nodes)
    std::vector<std::vector<int>> costMatrix;      // Stores distance/the cost of travel between each location
public:
   
    RoutePlanner(const std::vector<std::string>& inputLocations) : locationList(inputLocations) {
        int size = locationList.size();
        // Create cost matrix with INF_VALUE by default
        costMatrix.resize(size, std::vector<int>(size, INF_VALUE));
        // Distance from a location to itself is always 0
        for (int i = 0; i < size; ++i) {
            costMatrix[i][i] = 0;
        }
    }

    //Here this void function will Connect two locations as per given travel cost
    void connectCities(const std::string& fromCity, const std::string& toCity, int cost) {
        int fromIdx = indexOfLocation(fromCity);
        int toIdx = indexOfLocation(toCity);
        if (fromIdx != -1 && toIdx != -1) {
            costMatrix[fromIdx][toIdx] = cost;
            costMatrix[toIdx][fromIdx] = cost; // undirected graph (both directions)
        }
    }

    // This function will Show all connections including their costs
    void showConnections() {
        std::cout << "Travel Map Connections:\n";
        for (const auto& place : locationList) {
            std::cout << place << ": ";
            for (int j = 0; j < locationList.size(); ++j) {
                if (costMatrix[indexOfLocation(place)][j] < INF_VALUE) {
                    std::cout << locationList[j] << "(" << costMatrix[indexOfLocation(place)][j] << ") ";
                }
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }

    // Dijkstra's Algorithm: Main motive of this algorithm is to find shortest path from origin to destination( End point)
    std::pair<std::vector<std::string>, int> shortestPath(const std::string& origin, const std::string& destination) {
        std::vector<int> minDist(locationList.size(), INF_VALUE); // shortest known distance
        std::vector<int> parent(locationList.size(), -1);         // keeps track of path
        std::vector<bool> explored(locationList.size(), false);   // visited status

        int startIdx = indexOfLocation(origin);
        int destIdx = indexOfLocation(destination);
        minDist[startIdx] = 0; // distance from start to start = 0

        // Here this for loop is a Main loop which operates the algorithm
        for (int i = 0; i < locationList.size() - 1; ++i) {
            int currentNode = selectClosestUnvisited(minDist, explored);
            if (currentNode == -1) break; // no reachable nodes left
            
            explored[currentNode] = true; // mark as visited
            
            // Here it updates the distances to neighboring nodes
            for (int neighbor = 0; neighbor < locationList.size(); ++neighbor) {
                if (!explored[neighbor] && 
                    costMatrix[currentNode][neighbor] < INF_VALUE &&
                    minDist[currentNode] + costMatrix[currentNode][neighbor] < minDist[neighbor]) {
                    
                    minDist[neighbor] = minDist[currentNode] + costMatrix[currentNode][neighbor];
                    parent[neighbor] = currentNode;
                }
            }
        }

        std::vector<std::string> path;
        for (int at = destIdx; at != -1; at = parent[at]) {
            path.push_back(locationList[at]);
        }
        std::reverse(path.begin(), path.end());

        return {path, minDist[destIdx]};
    }

private:
    // Here we are trying to get index of a location by finding its name
    int indexOfLocation(const std::string& name) {
        auto it = std::find(locationList.begin(), locationList.end(), name);
        return (it != locationList.end()) ? std::distance(locationList.begin(), it) : -1;
    }

    // Selecting the unvisited node with the short distance which is known 
    int selectClosestUnvisited(const std::vector<int>& distArr, const std::vector<bool>& visitedArr) {
        int minValue = INF_VALUE;
        int minIndex = -1;
        for (int i = 0; i < locationList.size(); ++i) {
            if (!visitedArr[i] && distArr[i] < minValue) {
                minValue = distArr[i];
                minIndex = i;
            }
        }
        return minIndex;
    }
};

int main() {
    // Created a list of new and unique city names
    std::vector<std::string> locationNames = {"Boston", "Seattle", "Denver", "Miami", "Dallas", "Atlanta"};
    RoutePlanner planner(locationNames);

    // Define connections between locations (unique distances)
    planner.connectCities("Boston", "Denver", 1950);
    planner.connectCities("Boston", "Miami", 1250);
    planner.connectCities("Seattle", "Denver", 1300);
    planner.connectCities("Seattle", "Dallas", 2100);
    planner.connectCities("Denver", "Miami", 1720);
    planner.connectCities("Denver", "Atlanta", 1400);
    planner.connectCities("Miami", "Dallas", 1100);
    planner.connectCities("Dallas", "Atlanta", 780);
    planner.connectCities("Seattle", "Boston", 2480);

    // Show all travel connections
    planner.showConnections();

    // Here user should give a input to find shortest path query
    std::string origin, destination;
    std::cout << "Enter start location: ";
    std::cin >> origin;
    std::cout << "Enter destination location: ";
    std::cin >> destination;

    // Here finally we are running Dijkstra’s algorithm
    auto [path, cost] = planner.shortestPath(origin, destination);

   
    std::cout << "Shortest path from " << origin << " to " << destination << ":\n";
    for (const auto& place : path) {
        std::cout << place << " -> ";
    }
    std::cout << "\nTotal travel cost: " << cost << " miles\n";

    return 0;
}

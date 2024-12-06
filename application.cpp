#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue>  // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

double INF = numeric_limits<double>::max();

void buildGraph(istream& input, graph<long long, double>& g,
                vector<BuildingInfo>& buildings) {
  // TODO_STUDENT
  json j;
  input >> j;

  unordered_map<long long, Coordinates> coords;
  
  for (const auto& b : j["buildings"]) {
    Coordinates c{b["lat"].get<double>(), b["lon"].get<double>()};
    BuildingInfo bi;
    bi.id = b["id"].get<long long>();
    bi.name = b["name"].get<string>();
    bi.abbr = b["abbr"].get<string>();
    bi.location = c;
    buildings.push_back(bi);
    g.addVertex(bi.id);
  }
  for (const auto& n : j["waypoints"]) {
    long long waypoint_id = n["id"].get<long long>();
    Coordinates c{n["lat"].get<double>(), n["lon"].get<double>()};
    coords.emplace(waypoint_id, c);
    g.addVertex(waypoint_id);
  }

  for (const auto& path : j["footways"]) {
    auto vec_path = path.get<std::vector<long long>>();
    for (size_t i = 0; i + 1 < vec_path.size(); i++) {
      long long from_id = vec_path[i];
      long long to_id = vec_path[i + 1];

      Coordinates from = coords.at(from_id);
      Coordinates to = coords.at(to_id);
      double dist = distBetween2Points(from, to);

      g.addEdge(from_id, to_id, dist);
      g.addEdge(to_id, from_id, dist);
    }
  }
  for (const auto& b : buildings) {
    double minDist = INF;
    long long closest = 0;
    for (const auto& [n_id, target] : coords) {
      double dist = distBetween2Points(b.location, target);
      if (dist < minDist) {
        closest = n_id;
        minDist = dist;
      }
      if (dist > 0 && dist < 0.036) {
        g.addEdge(b.id, n_id, dist);
        g.addEdge(n_id, b.id, dist);
      }
    }
  }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo>& buildings,
                             const string& query) {
  for (const BuildingInfo& building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo>& buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo& building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

class prioritize {
   public:
    bool operator()(const pair<long long, double>& p1,
                    const pair<long long, double>& p2) const {
        return p1.second > p2.second;
    }
};


vector<long long> dijkstra(const graph<long long, double>& G, long long start,
                           long long target,
                           const set<long long>& ignoreNodes) {
    if (start == target) {
        return {start};
    }

    priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<pair<double, long long>>> worklist;
    unordered_map<long long, double> distances;
    unordered_map<long long, long long> predecessors;
    unordered_set<long long> visited;

    distances[start] = 0;
    worklist.emplace(0, start);

    while (!worklist.empty()) {
        long long current = worklist.top().second;
        double distToCurr = worklist.top().first;
        worklist.pop();

        if (visited.count(current)) continue;
        visited.insert(current);

        if (current == target) break;

        for (const auto& neighbor : G.neighbors(current)) {
            if (ignoreNodes.count(neighbor) && neighbor != target) continue;

            double edgeWeight = 0.0;
            if (!G.getWeight(current, neighbor, edgeWeight)) continue;

            double candidateDist = distToCurr + edgeWeight;
            if (distances.count(neighbor) == 0 || candidateDist < distances[neighbor]) {
                distances[neighbor] = candidateDist;
                predecessors[neighbor] = current;
                worklist.emplace(candidateDist, neighbor);
            }
        }
    }

    vector<long long> path;
    if (predecessors.count(target) == 0) {
        return {}; 
    }

    long long current = target;
    while (current != start) {
        path.push_back(current);
        current = predecessors[current];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());

    return path;
}

double pathLength(const graph<long long, double>& G,
                  const vector<long long>& path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

void application(const vector<BuildingInfo>& buildings,
                 const graph<long long, double>& G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto& building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
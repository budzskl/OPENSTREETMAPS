# Navigation Graph Application

This project implements a navigation system that helps find optimal meeting points between two people on a map using graph algorithms. The system uses building locations, waypoints, and footpaths to construct a weighted graph and find the shortest paths.

## Core Components

### Graph Implementation (`graph.h`)
- Template-based directed graph class using adjacency lists
- Supports adding vertices and edges, retrieving weights, and finding neighbors
- Provides basic graph operations like getting vertex count and edge count

### Distance Calculations (`dist.h`, `dist.cpp`)
- Implements geographical distance calculations between coordinates
- Features:
  - `distBetween2Points`: Calculates distance in miles between two points using lat/long coordinates
  - `centerBetween2Points`: Finds the center point between two coordinates

### Building Information (`BuildingInfo` struct)
- Stores building data including:
  - Unique ID
  - Location coordinates (latitude/longitude)
  - Full name
  - Abbreviation

### Main Application (`application.cpp`, `application.h`)
- Provides the core navigation functionality:
  - Graph construction from JSON input
  - Building lookup by name or abbreviation
  - Shortest path calculation using Dijkstra's algorithm
  - Interactive command-line interface for navigation requests

## Key Features

1. **Graph Construction**
   - Builds a navigation graph from JSON input containing buildings, waypoints, and footways
   - Automatically connects buildings to nearby waypoints
   - Creates bidirectional edges with distances as weights

2. **Path Finding**
   - Implements Dijkstra's algorithm for finding shortest paths
   - Supports avoiding specified nodes (except start/end points)
   - Calculates total path distances

3. **Meeting Point Selection**
   - Finds optimal meeting points between two locations
   - Calculates center point between two buildings
   - Identifies the closest building to the center point as the meeting location

## Usage

The application provides an interactive interface where users can:
1. Enter the first person's building (using partial name or abbreviation)
2. Enter the second person's building
3. View the optimal meeting point and paths for both people
4. See the total distance each person needs to travel

The program will display:
- Building information for both starting points
- The selected destination building
- Distance and path details for each person
- Path visualization using node IDs

## Technical Notes

- Uses JSON for input data parsing
- Implements custom distance calculations for geographical coordinates
- Supports both full building names and abbreviations for searches
- Creates edges between buildings and nearby waypoints (within 0.036 units)
- Provides bidirectional navigation support

## Dependencies

- C++ Standard Library
- nlohmann::json for JSON parsing
- Standard C++ map/set containers for graph implementation

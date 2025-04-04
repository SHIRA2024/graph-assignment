# Graph Algorithms Project

This project implements several graph algorithms in C++.  
It includes a `Graph` class, an `Algorithms` class, a `Queue` class, and unit tests using the **doctest** framework.

## Project Structure

- `main.cpp` – Demonstrates the functionality of all algorithms with a sample graph (edges are printed twice because the graph is undirected).  
- `Graph.hpp / Graph.cpp` – Implements the Graph class (adjacency list, add edge, etc.).  
- `Algorithms.hpp / Algorithms.cpp` – Implements algorithms:  
  - **BFS** – Breadth-First Search  
  - **DFS** – Depth-First Search  
  - **Dijkstra** – Shortest path from source  
  - **Prim** – Minimum spanning tree  
  - **Kruskal** – Minimum spanning tree  
- `Queue.hpp / Queue.cpp` – Implements a basic queue (used in BFS).  
- `test_algorithms.cpp / test_graph.cpp / test_queue.cpp` – Unit tests for the project.  
- `doctest.h` – The testing framework used.  
- `Makefile` – Automates building and testing the project.

## Requirements

Make sure the following are installed before running:

- `g++` (with C++11 support)
- `valgrind` (for memory checks)
- `make` (for running the provided Makefile)

You can install them on Ubuntu using:
```bash
sudo apt update
sudo apt install g++ valgrind make
```

## How to Run

### 1. Build and Run the Demo
```bash
make Main
```

### 2. Run Unit Tests
```bash
make test
```

### 3. Check for Memory Leaks (requires Valgrind)
```bash
make valgrind
```

### 4. Clean All Generated Files
```bash
make clean
```

## Notes

- Only `test_queue.cpp` contains the line:  
  `#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN`  
  to avoid multiple definitions of `main`.
- `prim` and `kruskal` will throw a `std::runtime_error` if the input graph is not connected.


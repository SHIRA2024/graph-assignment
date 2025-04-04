// Email: shiraba01@gmail.com

#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include "Graph.hpp" 

namespace graph {

class Algorithms {
public:
    static Graph bfs(const Graph& g, int start);
    static Graph dfs(const Graph& g, int start);
    static void dfs_visit(const Graph& g, int u, int* color, int* parent, Graph& tree);
    static Graph dijkstra(const Graph& g, int start);
    static bool isConnected(const Graph& g);
    static void dfs_helper(const Graph& g, int u, bool* visited);
    static Graph prim(const Graph& g);
    static Graph kruskal(const Graph& g);
};

} 

#endif 

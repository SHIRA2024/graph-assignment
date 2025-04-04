// Email: shiraba01@gmail.com

#include "Graph.hpp"
#include "Algorithms.hpp"
#include <iostream>

using namespace graph;
using namespace std;


/**
 * Main function that demonstrates the usage of the Graph and Algorithms classes.
 * It creates a graph with 5 vertices and runs several graph algorithms:
 * BFS, DFS, Dijkstra, Prim, and Kruskal.
 */
int main() {

    // Create a graph with 5 vertices
    Graph g(5);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 1);
    g.addEdge(1, 3, 7);
    g.addEdge(2, 4, 3);

    // Print the original graph.
    //  Note: each undirected edge appears twice in the output (once for each direction)
    cout << "Original Graph:\n";
    g.print_graph(); 

    // Run Breadth-First Search (BFS) from node 0 and print the resulting tree
    Graph bfsTree = Algorithms::bfs(g, 0);
    cout << "\nBFS Tree from node 0:\n";
    bfsTree.print_graph();

    // Run Depth-First Search (DFS) from node 0 and print the resulting tree
    Graph dfsTree = Algorithms::dfs(g, 0);
    cout << "\nDFS Tree from node 0:\n";
    dfsTree.print_graph();

    // Run Dijkstra's algorithm from node 0 and print the resulting shortest-path tree
    Graph dijkstraTree = Algorithms::dijkstra(g, 0);
    cout << "\nDijkstra Tree from node 0:\n";
    dijkstraTree.print_graph();


    // Run Prim's algorithm to find the Minimum Spanning Tree (MST) and print it
    // Note: each undirected edge appears twice in the output (once for each direction)
    //if the graph is not connected  the function catches exception and prints appropriate error message
        try {
            Graph primTree = Algorithms::prim(g);
            cout << "\nMinimum Spanning Tree using Prim:\n";
            primTree.print_graph(); 
        } catch (const std::runtime_error& e) {
            cout << "\nPrim Error: " << e.what() << endl;
        }
    
    //  Run Kruskal's algorithm to find the MST and print it
    // Note: each undirected edge appears twice in the output (once for each direction)
    //if the graph is not connected  the function catches exception and prints appropriate error message
        try {
            Graph kruskalTree = Algorithms::kruskal(g);
            cout << "\nMinimum Spanning Tree using Kruskal:\n";
            kruskalTree.print_graph(); 
        } catch (const std::runtime_error& e) {
            cout << "\nKruskal Error: " << e.what() << endl;
        }

        return 0;
    
}

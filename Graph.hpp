// Email: shiraba01@gmail.com

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <stdexcept>

namespace graph {

class Graph {
private:
    int num_vertices;           
    int** adjacency_list; 
    int** weights; 
    int* neighbor_counts;    

public:
   
    Graph(int vertices);
    ~Graph();

    
    void addEdge(int src, int dest, int weight = 1); // add an undirected edge

    void addDirectedEdge(int src, int dest, int weight = 1); // add a directed edge

    
    void removeEdge(int src, int dest);

    
    void print_graph() const;

    int get_num_vertices() const;

    int get_neighbor_count(int vertex) const;

    int get_neighbor(int vertex, int index) const;

    int* getNeighbors(int vertex) const;

    int get_weight(int u, int v) const;


};

} 

#endif 

// Email: shiraba01@gmail.com

#include "Graph.hpp"
#include <iostream> 

namespace graph {


// Constructor: Initializes a graph with the given number of vertices.
// Creates adjacency list, weight matrix, and neighbor counters.    

Graph::Graph(int vertices) {
    if (vertices <= 0) {
        throw std::invalid_argument("Number of vertices must be positive");

    }

    num_vertices = vertices;

    
    adjacency_list = new int*[num_vertices];
    weights = new int*[num_vertices]; 
    neighbor_counts = new int[num_vertices];

    for (int i = 0; i < num_vertices; ++i) {
       
        adjacency_list[i] = new int[num_vertices];
        weights[i] = new int[num_vertices];
      
        neighbor_counts[i] = 0;

        for (int j = 0; j < num_vertices; ++j) {
            weights[i][j] = 0; //no edge between vertex i and vertex j
        }
    }
}

// Destructor: Frees all dynamically allocated memory.


Graph::~Graph() {
    
    for (int i = 0; i < num_vertices; ++i) {
        delete[] adjacency_list[i];
        delete[] weights[i];
    }

    
    delete[] adjacency_list;
    delete[] weights;
    delete[] neighbor_counts;
}

// Adds an undirected edge between src and dest with the given weight.
// If the edge already exists, does nothing.
// Throws if indices are invalid or weight is zero.

void Graph::addEdge(int src, int dest, int weight) {
   
    if (src < 0 || src >= num_vertices || dest < 0 || dest >= num_vertices) {
        throw std::invalid_argument("Invalid vertex index");
    }

    if (weight == 0) {
        throw std::invalid_argument("Edge weight must be non-zero"); // 0 means there is np no edge between vertex src and vertex dest
    }
    

   
    for (int i = 0; i < neighbor_counts[src]; ++i) {
        if (adjacency_list[src][i] == dest) {
            return; 
        }
    }

    
    adjacency_list[src][neighbor_counts[src]] = dest;
    weights[src][dest] = weight;
    neighbor_counts[src]++;

    
    adjacency_list[dest][neighbor_counts[dest]] = src;
    weights[dest][src] = weight;
    neighbor_counts[dest]++;
}

// Removes the undirected edge between src and dest, if it exists.
// Updates both adjacency lists and weight matrix.
// Throws if indices are invalid.


void Graph::removeEdge(int src, int dest) {
    if (src < 0 || src >= num_vertices || dest < 0 || dest >= num_vertices) {
        throw std::invalid_argument("Invalid vertex index");
    }

   
    bool found_src = false;
    for (int i = 0; i < neighbor_counts[src]; ++i) {
        if (adjacency_list[src][i] == dest) {
            
            for (int j = i; j < neighbor_counts[src] - 1; ++j) {
                adjacency_list[src][j] = adjacency_list[src][j + 1];
            }
            neighbor_counts[src]--;
            found_src = true;
            break;
        }
    }

   
    bool found_dest = false;
    for (int i = 0; i < neighbor_counts[dest]; ++i) {
        if (adjacency_list[dest][i] == src) {
            for (int j = i; j < neighbor_counts[dest] - 1; ++j) {
                adjacency_list[dest][j] = adjacency_list[dest][j + 1];
            }
            neighbor_counts[dest]--;
            found_dest = true;
            break;
        }
    }

   
    if (found_src && found_dest) {
        weights[src][dest] = 0; // no edge
        weights[dest][src] = 0;
    }
}

// Prints all edges of the graph.
// Note: Each undirected edge appears twice in the output (once from each direction).


void Graph::print_graph() const { // note that this function prints the edges twice in an undirected graph
    for (int i = 0; i < num_vertices; ++i) {
        for (int j = 0; j < neighbor_counts[i]; ++j) {
            int neighbor = adjacency_list[i][j];
            int weight = weights[i][neighbor];
            std::cout << i << " --(" << weight << ")--> " << neighbor << std::endl;
        }
    }
}

// Returns the number of vertices in the graph.


int Graph::get_num_vertices() const {
    return num_vertices;
}


// Returns the number of neighbors for the given vertex.
// Throws if the vertex index is invalid.

int Graph::get_neighbor_count(int vertex) const {
    if (vertex < 0 || vertex >= num_vertices) {
        throw std::invalid_argument("Invalid vertex index");
    }
    return neighbor_counts[vertex];
}


// Returns the neighbor at the specified index in the adjacency list of the given vertex.
// Throws if indices are invalid.

int Graph::get_neighbor(int vertex, int index) const {
    if (vertex < 0 || vertex >= num_vertices || index < 0 || index >= neighbor_counts[vertex]) {
        throw std::invalid_argument("Invalid vertex or neighbor index");
    }
    return adjacency_list[vertex][index];
}


// Returns a pointer to the adjacency list of the given vertex.
// Throws if the vertex index is invalid.

int* Graph::getNeighbors(int vertex) const {
    if (vertex < 0 || vertex >= num_vertices) {
        throw std::invalid_argument("Invalid vertex index");
    }
    return adjacency_list[vertex];
}


// Returns the weight of the edge between vertex u and vertex v.
// Throws if indices are invalid.

int Graph::get_weight(int u, int v) const {
    if (u < 0 || u >= num_vertices || v < 0 || v >= num_vertices) {
        throw std::invalid_argument("Invalid vertex index");
    }

    return weights[u][v];  
}

// Adds a directed edge from src to dest with the given weight.
// If the edge already exists, does nothing.
// Throws if indices are invalid.

void Graph::addDirectedEdge(int src, int dest, int weight) {
    if (src < 0 || src >= num_vertices || dest < 0 || dest >= num_vertices) {
        throw std::invalid_argument("Invalid vertex index");
    }

    // Check if the edge already exists
    for (int i = 0; i < neighbor_counts[src]; ++i) {
        if (adjacency_list[src][i] == dest) {
            return;  // Edge already exists
        }
    }

    adjacency_list[src][neighbor_counts[src]] = dest;
    weights[src][dest] = weight;
    neighbor_counts[src]++;
}


} 

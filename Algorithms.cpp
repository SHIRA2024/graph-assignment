// Email: shiraba01@gmail.com

#include "Algorithms.hpp"
#include "Graph.hpp"
#include "Queue.hpp" 
#include <stdexcept>
#include <climits> // For INT_MAX

namespace graph {

    /**
     * Breadth-First Search (BFS) algorithm.
     * @param g The input graph.
     * @param start The starting vertex.
     * @return A BFS tree starting from the given vertex.
     */

    Graph Algorithms::bfs(const Graph& g, int start) {
        int n = g.get_num_vertices();
    
        if (start < 0 || start >= n) {
            throw std::invalid_argument("Invalid start vertex");
        }
    
        
        int* color = new int[n];      // 0 = white (unvisited), 1 = gray (discovered), 2 = black (finished)
        int* distance = new int[n];   
        int* parent = new int[n];     
    
        for (int u = 0; u < n; ++u) {
            color[u] = 0;          
            distance[u] = -1;      
            parent[u] = -1;        
        }
    
        color[start] = 1;          
        distance[start] = 0;       
        parent[start] = -1;        
    
        Queue q(n);                
        q.enqueue(start);         
    
       
        Graph tree(n);  // create a new empty graph to store the BFS tree
    
        while (!q.is_empty()) {    
            int u = q.dequeue();    
    
            for (int i = 0; i < g.get_neighbor_count(u); ++i) {
                int v = g.get_neighbor(u, i); 
    
                if (color[v] == 0) {           
                    color[v] = 1;              
                    distance[v] = distance[u] + 1; 
                    parent[v] = u;              
                    q.enqueue(v);                               
                    tree.addDirectedEdge(u, v, g.get_weight(u, v)); // this edge shows how we reached v
                }
            }
    
            color[u] = 2;           
        }
    
        delete[] color;
        delete[] distance;
        delete[] parent;
    
        return tree;  
    }

    
    static int time_dfs = 0; 
    
     /**
     * Depth-First Search (DFS) algorithm.
     * @param g The input graph.
     * @param start The starting vertex.
     * @return A DFS tree starting from the given vertex.
     */

    Graph Algorithms::dfs(const Graph& g, int start) {
        int n = g.get_num_vertices();
    
        int* color = new int[n];   // 0 = white (unvisited), 1 = gray (discovered), 2 = black (finished)
        int* parent = new int[n]; 
        time_dfs = 0;
    
     
        for (int i = 0; i < n; ++i) {
            color[i] = 0;     
            parent[i] = -1;  
        }
    
        Graph tree(n);
    
       
        if (color[start] == 0) {
            dfs_visit(g, start, color, parent, tree);
        }
    
        delete[] color;
        delete[] parent;
        return tree;
    }   

     /**
     * Helper function for DFS. Visits nodes recursively.
     * @param g The input graph.
     * @param u The current vertex.
     * @param color Visit status array.
     * @param parent Parent array.
     * @param tree DFS tree to be constructed.
     */

    void Algorithms::dfs_visit(const Graph& g, int u, int* color, int* parent, Graph& tree) {
        color[u] = 1;           
        time_dfs += 1;         
       
    
        int* neighbors = g.getNeighbors(u);
        int num = g.get_neighbor_count(u);
    
        for (int i = 0; i < num; ++i) {
            int v = neighbors[i];
    
            if (color[v] == 0) {           
                parent[v] = u;             
                tree.addDirectedEdge(u, v, g.get_weight(u, v));         // Add the tree edge (discovery edge) to the DFS tree
                dfs_visit(g, v, color, parent, tree); 
            }
        }
    
        color[u] = 2;          
        time_dfs += 1;         
    }

       /**
     * Dijkstra's algorithm for shortest paths.
     * @param g The input graph.
     * @param start The source vertex.
     * @return A tree representing the shortest paths from the source.
     * @throws std::invalid_argument if any edge has negative weight.
     */

    Graph Algorithms::dijkstra(const Graph& g, int start) {
        int n = g.get_num_vertices();
        
       // Check if there is any edge with a negative weight, Dijkstra doesn't support negative weights
    for (int u = 0; u < n; ++u) {
        int* neighbors = g.getNeighbors(u);
        int num_neighbors = g.get_neighbor_count(u);
        for (int i = 0; i < num_neighbors; ++i) {
            int v = neighbors[i];
            if (g.get_weight(u, v) < 0) {
                throw std::invalid_argument("Dijkstra does not support negative edge weights");
            }
        }
    }


        int* dist = new int[n];      // d[v]
        int* parent = new int[n];    // π[v]
        bool* visited = new bool[n]; // marks if a vertex was removed from the queue
    
        for (int i = 0; i < n; ++i) {
            dist[i] = INT_MAX;      // d[v] = ∞
            parent[i] = -1;         // π[v] = NULL
            visited[i] = false;    //vertex has not been visited yet (still in the queue)
        }
    
        dist[start] = 0;            
    
       
        Graph tree(n);
    
       
        for (int j = 0; j < n; ++j) {
            int u = -1;
            int min_dist = INT_MAX;
    
            
            for (int i = 0; i < n; ++i) { // extract min
                if (!visited[i] && dist[i] < min_dist) {
                    min_dist = dist[i];
                    u = i;
                }
            }
    
           
               
            if (u == -1) {// Make sure there is a reachable vertex to process
                    break;  // no more reachable vertices
                }

            visited[u] = true;  // we extracted u from the queue
        
    
           
            int* neighbors = g.getNeighbors(u);
            int num_neighbors = g.get_neighbor_count(u);
    
           
        for (int i = 0; i < num_neighbors; ++i) {
            int v = neighbors[i];
            int weight = g.get_weight(u, v);  // weight of the edge
        
            if (weight != 0) {  // if there is an edge
                if (!visited[v] && dist[u] != INT_MAX && dist[v] > dist[u] + weight) { // relax
                    dist[v] = dist[u] + weight;   
                    parent[v] = u;                
                }
            }
        }
        
        }
        for (int v = 0; v < n; ++v) {
            if (parent[v] != -1) {
                tree.addDirectedEdge(parent[v], v, g.get_weight(parent[v], v));
            }
        }
    
        delete[] dist;
        delete[] parent;
        delete[] visited;
    
        return tree;
     

  } 

 
  /**
 * Checks if the graph is connected using DFS.
 * @param g The input graph.
 * @return True if the graph is connected, false otherwise.
 */
bool Algorithms::isConnected(const Graph& g) {
    int n = g.get_num_vertices();
    bool* visited = new bool[n];
    for (int i = 0; i < n; ++i) {
        visited[i] = false;
    }

    // Start DFS from vertex 0
    dfs_helper(g, 0, visited);

    // Check if all vertices were visited
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            delete[] visited;
            return false;
        }
    }

    delete[] visited;
    return true;
}

/**
 * DFS helper function to mark visited vertices.
 * @param g The input graph.
 * @param u Current vertex.
 * @param visited Array of visited flags.
 */
void Algorithms::dfs_helper(const Graph& g, int u, bool* visited) {
    visited[u] = true;
    int num = g.get_neighbor_count(u);
    for (int i = 0; i < num; ++i) {
        int v = g.get_neighbor(u, i);
        if (!visited[v]) {
            dfs_helper(g, v, visited);
        }
    }
}


  /**
     * Prim's algorithm for minimum spanning tree.
     * @param g The input graph.
     * @return The MST built from the graph.
     * @throws std::runtime_error if the input graph is not connected.
     */ 

     Graph Algorithms::prim(const Graph& g) {
        if (!isConnected(g)) {
            throw std::runtime_error("Prim's algorithm requires the graph to be connected.");
        }
        
        int n = g.get_num_vertices();
    
        int* key = new int[n];            // Lightest edge weight to reach each vertex
        int* parent = new int[n];         
        bool* addedToTree = new bool[n];  // Marks that the vertex was added to the tree
    
       
        for (int i = 0; i < n; ++i) {
            key[i] = INT_MAX;
            parent[i] = -1;
            addedToTree[i] = false;
        }
    
        int start = 0;       //  Choose the  vertex 0 as the starting point for building the tree
        key[start] = 0;
    
        Graph mst(n);        
    
        for (int j = 0; j < n; ++j) {
            int u = -1;
            int minKey = INT_MAX;
    
            // delete min
            for (int i = 0; i < n; ++i) {
                if (!addedToTree[i] && key[i] < minKey) {
                    minKey = key[i];
                    u = i;
                }
            }
    
            if (u != -1) {
                addedToTree[u] = true;  
    
                
                if (parent[u] != -1) {
                    mst.addEdge(parent[u], u, key[u]);
                }
    
               
                int* neighbors = g.getNeighbors(u);
                int num_neighbors = g.get_neighbor_count(u);
    
                for (int i = 0; i < num_neighbors; ++i) {
                    int v = neighbors[i];
                    int weight = g.get_weight(u, v);
    
                    if (!addedToTree[v] && weight < key[v]) {
                        parent[v] = u; 
                        key[v] = weight;
                    }
                }
            }
        }
    
        delete[] key;
        delete[] parent;
        delete[] addedToTree;
    
        return mst;
    }

 /**
     * Find function for disjoint set (used in Kruskal).
     * @param u The vertex.
     * @param parent_arr Parent array.
     * @return The root of the set.
     */

 // Find
 int find(int u, int* parent_arr) {
    while (parent_arr[u] != u) {
        u = parent_arr[u];
    }
    return u;
}

   /**
     * Union function for disjoint set (used in Kruskal).
     * @param u One vertex.
     * @param v Another vertex.
     * @param parent_arr Parent array.
     * @return True if union was successful.
     */

// Union
bool unite(int u, int v, int* parent_arr) {
    int root_u = find(u, parent_arr);
    int root_v = find(v, parent_arr);
    if (root_u != root_v) {
        parent_arr[root_v] = root_u;
        return true;
    }
    return false;
}


/**
     * Kruskal's algorithm for minimum spanning tree.
     * @param g The input graph.
     * @return The MST built from the graph.
     *  @throws std::runtime_error if the input graph is not connected.
     */
    
    Graph Algorithms::kruskal(const Graph& g) {
        if (!isConnected(g)) {
            throw std::runtime_error("Kruskal's algorithm requires the graph to be connected.");
        }
        
        int n = g.get_num_vertices();
        Graph mst(n);  
    
        //build an array of Edge struct
        struct Edge {
            int u, v, weight;
        };
    
        int edge_count = 0;
        for (int u = 0; u < n; ++u) {
            edge_count += g.get_neighbor_count(u);
        }
    
        Edge* edges = new Edge[edge_count / 2];  // every edge appears twice because it's an undirected graph
        int idx = 0;
        for (int u = 0; u < n; ++u) {
            for (int i = 0; i < g.get_neighbor_count(u); ++i) {
                int v = g.get_neighbor(u, i);
                if (u < v) { //Add each undirected edge only once by checking u < v to avoid duplicates
                    edges[idx].u = u;
                    edges[idx].v = v;
                    edges[idx].weight = g.get_weight(u, v);
                    ++idx;
                }
            }
        }
    
        //sorting the edges by weight using bubble sort
        for (int i = 0; i < idx - 1; ++i) {
            for (int j = 0; j < idx - i - 1; ++j) {
                if (edges[j].weight > edges[j + 1].weight) {
                    Edge temp = edges[j];
                    edges[j] = edges[j + 1];
                    edges[j + 1] = temp;
                }
            }
        }
    
        // put each vertex in it's own connected component
        int* parent = new int[n];
        for (int i = 0; i < n; ++i) {
            parent[i] = i;
        }
    
       
    
        // add each edge while making sure there are no circles
        for (int i = 0; i < idx; ++i) {
            int u = edges[i].u;
            int v = edges[i].v;
            int w = edges[i].weight;
    
            if (unite(u, v, parent)) {
                mst.addEdge(u, v, w);
            }
        }
    
        delete[] edges;
        delete[] parent;
        return mst;
    }
    



     

}     
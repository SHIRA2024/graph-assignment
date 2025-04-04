
// Email: shiraba01@gmail.com

#include "doctest.h"
#include "Graph.hpp"
#include "Algorithms.hpp"

using namespace graph;
TEST_CASE("BFS basic test") {
    Graph g(6);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);
    g.addEdge(2, 4);
    // node 5 is disconnected

    Graph bfs_tree = Algorithms::bfs(g, 0);

    CHECK(bfs_tree.get_neighbor_count(0) == 2); 
    CHECK(bfs_tree.get_neighbor_count(1) == 1); 
    CHECK(bfs_tree.get_neighbor_count(2) == 1); 
    CHECK(bfs_tree.get_neighbor_count(3) == 0); 
    CHECK(bfs_tree.get_neighbor_count(4) == 0); 
    CHECK(bfs_tree.get_neighbor_count(5) == 0); 
}

TEST_CASE("BFS on graph with one node") {
    Graph g(1);
    Graph bfs_tree = Algorithms::bfs(g, 0);
    CHECK(bfs_tree.get_neighbor_count(0) == 0); 
}

TEST_CASE("BFS on disconnected graph") {
    Graph g(5); 
    g.addEdge(0, 1);
    g.addEdge(1, 2);
   

    Graph tree = Algorithms::bfs(g, 0);

  
    CHECK(tree.get_neighbor_count(0) == 1); 
    CHECK(tree.get_neighbor_count(1) == 1); 
    CHECK(tree.get_neighbor_count(2) == 0); 
    CHECK(tree.get_neighbor_count(3) == 0); 
    CHECK(tree.get_neighbor_count(4) == 0); 
}
TEST_CASE("BFS - check specific neighbors") {
    Graph g(3);
    g.addEdge(0, 1);
    g.addEdge(0, 2);

    Graph bfs_tree = Algorithms::bfs(g, 0);

    CHECK(bfs_tree.get_neighbor_count(0) == 2);

  
    int a = bfs_tree.get_neighbor(0, 0);
    int b = bfs_tree.get_neighbor(0, 1);
    CHECK((a == 1 || a == 2));
    CHECK((b == 1 || b == 2));
    CHECK(a != b); 
}



TEST_CASE("DFS on simple connected graph") {
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);

    Graph tree = Algorithms::dfs(g, 0);

    CHECK(tree.get_num_vertices() == 4);
    CHECK(tree.get_neighbor_count(0) == 2); 
    CHECK(tree.get_neighbor_count(1) == 1); 
    CHECK(tree.get_neighbor_count(2) == 0); 
    CHECK(tree.get_neighbor_count(3) == 0); 
}

TEST_CASE("DFS on disconnected graph - should only reach connected part") {
    Graph g(5);
    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(3, 4); 
    Graph tree = Algorithms::dfs(g, 0); 

    CHECK(tree.get_neighbor_count(0) == 1); 
    CHECK(tree.get_neighbor_count(1) == 1); 
    CHECK(tree.get_neighbor_count(2) == 0); 
    CHECK(tree.get_neighbor_count(3) == 0); 
    CHECK(tree.get_neighbor_count(4) == 0); 
}

TEST_CASE("DFS on graph with no edges") {
    Graph g(3); 
    Graph tree = Algorithms::dfs(g, 1); 

    CHECK(tree.get_num_vertices() == 3);
    CHECK(tree.get_neighbor_count(0) == 0);
    CHECK(tree.get_neighbor_count(1) == 0);
    CHECK(tree.get_neighbor_count(2) == 0);
}
TEST_CASE("Dijkstra basic test") {
    Graph g(5);
    g.addEdge(0, 1, 4);
    g.addEdge(0, 2, 1);
    g.addEdge(2, 1, 2);
    g.addEdge(1, 3, 1);
    g.addEdge(2, 3, 5);
    g.addEdge(3, 4, 3);

    Graph tree = Algorithms::dijkstra(g, 0);

    CHECK(tree.get_neighbor_count(0) == 1); 
    CHECK(tree.get_neighbor_count(2) == 1);
    CHECK(tree.get_neighbor_count(1) == 1); 
    CHECK(tree.get_neighbor_count(3) == 1);
    CHECK(tree.get_neighbor_count(4) == 0); // 4 has no outgoing edges in shortest path tree
}

TEST_CASE("Dijkstra throws on negative weights") {
    Graph g(3);
    g.addEdge(0, 1, -1);
    g.addEdge(1, 2, 2);

    CHECK_THROWS_AS(Algorithms::dijkstra(g, 0), std::invalid_argument);
}

TEST_CASE("Dijkstra on disconnected graph") {
    Graph g(4);
    g.addEdge(0, 1, 2);
    // node 2 and 3 disconnected

    Graph tree = Algorithms::dijkstra(g, 0);

    CHECK(tree.get_neighbor_count(0) == 1); 
    CHECK(tree.get_neighbor_count(1) == 0);
    CHECK(tree.get_neighbor_count(2) == 0);
    CHECK(tree.get_neighbor_count(3) == 0);
}

TEST_CASE("Prim on connected graph") {
    Graph g(5);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 3, 6);
    g.addEdge(1, 2, 3);
    g.addEdge(1, 3, 8);
    g.addEdge(1, 4, 5);
    g.addEdge(2, 4, 7);
    g.addEdge(3, 4, 9);

    Graph mst = Algorithms::prim(g);

    CHECK(mst.get_num_vertices() == 5);

    //a spanning tree of a graph with 5 vertices must contain exactly 4 edges
    int edge_count = 0;
    for (int u = 0; u < 5; ++u) {
        edge_count += mst.get_neighbor_count(u);
    }
    CHECK(edge_count / 2 == 4); // each edge was counted twice because this is an undirected graph
}

TEST_CASE("Prim on graph with one node") {
    Graph g(1);
    Graph mst = Algorithms::prim(g);
    CHECK(mst.get_num_vertices() == 1);
    CHECK(mst.get_neighbor_count(0) == 0);
}

TEST_CASE("Prim on disconnected graph") {
    Graph g(4);
    g.addEdge(0, 1, 1);
    g.addEdge(2, 3, 2); // two components

    
    CHECK_THROWS_AS(Algorithms::prim(g), std::runtime_error);


}


TEST_CASE("Kruskal on connected graph") {
    Graph g(4);
    g.addEdge(0, 1, 1);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 2);
    g.addEdge(1, 3, 3);

    Graph mst = Algorithms::kruskal(g);

    // for 4 vertices, a spanning tree should have exactly 3 edges
    int total_edges = 0;
    for (int i = 0; i < mst.get_num_vertices(); ++i) {
        total_edges += mst.get_neighbor_count(i);
    }
    CHECK(total_edges / 2 == 3); //  each edge was counted twice because this is an undirected graph
}

TEST_CASE("Kruskal on disconnected graph") {
    Graph g(5);
    g.addEdge(0, 1, 1);
    g.addEdge(2, 3, 2); // two components


    CHECK_THROWS_AS(Algorithms::kruskal(g), std::runtime_error);

}


TEST_CASE("Kruskal on graph that is already a tree") {
    Graph g(3);
    g.addEdge(0, 1, 1);
    g.addEdge(1, 2, 1);

    Graph mst = Algorithms::kruskal(g);

    // the MST of a tree is the tree itself
    int total_edges = 0;
    for (int i = 0; i < 3; ++i) {
        total_edges += mst.get_neighbor_count(i);
    }
    CHECK(total_edges / 2 == 2);
}

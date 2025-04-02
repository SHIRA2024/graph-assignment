// Email: shiraba01@gmail.com

#include "doctest.h"
#include "Graph.hpp"

using namespace graph;

TEST_CASE("Graph constructor and get_num_vertices") {
    Graph g(5);
    CHECK(g.get_num_vertices() == 5);
}

TEST_CASE("Add and get edge weight") {
    Graph g(3);
    g.addEdge(0, 1, 4);
    CHECK(g.get_weight(0, 1) == 4);
    CHECK(g.get_weight(1, 0) == 4); // because it's undirected
}

TEST_CASE("Add edge twice doesn't duplicate") {
    Graph g(3);
    g.addEdge(0, 1, 4);
    g.addEdge(0, 1, 4); // Should not add twice
    CHECK(g.get_neighbor_count(0) == 1);
    CHECK(g.get_neighbor_count(1) == 1);
}

TEST_CASE("Remove edge") {
    Graph g(3);
    g.addEdge(0, 1, 2);
    g.removeEdge(0, 1);
    CHECK(g.get_weight(0, 1) == 0);
    CHECK(g.get_neighbor_count(0) == 0);
    CHECK(g.get_neighbor_count(1) == 0);
}

TEST_CASE("Get neighbor") {
    Graph g(4);
    g.addEdge(1, 2, 6);
    CHECK(g.get_neighbor(1, 0) == 2);
    CHECK(g.get_neighbor(2, 0) == 1);
}

TEST_CASE("Get neighbors array") {
    Graph g(4);
    g.addEdge(0, 1, 3);
    g.addEdge(0, 2, 5);
    int* neighbors = g.getNeighbors(0);
    CHECK((neighbors[0] == 1 || neighbors[0] == 2));
    CHECK((neighbors[1] == 1 || neighbors[1] == 2));
}

TEST_CASE("Invalid arguments throw exceptions") {
    Graph g(3); //creats graph with vertex 0,1,2
    CHECK_THROWS_AS(Graph(0), std::invalid_argument);
    CHECK_THROWS_AS(g.addEdge(-1, 2, 1), std::invalid_argument);
    CHECK_THROWS_AS(g.addEdge(1, 5, 1), std::invalid_argument);
    CHECK_THROWS_AS(g.addEdge(0, 1, 0), std::invalid_argument);
    CHECK_THROWS_AS(g.get_neighbor(2, 5), std::invalid_argument);
    CHECK_THROWS_AS(g.getNeighbors(4), std::invalid_argument);
    CHECK_THROWS_AS(g.get_weight(1, 6), std::invalid_argument);
}

TEST_CASE("Test addDirectedEdge") {
    Graph g(4);
    g.addDirectedEdge(0, 1);
    g.addDirectedEdge(0, 2);

    CHECK(g.get_neighbor_count(0) == 2);
    CHECK(g.get_neighbor_count(1) == 0);
    CHECK(g.get_neighbor_count(2) == 0);

    CHECK(g.get_neighbor(0, 0) == 1);
    CHECK(g.get_neighbor(0, 1) == 2);
}

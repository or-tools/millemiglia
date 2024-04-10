#include <gtest/gtest.h>
#include "../graph_utils/Vertex.h"

TEST(VertexTest, DefaultConstructor) {
  Vertex v;
  Vertex::restart_id_counter();

  EXPECT_EQ(v.get_id(), 0); 
  EXPECT_EQ(v.get_name(), "Vertex_0");
}

TEST(VertexTest, NameConstructor) {
  Vertex v("CustomName");
  Vertex::restart_id_counter();

  EXPECT_EQ(v.get_id(),0);
  EXPECT_EQ(v.get_name(), "CustomName");
}

TEST(VertexTest, AddNeighbourOut) {
  Vertex v;
  Vertex::restart_id_counter();
  v.add_neighbour_out(1, "Line1");
  
  EXPECT_EQ(v.get_adjacency_list_out().size(), 1);
  EXPECT_EQ(v.get_lines_out(1)[0], "Line1");

  v.add_neighbour_out(1, "Line2");
  
  EXPECT_EQ(v.get_adjacency_list_out().size(), 1);
  EXPECT_EQ(v.get_lines_out(1).size(), 2);
}

TEST(VertexTest, AddNeighbourIn) {
  Vertex v;
  Vertex::restart_id_counter();
  v.add_neighbour_in(2, "Line3");
  
  EXPECT_EQ(v.get_adjacency_list_in().size(), 1);
  EXPECT_EQ(v.get_lines_in(2)[0], "Line3");
}

TEST(VertexTest, IsNeighbour) {
  Vertex v;
  Vertex::restart_id_counter();

  v.add_neighbour_out(3, "Line4");
  EXPECT_TRUE(v.is_neighbour_out(3));
  EXPECT_FALSE(v.is_neighbour_in(3));
}

TEST(VertexTest, Getters) {
  Vertex v("TestVertex");
  Vertex::restart_id_counter();
  v.add_neighbour_out(4, "Line5");
  v.add_neighbour_in(5, "Line6");

  EXPECT_EQ(v.get_id(), 0); 
  EXPECT_EQ(v.get_name(), "TestVertex");
  EXPECT_EQ(v.get_neighbours_number(), 2);
}

TEST(VertexTest, GetLines) {
  Vertex v;
  Vertex::restart_id_counter();  
  v.add_neighbour_out(6, "Line7");

  EXPECT_EQ(v.get_lines_out(6)[0], "Line7");
  EXPECT_TRUE(v.get_lines_in(6).empty());
}

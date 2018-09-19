#include "../include/headers/RobotGraph.h"
#include "../include/headers/RigidObject.h"

RobotGraph::RobotGraph(unsigned int num_vertices) : num_vertices_(num_vertices)
{
//  adj_matrix.resize(num_vertices_);
//  for (auto& row : adj_matrix)
//  {
//    row.resize(num_vertices_);
//  }
}

bool RobotGraph::isEdgeExist(Robot i, Robot j) const
{
  return adj_matrix.at(i.getId()).at(j.getId());
}

unsigned int RobotGraph::getNumVertices() const
{
  return num_vertices_;
}

#ifndef PROJECT_ROBOTGRAPH_H
#define PROJECT_ROBOTGRAPH_H

#include "RigidObject.h"
#include <vector>

class RobotGraph
{
public:
  explicit RobotGraph(unsigned int num_vertices);
  bool isEdgeExist(Robot i, Robot j) const;
  unsigned int getNumVertices() const;
  std::vector<Robot> robots;

private:
  std::vector<std::vector<bool>> adj_matrix;
  unsigned int num_vertices_;
};

#endif  // PROJECT_ROBOTGRAPH_H

#ifndef RIGIDOBJECT_H
#define RIGIDOBJECT_H

#include "../Eigen/Dense"

class RigidObject
{
public:
  RigidObject();
  explicit RigidObject(Eigen::Vector2d position);
  Eigen::Vector2d getPosition() const;
  void setPosition(Eigen::Vector2d position);

private:
  Eigen::Vector2d current_position_;  // current Object position
};

class Robot : public RigidObject
{
public:
  explicit Robot(Eigen::Vector2d position);
  double getUmax() const;
  unsigned int getId() const;

private:
  unsigned int unique_id;  // unique id for all the runtime
  double u_max_;           // maximum speed of movement
};

class Obstacle : public RigidObject
{
};

#endif
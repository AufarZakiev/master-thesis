//
// Created by aufarz on 04.09.18.
//

#include "../include/headers/RigidObject.h"

RigidObject::RigidObject()
{
  current_position_ << 0, 0;
}

RigidObject::RigidObject(Eigen::Vector2d position)
{
  current_position_ = position;
}

Eigen::Vector2d RigidObject::getPosition() const
{
  return current_position_;
}

void RigidObject::setPosition(Eigen::Vector2d position)
{
  current_position_ = position;
}

Robot::Robot(Eigen::Vector2d position) : RigidObject(position)
{
}

double Robot::getUmax() const
{
  return u_max_;
}

unsigned int Robot::getId() const
{
  return unique_id;
}
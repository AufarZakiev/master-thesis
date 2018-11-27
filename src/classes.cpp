#include <ros/init.h>
#include "../include/headers/classes.h"
#include "../include/headers/Variables.h"
#include "../include/headers/helper_functions.h"
#include <tuple>
#include <utility>
#include "../include/gnuplot-iostream/gnuplot-iostream.h"

RigidObject::RigidObject()
{
  current_position_ << 0, 0;
}

RigidObject::RigidObject(Position_t position)
{
  current_position_ = std::move(position);
}

Position_t RigidObject::getPosition() const
{
  return current_position_;
}

void RigidObject::setPosition(Position_t position)
{
  current_position_ = std::move(position);
}

Obstacle::Obstacle(Position_t position)
{
  current_position_ = std::move(position);
}
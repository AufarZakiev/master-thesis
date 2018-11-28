#include "../include/headers/classes.h"

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

double Robot::getSpeed() const {
  return current_speed_;
}

void Robot::setSpeed(double speed) {
  current_speed_ = speed;
}

Obstacle::Obstacle(Position_t position)
{
  current_position_ = std::move(position);
}
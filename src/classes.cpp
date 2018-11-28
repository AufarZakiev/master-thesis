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

double Robot::getSpeedValue() const {
  return current_speed_value_;
}

void Robot::setSpeedValue(double current_speed_value_) {
  this->current_speed_value_ = current_speed_value_;
}

const Vector_t Robot::getSpeedDirection() const {
  return current_speed_direction_;
}

void Robot::setSpeedDirection(const Vector_t &current_speed_direction) {
  this->current_speed_direction_ = current_speed_direction;
}

Obstacle::Obstacle(Position_t position)
{
  current_position_ = std::move(position);
}
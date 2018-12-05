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

Robot::Robot(Position_t position, Vector_t current_speed_direction, double current_speed_value, double max_speed_value)
  : RigidObject(std::move(position))
{
  this->current_speed_direction_ = std::move(current_speed_direction);
  this->current_speed_value_ = current_speed_value;
}

double Robot::getSpeedValue() const
{
  return current_speed_value_;
}

void Robot::setSpeedValue(double current_speed_value_)
{
  this->current_speed_value_ = current_speed_value_;
}

double Robot::getMaxSpeedValue() const
{
  return this->max_speed_value_;
};

void Robot::setMaxSpeedValue(double max_speed_value)
{
  this->max_speed_value_ = max_speed_value;
};

const Vector_t Robot::getSpeedDirection() const
{
  return current_speed_direction_;
}

void Robot::setSpeedDirection(const Vector_t& current_speed_direction)
{
  this->current_speed_direction_ = current_speed_direction;
}

Obstacle::Obstacle(Position_t position)
{
  current_position_ = std::move(position);
}

double Obstacle::getRadius() const
{
  return radius_;
}

void Obstacle::setRadius(double radius)
{
  this->radius_ = radius;
}

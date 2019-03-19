#include "../include/headers/classes.h"
#include "../include/headers/geometric_functions.h"

int Robot::robots_count = 0;
int Obstacle::obstacles_count = 0;

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

Robot::Robot(Position_t position, Vector_t current_speed_direction, double max_speed_value)
  : RigidObject(std::move(position))
{
  this->current_speed_direction_ = std::move(current_speed_direction);
  this->max_speed_value_ = max_speed_value;
  this->ID = robots_count;
  robots_count++;
}

double Robot::getSpeedValue() const
{
  return getVectorLength(current_speed_direction_);
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

int Robot::getRobotID() const
{
  return ID;
}

Robot::~Robot()
{
  robots_count--;
}

Obstacle::Obstacle(Position_t position, double radius)
{
  this->radius_ = radius;
  this->current_position_ = std::move(position);
  this->ID = obstacles_count;
  obstacles_count++;
}

double Obstacle::getRadius() const
{
  return radius_;
}

void Obstacle::setRadius(double radius)
{
  this->radius_ = radius;
}

int Obstacle::getObstacleID() const
{
  return ID;
}

Obstacle::~Obstacle()
{
  obstacles_count--;
}

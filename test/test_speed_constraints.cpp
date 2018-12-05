#include <gtest/gtest.h>
#include "../include/headers/speed_constraints.h"

TEST(maximumDistanceConstraint, ShouldPass)
{
}

TEST(maximumDistanceConstraint2, ShouldPass)
{
}

TEST(interrobotAvoidanceConstraint, ShouldPass)
{
}

TEST(obstacleAvoidanceConstraint, ShouldPass)
{
}

TEST(LOSPreservationConstraint, ShouldPass)
{
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

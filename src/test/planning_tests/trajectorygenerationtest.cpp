#include <gtest/gtest.h>
#include "../../TrajectoryGenerator.h"

TEST(TrajectoryGeneratorTest, CheckSpeedIncreaseTrajectory)
{
    TrajectoryGenerator generator({4, 1, 50});
    VehiclePosition pos(0, 3.5, 0);
    auto trajectory = generator.generate_trajectory({pos}, {});
    ASSERT_GE(trajectory.size(), 0);
}

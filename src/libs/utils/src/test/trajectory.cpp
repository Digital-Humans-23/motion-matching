#include <gtest/gtest.h>

#include <crl-basic/utils/trajectory.h>

namespace crl {

TEST(Trajectory1DTest, linearInterpolationWithTwoPoints) {
    Trajectory1D trajectory;

    constexpr double p0{-1.0};
    constexpr double p1{1.0};

    trajectory.addKnot(0.0, p0);
    trajectory.addKnot(1.0, p1);

    constexpr double expected{0.0};
    constexpr double halfway{0.5};

    double output;

    trajectory.evaluate_linear(halfway, output);

    constexpr double eps{1e-9};
    EXPECT_NEAR(output, expected, eps);
}

}

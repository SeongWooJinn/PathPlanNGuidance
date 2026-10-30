// 261011 수정: 궤적 생성의 속도, 종점, 후진 및 회전 경계 회귀 테스트.
#include <gtest/gtest.h>
#include "velocity_planner.h"

TEST(ReferenceTrajectory, ConstantSpeedMatchesPositionAndKeepsEndpoint)
{
    ReferenceTraj a{}, b{};
    a.mode = b.mode = VehicleMode::BicycleMode;
    a.v = b.v = 0.2;
    b.x = 1.0;
    const auto result = resampleTimeBasedTrajectory({a, b}, 0.05);
    ASSERT_GT(result.size(), 2u);
    for (size_t i = 1; i + 1 < result.size(); ++i) {
        EXPECT_NEAR(result[i].x - result[i - 1].x, 0.01, 1e-9);
        EXPECT_NEAR(result[i].v, 0.2, 1e-9);
    }
    EXPECT_DOUBLE_EQ(result.back().x, 1.0);
    EXPECT_DOUBLE_EQ(result.back().v, 0.0);
}

TEST(ReferenceTrajectory, ReverseAndSinglePointAreValid)
{
    ReferenceTraj a{}, b{};
    a.mode = b.mode = VehicleMode::BicycleMode;
    a.v = b.v = -0.2;
    b.x = -1.0;
    const auto result = resampleTimeBasedTrajectory({a, b}, 0.05);
    EXPECT_LT(result[1].v, 0.0);
    EXPECT_LT(result[1].x, 0.0);
    EXPECT_DOUBLE_EQ(result.back().x, -1.0);
    EXPECT_EQ(resampleTimeBasedTrajectory({a}, 0.05).size(), 1u);
}

TEST(ReferenceTrajectory, SpinHasNoTranslationAndRespectsAngularBound)
{
    ReferenceTraj a{}, b{};
    a.mode = b.mode = VehicleMode::SpinMode;
    b.theta = -1.0;
    const auto result = resampleTimeBasedTrajectory({a, b}, 0.05);
    for (const auto & point : result) {
        EXPECT_DOUBLE_EQ(point.x, 0.0);
        EXPECT_DOUBLE_EQ(point.v, 0.0);
        EXPECT_LE(std::abs(point.delta), 0.3 + 1e-9);
    }
    EXPECT_NEAR(result.back().theta, -1.0, 1e-9);
}

TEST(ReferenceTrajectory, AccelerationUsesIntegratedDistance)
{
    ReferenceTraj a{}, b{};
    a.mode = b.mode = VehicleMode::BicycleMode;
    b.x = 1.0;
    b.v = 0.2;
    const auto result = resampleTimeBasedTrajectory({a, b}, 0.05);
    for (size_t i = 1; i + 1 < result.size(); ++i) {
        EXPECT_NEAR(result[i].x - result[i-1].x,
            0.5 * (result[i].v + result[i-1].v) * 0.05, 1e-9);
    }
}

// 261011 수정: 두 점 모두 정지여도 후진 기어와 속도 한계를 보존하며 유한 시간에 도달한다.
TEST(ReferenceTrajectory, RestToRestProducesBoundedReverseMotion)
{
    ReferenceTraj a{}, b{};
    a.mode = b.mode = VehicleMode::BicycleMode;
    a.gear = b.gear = 1;
    b.x = -0.1;
    const auto result = resampleTimeBasedTrajectory({a, b}, 0.05, 0.5, 0.2);
    bool moved = false;
    for (const auto & point : result) {
        EXPECT_GE(point.v, -0.2 - 1e-9);
        EXPECT_LE(point.v, 0.0);
        moved = moved || point.v < -0.01;
    }
    EXPECT_TRUE(moved);
    EXPECT_DOUBLE_EQ(result.back().x, -0.1);
    EXPECT_DOUBLE_EQ(result.back().v, 0.0);
}

// 261011 수정: 곡선에서 MPC에 전달하는 조향 feed-forward가 리샘플링 중 소실되지 않는다.
TEST(ReferenceTrajectory, CurvedPathPreservesSteeringReference)
{
    ReferenceTraj a{}, b{};
    a.mode = b.mode = VehicleMode::BicycleMode;
    a.v = b.v = 0.2;
    a.delta = b.delta = 0.4;
    b.x = 0.1;
    b.theta = 0.2;
    const auto result = resampleTimeBasedTrajectory({a, b}, 0.05);
    ASSERT_GT(result.size(), 2u);
    EXPECT_NEAR(result[1].delta, 0.4, 1e-9);
}

// 260905 추가: 실제 TF 버퍼로 MPC 좌표계 변환과 오류 처리를 검증하는 회귀 테스트.
#include <cmath>
#include <gtest/gtest.h>

#include "path_frame_utils.hpp"
// 260905 추가: quaternion 변환 결과의 yaw를 검사하는 TF 유틸리티.
#include "tf2/utils.hpp"

namespace
{

// 260905 추가: ROS 통신 없이 TF 버퍼에 변환을 직접 넣어 재현 가능한 입력을 구성한다.
class PathFrameTest : public ::testing::Test
{
protected:
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};

    geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw, int seconds = 10)
    {
        geometry_msgs::msg::PoseStamped result;
        result.header.frame_id = "odom";
        result.header.stamp.sec = seconds;
        result.pose.position.x = x;
        result.pose.position.y = y;
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, yaw);
        result.pose.orientation = tf2::toMsg(rotation);
        return result;
    }

    void setMapTransform(double x, double y, double yaw, int seconds = 10, bool is_static = true)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        transform.header.stamp.sec = seconds;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, yaw);
        transform.transform.rotation = tf2::toMsg(rotation);
        ASSERT_TRUE(buffer.setTransform(transform, "test", is_static));
    }
};

// 260905 추가: map->odom에 평행이동과 90도 회전을 동시에 넣어 x/y/yaw를 검증한다.
TEST_F(PathFrameTest, TransformsTranslationAndRotationWithoutChangingInput)
{
    const double half_pi = std::acos(-1.0) / 2.0;
    setMapTransform(2.0, -1.0, half_pi);
    const auto input = pose(1.0, 2.0, -0.2);
    const auto result = mpc_planner::transformPoseToPathFrame(input, "map", buffer, tf2::Duration::zero());
    EXPECT_NEAR(result.pose.position.x, 0.0, 1e-9);
    EXPECT_NEAR(result.pose.position.y, 0.0, 1e-9);
    EXPECT_NEAR(tf2::getYaw(result.pose.orientation), half_pi - 0.2, 1e-9);
    EXPECT_EQ(result.header.frame_id, "map");
    EXPECT_EQ(input.header.frame_id, "odom");
    EXPECT_DOUBLE_EQ(input.pose.position.x, 1.0);
}

// 260905 추가: 이미 같은 좌표계인 자세는 TF가 없어도 timestamp와 orientation을 그대로 보존한다.
TEST_F(PathFrameTest, SameFrameNeedsNoTransform)
{
    const auto input = pose(1.0, 2.0, -0.3);
    const auto result = mpc_planner::transformPoseToPathFrame(input, "odom", buffer, tf2::Duration::zero());
    EXPECT_EQ(result, input);
}

// 260905 추가: 최신 변환 대신 pose가 기록된 시각의 변환을 사용한다.
TEST_F(PathFrameTest, UsesPoseTimestampInsteadOfLatestTransform)
{
    setMapTransform(2.0, 0.0, 0.0, 10, false);
    setMapTransform(4.0, 0.0, 0.0, 20, false);
    const auto result = mpc_planner::transformPoseToPathFrame(pose(1.0, 0.0, 0.0, 15), "map", buffer, tf2::Duration::zero());
    EXPECT_NEAR(result.pose.position.x, 4.0, 1e-9);
    EXPECT_EQ(result.header.stamp.sec, 15);
}

// 260905 추가: 같은 시각의 AMCL 보정이 갱신되면 다음 제어 주기에도 새 변환이 반영된다.
TEST_F(PathFrameTest, AppliesUpdatedLocalizationTransform)
{
    setMapTransform(2.0, 0.0, 0.0);
    const auto input = pose(1.0, 0.0, 0.0);
    EXPECT_NEAR(mpc_planner::transformPoseToPathFrame(input, "map", buffer, tf2::Duration::zero()).pose.position.x, 3.0, 1e-9);
    setMapTransform(2.5, 0.0, 0.0);
    EXPECT_NEAR(mpc_planner::transformPoseToPathFrame(input, "map", buffer, tf2::Duration::zero()).pose.position.x, 3.5, 1e-9);
}

// 260905 추가: 끊긴 TF와 시각 범위 밖 TF는 잘못된 명령 대신 Nav2 TF 오류로 전달한다.
TEST_F(PathFrameTest, MissingTransformRaisesControllerTFError)
{
    EXPECT_THROW(mpc_planner::transformPoseToPathFrame(pose(0.0, 0.0, 0.0), "map", buffer, tf2::Duration::zero()), nav2_core::ControllerTFError);
}

TEST_F(PathFrameTest, ExtrapolationRaisesControllerTFError)
{
    setMapTransform(2.0, 0.0, 0.0, 10, false);
    EXPECT_THROW(mpc_planner::transformPoseToPathFrame(pose(0.0, 0.0, 0.0, 20), "map", buffer, tf2::Duration::zero()), nav2_core::ControllerTFError);
}

// 260905 추가: 누락된 좌표계 이름을 묵시적으로 map/odom으로 간주하지 않는다.
TEST_F(PathFrameTest, RejectsEmptyPoseOrTargetFrame)
{
    auto input = pose(0.0, 0.0, 0.0);
    EXPECT_THROW(mpc_planner::transformPoseToPathFrame(input, "", buffer, tf2::Duration::zero()), nav2_core::ControllerTFError);
    input.header.frame_id.clear();
    EXPECT_THROW(mpc_planner::transformPoseToPathFrame(input, "map", buffer, tf2::Duration::zero()), nav2_core::ControllerTFError);
}

// 260905 추가: Path.header.frame_id 상속은 허용하고 혼합/누락된 경로 좌표계는 거부한다.
TEST_F(PathFrameTest, ValidatesPathFrames)
{
    nav_msgs::msg::Path path;
    path.poses.push_back(pose(0.0, 0.0, 0.0));
    EXPECT_THROW(mpc_planner::validatePathFrame(path), nav2_core::InvalidPath);
    path.header.frame_id = "map";
    EXPECT_THROW(mpc_planner::validatePathFrame(path), nav2_core::InvalidPath);
    path.poses.front().header.frame_id.clear();
    EXPECT_NO_THROW(mpc_planner::validatePathFrame(path));
    path.poses.front().header.frame_id = "map";
    EXPECT_NO_THROW(mpc_planner::validatePathFrame(path));
}

}  // namespace

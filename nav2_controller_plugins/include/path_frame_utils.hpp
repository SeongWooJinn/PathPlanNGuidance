// 260905 추가: MPC 경로와 로봇 자세의 좌표계 검증 및 시각을 보존하는 TF 변환.
#ifndef NAV2_CONTROLLER_PLUGINS__PATH_FRAME_UTILS_HPP_
#define NAV2_CONTROLLER_PLUGINS__PATH_FRAME_UTILS_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"

namespace mpc_planner
{

// 260905 추가: 좌표계가 없는 경로와 서로 다른 좌표가 섞인 경로를 MPC 입력 전에 거부한다.
// 개별 pose의 frame_id가 비어 있으면 Path.header.frame_id를 상속한 것으로 처리한다.
inline void validatePathFrame(const nav_msgs::msg::Path & path)
{
    if (path.header.frame_id.empty()) {
        throw nav2_core::InvalidPath("MPC path header.frame_id is empty");
    }
    for (const auto & pose : path.poses) {
        if (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) {
            throw nav2_core::InvalidPath(
                "MPC path contains pose frame '" + pose.header.frame_id +
                "' different from path frame '" + path.header.frame_id + "'");
        }
    }
}

// 260905 추가: 현재 pose의 timestamp에서 경로 좌표계로 위치와 방향을 함께 변환한다.
// 최신 TF로 시각을 바꾸거나 frame_id 문자열만 덮어쓰지 않으며, 실패 시 제어 명령 생성을 중단한다.
inline geometry_msgs::msg::PoseStamped transformPoseToPathFrame(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & path_frame,
    const tf2_ros::Buffer & tf_buffer,
    const tf2::Duration & timeout)
{
    if (pose.header.frame_id.empty() || path_frame.empty()) {
        throw nav2_core::ControllerTFError("MPC pose or path frame_id is empty");
    }
    if (pose.header.frame_id == path_frame) {
        return pose;
    }
    try {
        return tf_buffer.transform(pose, path_frame, timeout);
    } catch (const tf2::TransformException & error) {
        throw nav2_core::ControllerTFError(
            "MPC cannot transform robot pose from '" + pose.header.frame_id +
            "' to path frame '" + path_frame + "': " + error.what());
    }
}

}  // namespace mpc_planner

#endif  // NAV2_CONTROLLER_PLUGINS__PATH_FRAME_UTILS_HPP_

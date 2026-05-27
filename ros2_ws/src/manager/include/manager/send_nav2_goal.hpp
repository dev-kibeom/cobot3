#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <cmath>

// 🚀 StatefulActionNode: 시간이 오래 걸리는 작업(비동기)에 사용하는 노드
class SendNav2Goal : public BT::StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // 생성자에서 ROS 2 Node 포인터를 주입받아 액션 클라이언트를 만듭니다.
    SendNav2Goal(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("yaw")
        };
    }

    // 1️⃣ 트리가 이 노드를 처음 밟았을 때 1번 실행 (목표 전송)
    BT::NodeStatus onStart() override
    {
        double x, y, yaw;
        if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
            RCLCPP_ERROR(node_->get_logger(), "포트에서 좌표를 읽어올 수 없습니다!");
            return BT::NodeStatus::FAILURE;
        }

        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(node_->get_logger(), "Nav2 서버가 준비되지 않았습니다. 재시도합니다.");
            return BT::NodeStatus::FAILURE;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        
        // 🚀 파이썬에서 짰던 Yaw to Quaternion (Z, W) 공식을 그대로 사용합니다.
        goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
        goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

        RCLCPP_INFO(node_->get_logger(), "🚀 Nav2 목적지 출발: [x: %.2f, y: %.2f, yaw: %.2f]", x, y, yaw);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        goal_done_ = false;
        goal_success_ = false;

        // 도착 시 실행될 콜백 함수 설정
        send_goal_options.result_callback =
            [this](const GoalHandleNav2::WrappedResult& result) {
                goal_done_ = true;
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    goal_success_ = true;
                    RCLCPP_INFO(node_->get_logger(), "✅ Nav2 목적지 도착 완료!");
                } else {
                    goal_success_ = false;
                    RCLCPP_ERROR(node_->get_logger(), "❌ Nav2 이동 실패 또는 취소됨!");
                }
            };

        // 목표를 비동기로 쏘고 즉시 RUNNING을 반환 (트리 블로킹 방지)
        action_client_->async_send_goal(goal_msg, send_goal_options);
        return BT::NodeStatus::RUNNING; 
    }

    // 2️⃣ RUNNING 상태일 때 관제탑이 계속 찌르면서 상태를 확인하는 함수
    BT::NodeStatus onRunning() override
    {
        if (goal_done_) {
            return goal_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    // 3️⃣ 누군가 이 노드를 강제 종료(취소)시켰을 때 처리
    void onHalted() override
    {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Nav2 이동 강제 취소 (Halted)");
        action_client_->async_cancel_all_goals();
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    bool goal_done_;
    bool goal_success_;
};
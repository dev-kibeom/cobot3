#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <string>

class NotifyArrival : public BT::SyncActionNode
{
public:
    // 토픽 발행을 위해 ROS 2 노드 포인터를 주입받습니다.
    NotifyArrival(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        publisher_ = node_->create_publisher<std_msgs::msg::String>("/amr/arrival_trigger", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("task_id"),
            BT::InputPort<std::string>("robot_id")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string task_id, robot_id;
        if (!getInput("task_id", task_id) || !getInput("robot_id", robot_id)) {
            RCLCPP_ERROR(node_->get_logger(), "포트 입력 오류!");
            return BT::NodeStatus::FAILURE;
        }

        // 파이썬 브릿지로 보낼 JSON 데이터 조립
        nlohmann::json payload = {
            {"robot_id", robot_id},
            {"task_id", task_id},
            {"status", "ARRIVED"}
        };

        std_msgs::msg::String msg;
        msg.data = payload.dump();
        publisher_->publish(msg); // ROS 2 토픽 쏘기!

        RCLCPP_INFO(node_->get_logger(), "🔔 도착 알림(ROS 2 토픽) 발행 완료: %s", msg.data.c_str());
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
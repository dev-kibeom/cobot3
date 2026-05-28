#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>

class BackUp : public BT::SyncActionNode
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

public:
    BackUp(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        // 바퀴를 직접 제어하기 위한 퍼블리셔 생성
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "⏪ [AMR] 충돌 회피를 위해 2초간 안전 후진합니다..." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        
        // 2초 동안 10Hz의 속도로 후진 명령을 계속 쏴줍니다.
        while (rclcpp::ok()) {
            auto current_time = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() >= 2000) {
                break;
            }
            
            geometry_msgs::msg::Twist msg;
            msg.linear.x = -0.2;  // 초당 20cm 속도로 스무스하게 후진
            msg.angular.z = 0.0;  // 회전 없이 일직선으로!
            cmd_pub_->publish(msg);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        }

        // 2초가 지나면 브레이크(정지) 명령 전송
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        cmd_pub_->publish(stop_msg);
        std::cout << "⏹️ [AMR] 후진 완료! 다음 목적지로의 회전을 준비합니다." << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};
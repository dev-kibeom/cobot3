#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <string>

using json = nlohmann::json;

class FetchTaskFromDB : public BT::SyncActionNode
{
public:
    FetchTaskFromDB(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("robot_id"),
            BT::InputPort<std::string>("server_url"),
            BT::OutputPort<std::string>("task_id"),   
            BT::OutputPort<std::string>("task_type"), 
            BT::OutputPort<double>("task_x"),
            BT::OutputPort<double>("task_y"),
            BT::OutputPort<double>("task_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string robot_id, server_url;
        if (!getInput("robot_id", robot_id) || !getInput("server_url", server_url)) {
            std::cerr << "포트 입력 오류!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::string request_url = server_url + "/" + robot_id + "/task";
        cpr::Response r = cpr::Get(cpr::Url{request_url}, cpr::Timeout{1000});

        if (r.status_code == 200) {
            try {
                auto data = json::parse(r.text);
                if (data["has_task"] == true) {
                    std::cout << "✅ [" << robot_id << "] 새 작업 수신: " << r.text << std::endl;
                    
                    // JSON 구조에 맞게 데이터 추출 (서버 응답 구조에 맞춰 키값 확인 필요)
                    setOutput("task_id", data.value("task_id", "unknown_task")); 
                    setOutput("task_type", data.value("task_type", "supply"));
                    
                    setOutput("task_x", data["goal"]["x"].get<double>());
                    setOutput("task_y", data["goal"]["y"].get<double>());
                    setOutput("task_yaw", data["goal"]["yaw"].get<double>());
                    return BT::NodeStatus::SUCCESS; 
                }
            } catch (const json::parse_error& e) {
                std::cerr << "JSON 파싱 에러: " << e.what() << std::endl;
            }
        }
        return BT::NodeStatus::FAILURE; 
    }
};
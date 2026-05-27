#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <string>

using json = nlohmann::json;

class ReportTaskCompleteToDB : public BT::SyncActionNode
{
public:
    ReportTaskCompleteToDB(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("task_id"),
            BT::InputPort<std::string>("robot_id"),
            BT::InputPort<std::string>("server_url")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string task_id, robot_id, server_url;
        if (!getInput("task_id", task_id) || !getInput("robot_id", robot_id) || !getInput("server_url", server_url)) {
            std::cerr << "포트 입력 오류!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // 파이썬 브릿지의 상태 업데이트 URL과 동일한 엔드포인트 조합
        std::string request_url = server_url + "/" + robot_id + "/status";
        
        // 상태를 IDLE로 되돌리고 완료된 task_id를 함께 전송
        json payload = {
            {"status", "IDLE"},
            {"completed_task", task_id}
        };

        cpr::Response r = cpr::Post(cpr::Url{request_url},
                                    cpr::Header{{"Content-Type", "application/json"}},
                                    cpr::Body{payload.dump()},
                                    cpr::Timeout{2000});

        if (r.status_code == 200 || r.status_code == 201) {
            std::cout << "🏁 [" << robot_id << "] DB에 작업(" << task_id << ") 완료 보고 성공!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        std::cerr << "❌ 작업 완료 보고 실패: HTTP " << r.status_code << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};
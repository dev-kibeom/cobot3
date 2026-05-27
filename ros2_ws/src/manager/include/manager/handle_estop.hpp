#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <iostream>

class HandleEmergencyStop : public BT::SyncActionNode
{
public:
    HandleEmergencyStop(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        // TODO: 실제 E-Stop 토픽 구독 로직 추가
        // 지금은 비상 상황이 아니라고 판단하고 FAILURE 반환 (다음 Sequence 진행)
        return BT::NodeStatus::FAILURE; 
    }
};
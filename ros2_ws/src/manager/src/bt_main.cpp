#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "manager/fetch_task.hpp"     // DB 확인 노드 
#include "manager/send_nav2_goal.hpp" // Nav2 이동 노드
#include "manager/report_task.hpp"
#include "manager/handle_estop.hpp"
#include "manager/notify_arrival.hpp"

#include <chrono>
#include <thread>
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // ROS 2 노드 생성 및 백그라운드 스레드 동작
    auto node = std::make_shared<rclcpp::Node>("bt_manager");
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });

    node->declare_parameter<std::string>("robot_id", "UNKNOWN_ROBOT");
    node->declare_parameter<std::string>("server_url", "http://127.0.0.1:8001/api/robots/amr");

    std::string robot_id = node->get_parameter("robot_id").as_string();
    std::string server_url = node->get_parameter("server_url").as_string();
    
    std::string register_url = server_url + "/" + robot_id + "/register";

    std::cout << "⏳ FMS 서버에 로봇(" << robot_id << ") 자동 등록 요청 중..." << std::endl;
    cpr::Response r = cpr::Post(cpr::Url{register_url}, cpr::Timeout{2000});

    if (r.status_code == 200 || r.status_code == 201) {
        std::cout << "✅ 로봇 등록 성공: " << r.text << std::endl;
    } else {
        std::cerr << "⚠️ 로봇 등록 실패: HTTP " << r.status_code << std::endl;
    }

    BT::BehaviorTreeFactory factory;

    // 1️⃣ DB 확인 노드 복구 (동기 노드)
    factory.registerNodeType<FetchTaskFromDB>("FetchTaskFromDB");

    // 2️⃣ Nav2 이동 노드 등록 (비동기 액션 노드)
    factory.registerBuilder<SendNav2Goal>("SendNav2Goal",
        [&node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<SendNav2Goal>(name, config, node);
        });
    
    // 🚀 새로 만든 실제 노드 등록
    factory.registerNodeType<ReportTaskCompleteToDB>("ReportTaskCompleteToDB");
    factory.registerNodeType<HandleEmergencyStop>("HandleEmergencyStop");
    factory.registerBuilder<NotifyArrival>("NotifyArrival",
        [&node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<NotifyArrival>(name, config, node);
        });

    // ---------------------------------------------------------
    // 🚀 2️⃣ 미구현 노드들을 임시(Dummy)로 등록하여 에러 우회
    // ---------------------------------------------------------

    factory.registerSimpleAction("HoldPosition", [](BT::TreeNode&) {
        std::cout << "[Dummy] 대기 위치 유지 중..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    });

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("manager");
    std::string xml_path = pkg_share_dir + "/config/task_tree.xml";
    
    auto tree = factory.createTreeFromFile(xml_path);
    BT::Groot2Publisher publisher(tree);

    std::cout << "🌳 BT 관제탑 가동 완료! (DB 통신 & Nav2 연결 준비 됨)" << std::endl;

    while (rclcpp::ok())
    {
        tree.tickWhileRunning();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
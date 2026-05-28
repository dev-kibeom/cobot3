#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "manager/fetch_task.hpp"     
#include "manager/send_nav2_goal.hpp" 
#include "manager/report_task.hpp"
#include "manager/handle_estop.hpp"
#include "manager/notify_arrival.hpp"

#include <chrono>
#include <thread>
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("bt_manager");
    std::thread spin_thread([&node]() { rclcpp::spin(node); });

    node->declare_parameter<std::string>("robot_id", "UNKNOWN_ROBOT");
    node->declare_parameter<std::string>("server_url", "http://127.0.0.1:8001/api/robots/amr");

    std::string robot_id = node->get_parameter("robot_id").as_string();
    std::string server_url = node->get_parameter("server_url").as_string();
    
    std::string register_url = server_url + "/" + robot_id + "/register";

    // FastAPI에 로봇 등록
    std::cout << "⏳ FMS 서버에 로봇(" << robot_id << ") 자동 등록 요청 중..." << std::endl;
    while (rclcpp::ok()) {
        cpr::Response r = cpr::Post(cpr::Url{register_url}, cpr::Timeout{2000});
        if (r.status_code == 200 || r.status_code == 201) {
            std::cout << "✅ 로봇 등록 성공: " << r.text << std::endl;
            break; // 성공하면 루프 탈출
        } else {
            std::cerr << "⚠️ 서버 연결 대기 중... (FastAPI 서버를 켜주세요)" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<FetchTaskFromDB>("FetchTaskFromDB");
    factory.registerBuilder<SendNav2Goal>("SendNav2Goal",
        [&node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<SendNav2Goal>(name, config, node);
        });
    factory.registerNodeType<ReportTaskCompleteToDB>("ReportTaskCompleteToDB");
    factory.registerNodeType<HandleEmergencyStop>("HandleEmergencyStop");
    factory.registerBuilder<NotifyArrival>("NotifyArrival",
        [&node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<NotifyArrival>(name, config, node);
        });

    factory.registerSimpleAction("HoldPosition", [](BT::TreeNode&) {
        std::cout << "[Dummy] 대기 위치 유지 중..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    });

    factory.registerSimpleAction("SimulateLoading", [](BT::TreeNode&) {
        std::cout << "\n🏗️ [AMR] 창고 도착! 자재를 적재 중입니다... (3초 대기)" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "✅ [AMR] 자재 상차 완료! 작업대로 출발합니다.\n" << std::endl;
        return BT::NodeStatus::SUCCESS;
    });
    
    factory.registerSimpleAction("BackUp", [](BT::TreeNode&) {
            std::cout << "🔄 [AMR] 후진 중... (2초 대기)" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cout << "✅ [AMR] 후진 완료!\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
    });
    
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("manager");
    std::string xml_path = pkg_share_dir + "/config/task_tree.xml";
    
    auto tree = factory.createTreeFromFile(xml_path);
    
    // 런치 파일에서 받은 ID를 XML 트리의 칠판(Blackboard)에 연동
    tree.rootBlackboard()->set<std::string>("robot_id", robot_id);
    tree.rootBlackboard()->set<std::string>("server_url", server_url);

    // 포트 동적 할당
    std::unique_ptr<BT::Groot2Publisher> publisher;
    try {
        // 1호기는 1667, 2호기는 1668 포트 사용
        int groot_port = (robot_id == "IW_HUB-02") ? 1668 : 1667;
        publisher = std::make_unique<BT::Groot2Publisher>(tree, groot_port);
    } catch (const std::exception& e) {
        std::cerr << "⚠️ Groot2 포트 바인딩 실패 (주행에는 문제 없음): " << e.what() << std::endl;
    }
    std::cout << "🌳 BT 관제탑 가동 완료!" << std::endl;

    while (rclcpp::ok()) {
        tree.tickWhileRunning();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
#include <memory>
#include <string>
#include <cmath>

// ROS2
#include <rclcpp/rclcpp.hpp>

// 메시지 타입
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32.hpp>

// MoveIt2
#include <moveit/move_group_interface/move_group_interface.hpp>

// Custom Service
#include "yolo_depth_interfaces/srv/move_to_point.hpp"

class TargetMoveServer : public rclcpp::Node
{
public:
  TargetMoveServer()
  : Node("target_move_server")
  {
    // Service 서버 생성
    service_ = this->create_service<yolo_depth_interfaces::srv::MoveToPoint>(
      "move_to_point",
      std::bind(&TargetMoveServer::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    // /target_angle_info 토픽 구독
    angle_info_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target_angle_info",
      10,
      std::bind(&TargetMoveServer::angle_info_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Service server ready: /move_to_point");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /target_angle_info");
  }

  void initialize()
  {
    try {
      // MoveGroupInterface 생성 (shared_from_this 사용을 위해 별도 함수에서 초기화)
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "manipulator"
      );

      // MoveIt 설정
      move_group_->setPlanningTime(10.0);
      move_group_->setNumPlanningAttempts(10);
      move_group_->setGoalPositionTolerance(0.01);
      move_group_->setGoalOrientationTolerance(0.1);

      RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
      RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    }
    catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "MoveIt2 initialization failed: %s. Service will still work for logging.", e.what());
      move_group_ = nullptr;
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_info_sub_;
  float latest_angle_deg_ = 0.0;  // 최근 각도값 (도 단위)

  void angle_info_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_angle_deg_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Received angle info: %.2f degrees", latest_angle_deg_);
  }

  void handle_service(
    const std::shared_ptr<yolo_depth_interfaces::srv::MoveToPoint::Request> request,
    std::shared_ptr<yolo_depth_interfaces::srv::MoveToPoint::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request: x=%.2f, y=%.2f, z=%.2f",
                request->target_position.x,
                request->target_position.y,
                request->target_position.z);

    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt2 not initialized, cannot process request");
      response->success = false;
      response->message = "MoveIt2 not initialized";
      return;
    }

    // 목표 Pose 설정
    geometry_msgs::msg::Pose target_pose;
    
    // Orientation: 엔드 이펙터가 아래를 향하도록 설정 (X축 기준 180도 회전)
    // 쿼터니언: (x, y, z, w) = (1, 0, 0, 0)
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;
    
    // Service로 받은 위치 설정
    target_pose.position.x = request->target_position.x;
    target_pose.position.y = request->target_position.y;
    target_pose.position.z = request->target_position.z;

    move_group_->setPoseTarget(target_pose);

    // 계획 생성
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    // 결과 실행
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded! Executing trajectory...");
      move_group_->execute(plan);
      
      // joint6 회전
      rotate_joint6_by_angle(latest_angle_deg_);
      
      response->success = true;
      response->message = "Successfully moved to target position and rotated joint6";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      response->success = false;
      response->message = "Planning failed";
    }
  }

  void rotate_joint6_by_angle(float angle_deg)
  {
    RCLCPP_INFO(this->get_logger(), "Rotating joint6 by %.2f degrees", angle_deg);
    
    try {
      // 현재 joint 각도 조회
      std::vector<double> current_joints = move_group_->getCurrentJointValues();
      
      if (current_joints.size() < 6)
      {
        RCLCPP_ERROR(this->get_logger(), "Joint 수가 부족합니다: %lu", current_joints.size());
        return;
      }
      
      // joint6 (마지막 관절)을 각도값으로 설정 (라디안으로 변환)
      double angle_rad = angle_deg * M_PI / 180.0;
      current_joints[5] = angle_rad;  // joint6는 인덱스 5 (0-based)
      
      move_group_->setJointValueTarget(current_joints);
      
      // 계획 생성 및 실행
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group_->plan(plan));
      
      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Joint6 rotation planning succeeded! Executing...");
        move_group_->execute(plan);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Joint6 rotation planning failed!");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error rotating joint6: %s", e.what());
    }
  }

  rclcpp::Service<yolo_depth_interfaces::srv::MoveToPoint>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TargetMoveServer>();
  
  try {
    node->initialize();  // shared_ptr 생성 후 초기화
  }
  catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(), "MoveIt2 initialization failed, but service will continue: %s", e.what());
  }
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

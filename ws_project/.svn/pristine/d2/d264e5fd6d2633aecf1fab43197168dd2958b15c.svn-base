#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <thread>
#include <chrono>
#include <csignal>
#include <string>
#include <vector>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>


// Action 인터페이스 헤더 (패키지 이름 kairos_interfaces, 액션 이름 PlcCobot)
// 주의: 생성된 헤더는 소문자로 되어 있을 수 있으므로, 실제 생성된 파일명을 확인하세요.
#include "kairos_interfaces/action/plc_cobot.hpp"

#define CAMFOCUS_X 288
#define CAMFOCUS_Y 288

#define m_per_pixel 0.0002645833

#define cal_x -0.233454
#define cal_y 0.177755

using namespace std::chrono_literals;

// 전역 종료 플래그
bool g_signal_shutdown = false;
void signalHandler(int /*signum*/) {
  RCLCPP_INFO(rclcpp::get_logger("pickplace_test_node"), "Interrupt signal received");
  g_signal_shutdown = true;
}

//==================== Helper Functions ====================//
bool planAndExecute(moveit::planning_interface::MoveGroupInterface & move_group_interface,
                    moveit::planning_interface::MoveGroupInterface::Plan & plan,
                    rclcpp::Logger logger)
{
  bool success = static_cast<bool>(move_group_interface.plan(plan));
  if (success) {
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Execution successful!");
    return true;
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
    return false;
  }
}

std::vector<moveit_msgs::msg::CollisionObject> addCollisionObjects(const moveit::planning_interface::MoveGroupInterface & move_group_interface)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::string frame_id = move_group_interface.getPlanningFrame();

  // Conveyor
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "conveyor";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.3;
    primitive.dimensions[primitive.BOX_Y] = 0.13;
    primitive.dimensions[primitive.BOX_Z] = 0.115;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.385;
    box_pose.position.y = -0.10;
    box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
  }

  // Left side
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "left_side";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.25;
    primitive.dimensions[primitive.BOX_Y] = 0.015;
    primitive.dimensions[primitive.BOX_Z] = 0.12;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.36;
    box_pose.position.y = 0.0;
    box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
  }

  // Right side
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "right_side";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.25;
    primitive.dimensions[primitive.BOX_Y] = 0.015;
    primitive.dimensions[primitive.BOX_Z] = 0.12;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.36;
    box_pose.position.y = -0.20;
    box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
  }

  // Camera frame
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "camera_frame";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.02;
    primitive.dimensions[primitive.BOX_Y] = 0.32;
    primitive.dimensions[primitive.BOX_Z] = 0.32;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.433;
    box_pose.position.y = -0.08;
    box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2 + 0.155;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
  }

  // Pick-up box
  /*{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "pick_up_box";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.04;
    primitive.dimensions[primitive.BOX_Y] = 0.02;
    primitive.dimensions[primitive.BOX_Z] = 0.06;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.275;
    box_pose.position.y = -0.1;
    box_pose.position.z = 0.15;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
  }*/

  return collision_objects;
}

void executePickAndPlace(moveit::planning_interface::MoveGroupInterface & arm_move_group_interface,
                         moveit::planning_interface::MoveGroupInterface & hand_move_group_interface,
                         moveit::planning_interface::MoveGroupInterface::Plan & plan,
                          double target_x, double target_y, double target_theta,
                         rclcpp::Logger logger)
{

  RCLCPP_INFO(logger, "Gripper open");
  hand_move_group_interface.setNamedTarget("release");
  planAndExecute(hand_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);

  RCLCPP_INFO(logger, "Moving");
  arm_move_group_interface.setNamedTarget("place");
  planAndExecute(arm_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);

  RCLCPP_INFO(logger, "Moving to pick position");
  arm_move_group_interface.setNamedTarget("layover");
  planAndExecute(arm_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);
  
  RCLCPP_INFO(logger, "Moving to place position");
  arm_move_group_interface.setNamedTarget("pick");
  planAndExecute(arm_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);
  
//Move angle to theta
  if (target_theta > 45) target_theta = (target_theta - 90) * M_PI / 180; // 목표 각도가 45도보다 크면 90도를 빼고 라디안으로 변환
  else target_theta = target_theta * M_PI / 180; // 그렇지 않으면 그대로 라디안으로 변환
  geometry_msgs::msg::Pose current_pose = arm_move_group_interface.getCurrentPose().pose; // 현재 자세 가져오기
  geometry_msgs::msg::Pose target_pose = current_pose; // 목표 자세를 현재 자세로 초기화

  std::vector<std::string> jointname = arm_move_group_interface.getJointNames(); // 관절 이름 목록 가져오기
  std::vector<double> jointstate = arm_move_group_interface.getCurrentJointValues(); // 현재 관절 값 가져오기
  int cnt = 0; // 카운터 초기화
  for (auto& elem : jointname) { // 관절 이름 목록 순회
      if (elem == "joint6output_to_joint6") jointstate[cnt] += target_theta; // joint6output_to_joint6 관절에 목표 각도 추가
      cnt++; // 카운터 증가
  }
  sensor_msgs::msg::JointState arm_target_joint;
  arm_target_joint.name = jointname; // 관절 이름 설정
  arm_target_joint.position = jointstate; // 관절 위치 설정
  arm_move_group_interface.setJointValueTarget(arm_target_joint); // 목표 관절 값 설정
  planAndExecute(arm_move_group_interface, plan, logger); // 계획 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기
//Move angle to theta end

//Move coordinate to x, y
  current_pose = arm_move_group_interface.getCurrentPose().pose; // 현재 자세 가져오기
  // float end_Y = arm_move_group_interface.getCurrentRPY()[2];
  float dx = (target_x - CAMFOCUS_X) * m_per_pixel;
  float dy = (CAMFOCUS_Y - target_y) * m_per_pixel;

  float position_x = cal_x - dy;
  float position_y = cal_y + dx;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  current_pose = arm_move_group_interface.getCurrentPose().pose;
  target_pose = current_pose;
  target_pose.position.x = position_x;
  target_pose.position.y = position_y;
  target_pose.position.z = 0.29;
  waypoints.push_back(target_pose);

  target_pose.position.z-=0.024;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.001;
  // double fraction = arm_move_group_interface.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
  arm_move_group_interface.execute(trajectory);

  std::this_thread::sleep_for(3s);
//Move coordinate to x, y end


  RCLCPP_INFO(logger, "Grasping object");
  hand_move_group_interface.setNamedTarget("hold");
  planAndExecute(hand_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);
  
  RCLCPP_INFO(logger, "Moving to place position");
  arm_move_group_interface.setNamedTarget("pickup");
  planAndExecute(arm_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);

  RCLCPP_INFO(logger, "Moving to place position");
  arm_move_group_interface.setNamedTarget("place");
  planAndExecute(arm_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);

  RCLCPP_INFO(logger, "Releasing object");
  hand_move_group_interface.setNamedTarget("release");
  planAndExecute(hand_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);

  RCLCPP_INFO(logger, "Returning to initial position");
  arm_move_group_interface.setNamedTarget("home");
  planAndExecute(arm_move_group_interface, plan, logger);
  std::this_thread::sleep_for(3s);

  RCLCPP_INFO(logger, "Pick and place cycle completed");
}

//==================== Action Server ====================//
class PickPlaceActionServer : public rclcpp::Node
{
public:
  using PlcCobot = kairos_interfaces::action::PlcCobot;
  using GoalHandlePlcCobot = rclcpp_action::ServerGoalHandle<PlcCobot>;

  PickPlaceActionServer()
  : Node("pickplace_action_server")
  {
    // 생성된 노드를 non-owning shared_ptr로 변환 (이미 외부에서 관리중)
    auto node_ptr = rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){});

    // MoveIt 인터페이스 생성 (팔과 그리퍼)
    arm_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "mycobot_arm");
    hand_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "hand");

    // Planning Scene 설정 및 충돌 객체 적용
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    auto object_names = planning_scene_interface_->getKnownObjectNames();
    if (!object_names.empty()) {
      planning_scene_interface_->removeCollisionObjects(object_names);
    }
    auto collision_objects = addCollisionObjects(*arm_move_group_interface_);
    for (auto & obj : collision_objects) {
      planning_scene_interface_->applyCollisionObject(obj);
    }

    // Action Server 생성
    action_server_ = rclcpp_action::create_server<PlcCobot>(
      this,
      "pick_place_action",
      std::bind(&PickPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PickPlace Action Server started.");
  }

private:
  rclcpp_action::Server<PlcCobot>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  // Goal 수신 콜백
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const PlcCobot::Goal> /*goal*/)
  {
    RCLCPP_INFO(this->get_logger(), "Received PlcCobot goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 취소 요청 콜백
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandlePlcCobot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request received.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Goal 수락 후 별도 스레드에서 실행
  void handle_accepted(const std::shared_ptr<GoalHandlePlcCobot> goal_handle)
  {
    std::thread{std::bind(&PickPlaceActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // 실제 Pick and Place 사이클 실행 및 결과 전송
  void execute(const std::shared_ptr<GoalHandlePlcCobot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing Pick and Place cycle...");
    const auto goal = goal_handle->get_goal();
    const auto x = goal->x;
    const auto y = goal->y;
    const auto theta = goal->theta;
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%f, y=%f, theta=%f", x, y, theta);
    executePickAndPlace(*arm_move_group_interface_, *hand_move_group_interface_, plan_, x, y, theta, this->get_logger());

    auto result = std::make_shared<PlcCobot::Result>();
    result->plc_result_action = true;
    //result->plc_feedback_action = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Cycle completed. Result sent.");
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  signal(SIGINT, signalHandler);

  // Action Server 노드 생성 후 spin
  auto server_node = std::make_shared<PickPlaceActionServer>();
  
  while (rclcpp::ok()){
    if(g_signal_shutdown){
      RCLCPP_INFO(rclcpp::get_logger("pickplace_test_node"), "Shutdown signal received, shutting down...");
      rclcpp::shutdown();
      break;
    }
    rclcpp::spin_some(server_node);
  }
  
  return 0;
}

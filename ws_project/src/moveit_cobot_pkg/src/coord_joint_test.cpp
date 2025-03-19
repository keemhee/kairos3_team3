#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <thread>
#include <chrono>
#include <csignal>
#include "kairos_interfaces/action/coord_joint.hpp"

using namespace std::chrono_literals;
using CoordJointAction = kairos_interfaces::action::CoordJoint;
using GoalHandleCoordJoint = rclcpp_action::ServerGoalHandle<CoordJointAction>;

bool g_signal_shutdown = false;
// 액션 실행 상태를 추적하는 전역 플래그
bool action_executed = false;

void signalHandler(int /*signum*/) {
    RCLCPP_INFO(rclcpp::get_logger("coordjoint_test_node"), "Interrupt signal received");
    g_signal_shutdown = true;
}

bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, 
                    moveit::planning_interface::MoveGroupInterface::Plan& plan, 
                    rclcpp::Logger logger) {
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

std::vector<moveit_msgs::msg::CollisionObject> addCollisionObjects(const moveit::planning_interface::MoveGroupInterface& move_group_interface) {
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
        box_pose.position.x = 0.383;
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

    return collision_objects;
}

// 관절 수정 및 XY 조정 실행 함수
void execute_joint_modification(
  const std::shared_ptr<GoalHandleCoordJoint>& goal_handle,
  moveit::planning_interface::MoveGroupInterface& arm_move_group_interface,
  moveit::planning_interface::MoveGroupInterface::Plan& plan,
  rclcpp::Logger logger) {
  
  auto goal = goal_handle->get_goal();
  double joint_value = goal->theta;
  double x_position = goal->x;
  double y_position = goal->y;
  
  // 피드백 메시지 초기화
  auto feedback = std::make_shared<CoordJointAction::Feedback>();
  auto result = std::make_shared<CoordJointAction::Result>();
  
  // 현재 진행 상황 업데이트
  feedback->current_joint_value = arm_move_group_interface.getCurrentState()->getVariablePosition("joint6output_to_joint6");
  goal_handle->publish_feedback(feedback);
  
  // 1. 먼저 마지막 관절 값 설정 및 실행
  RCLCPP_INFO(logger, "Setting joint6 value to %f", joint_value);
  arm_move_group_interface.setJointValueTarget("joint6output_to_joint6", joint_value);
  bool success = planAndExecute(arm_move_group_interface, plan, logger);
  
  // 작업이 취소되었는지 확인
  if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Joint modification was canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(logger, "Goal canceled");
      return;
  }
  
  if (!success) {
      // 관절 이동이 실패한 경우
      result->success = false;
      result->message = "Failed to set joint6 value";
      goal_handle->succeed(result);
      RCLCPP_ERROR(logger, "Failed to set joint6 value");
      action_executed = false;
      return;
  }
  
  // 2. 현재 말단 위치(end effector) 가져오기
  geometry_msgs::msg::PoseStamped current_pose = arm_move_group_interface.getCurrentPose();
  
  // 3. 지정된 x, y 위치로 카트시안 경로를 사용하여 이동 시도
  if (x_position != 0.0 || y_position != 0.0) {
      RCLCPP_INFO(logger, "Attempting to move to position x: %f, y: %f", x_position, y_position);
      
      // 목표 위치 설정 - 현재 z, 회전은 유지
      geometry_msgs::msg::Pose target_pose = current_pose.pose;
      
      // 유효한 값인 경우에만 좌표 업데이트
      bool position_updated = false;
      if (x_position != 0.0) {
          target_pose.position.x = x_position;
          position_updated = true;
      }
      
      if (y_position != 0.0) {
          target_pose.position.y = y_position;
          position_updated = true;
      }
      
      if (!position_updated) {
          // x,y 모두 0인 경우 건너뛰기
          RCLCPP_INFO(logger, "No position update needed");
      } else {
          // 카트시안 경로 계획 시도
          RCLCPP_INFO(logger, "Planning Cartesian path to target position");
          std::vector<geometry_msgs::msg::Pose> waypoints;
          waypoints.push_back(target_pose);
          
          moveit_msgs::msg::RobotTrajectory trajectory;
          const double jump_threshold = 0.0;
          const double eef_step = 0.01;  // 1cm 단위로 보간
          
          // 카트시안 경로 계획
          double fraction = arm_move_group_interface.computeCartesianPath(
              waypoints, eef_step, jump_threshold, trajectory);
              
          if (fraction > 0.9) {  // 90% 이상 경로가 성공적으로 계획된 경우
              RCLCPP_INFO(logger, "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);
              
              // 궤적 실행
              plan.trajectory_ = trajectory;
              arm_move_group_interface.execute(plan);
              success = true;
          } else {
              RCLCPP_WARN(logger, "Cartesian planning failed (%.2f%% achieved). Trying incremental approach.", fraction * 100.0);
              
              // 점진적 접근법 시도 - 현재 위치에서 목표 위치까지 점진적으로 이동
              double step_size = 0.05;  // 5cm 단위로 이동
              double current_x = current_pose.pose.position.x;
              double current_y = current_pose.pose.position.y;
              double target_x = (x_position != 0.0) ? x_position : current_x;
              double target_y = (y_position != 0.0) ? y_position : current_y;
              
              // 각 축의 이동 방향 및 거리 계산
              double dx = target_x - current_x;
              double dy = target_y - current_y;
              double distance = std::sqrt(dx*dx + dy*dy);
              
              if (distance > 0) {
                  int steps = std::ceil(distance / step_size);
                  dx /= steps;
                  dy /= steps;
                  
                  RCLCPP_INFO(logger, "Moving in %d steps of %.3f m", steps, step_size);
                  
                  success = true;  // 모든 단계가 성공할 때까지 성공으로 가정
                  
                  for (int i = 1; i <= steps && success; ++i) {
                      // 다음 중간 목표 계산
                      geometry_msgs::msg::Pose intermediate_pose = current_pose.pose;
                      intermediate_pose.position.x = current_x + dx * i;
                      intermediate_pose.position.y = current_y + dy * i;
                      
                      RCLCPP_INFO(logger, "Step %d/%d: Moving to x: %.3f, y: %.3f", 
                                 i, steps, intermediate_pose.position.x, intermediate_pose.position.y);
                      
                      // 목표 위치 설정 및 계획 실행
                      arm_move_group_interface.setPoseTarget(intermediate_pose);
                      success = planAndExecute(arm_move_group_interface, plan, logger);
                      
                      // 작업이 취소되었는지 확인
                      if (goal_handle->is_canceling()) {
                          result->success = false;
                          result->message = "Position movement was canceled during step " + std::to_string(i);
                          goal_handle->canceled(result);
                          RCLCPP_INFO(logger, "Goal canceled during incremental movement");
                          return;
                      }
                      
                      if (!success) {
                          RCLCPP_ERROR(logger, "Failed at step %d/%d", i, steps);
                          break;
                      }
                      
                      // 잠시 대기하여 로봇이 움직임을 완료할 시간 제공
                      std::this_thread::sleep_for(100ms);
                  }
              } else {
                  RCLCPP_INFO(logger, "Target position is same as current position");
                  success = true;
              }
          }
          
          if (!success) {
              // 모든 이동 시도가 실패한 경우
              result->success = false;
              result->message = "Joint6 set successfully but position movement failed";
              goal_handle->succeed(result);
              RCLCPP_ERROR(logger, "All position movement attempts failed");
              action_executed = false;
              return;
          }
      }
  }
  
  // 최종 피드백 전송
  feedback->current_joint_value = arm_move_group_interface.getCurrentState()->getVariablePosition("joint6output_to_joint6");
  goal_handle->publish_feedback(feedback);
  
  // 결과 설정
  result->success = true;
  result->message = "Joint modification and position movement executed successfully";
  
  // 작업 완료 플래그 설정
  action_executed = true;
  
  // 작업 완료
  goal_handle->succeed(result);
  RCLCPP_INFO(logger, "Goal succeeded with joint value %f and target position x: %f, y: %f", 
              joint_value, x_position, y_position);
}

void executePickAndPlace(moveit::planning_interface::MoveGroupInterface& arm_move_group_interface, 
    moveit::planning_interface::MoveGroupInterface& hand_move_group_interface, 
    moveit::planning_interface::MoveGroupInterface::Plan& plan, 
    rclcpp::Logger logger,
    rclcpp::Node::SharedPtr node) {

    action_executed = false;

    RCLCPP_INFO(logger, "gripper open");
    hand_move_group_interface.setNamedTarget("release");
    planAndExecute(hand_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    RCLCPP_INFO(logger, "Moving");
    arm_move_group_interface.setNamedTarget("place");
    planAndExecute(arm_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    RCLCPP_INFO(logger, "Moving");
    arm_move_group_interface.setNamedTarget("layover");
    planAndExecute(arm_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    // Pick 위치로 이동
    //RCLCPP_INFO(logger, "Moving to pick position");
    //arm_move_group_interface.setNamedTarget("pick2");
    //planAndExecute(arm_move_group_interface, plan, logger);
    //std::this_thread::sleep_for(3s);

    // 액션 서버 설정
    auto action_server = rclcpp_action::create_server<CoordJointAction>(
        node,
        "set_coord_joint", // 액션 서버 이름 변경
        // 목표 요청 콜백
        [logger](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CoordJointAction::Goal> goal) -> rclcpp_action::GoalResponse {
            RCLCPP_INFO(logger, "Received goal request with joint value %f, x: %f, y: %f", 
                      goal->theta, goal->x, goal->y);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // 취소 요청 콜백
        [logger](const std::shared_ptr<GoalHandleCoordJoint>& goal_handle) -> rclcpp_action::CancelResponse {
            RCLCPP_INFO(logger, "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        // 목표 실행 콜백
        [logger, &arm_move_group_interface, &plan](const std::shared_ptr<GoalHandleCoordJoint>& goal_handle) {
            // 이 쓰레드는 goal_handle을 처리합니다
            std::thread{
                [goal_handle, &arm_move_group_interface, &plan, logger]() {
                    execute_joint_modification(goal_handle, arm_move_group_interface, plan, logger);
                }
            }.detach();
        }
    );
    
    RCLCPP_INFO(logger, "Waiting for SetCoordJoint action request..."); // 로그 메시지 변경
    // 액션 요청이 성공적으로 처리될 때까지 대기
    while (rclcpp::ok() && !g_signal_shutdown && !action_executed) {
        rclcpp::spin_some(node); // 액션 요청 대기
        std::this_thread::sleep_for(100ms); // CPU 사용량 줄이기
    }
    
    // 액션 실행이 성공했을 때만 다음 동작으로 진행
    if (action_executed) {
        std::this_thread::sleep_for(3s);
    
        // 이후 동작들은 변경 없음...
        RCLCPP_INFO(logger, "Grasping object");
        hand_move_group_interface.setNamedTarget("hold");
        planAndExecute(hand_move_group_interface, plan, logger);
        std::this_thread::sleep_for(3s);

        //RCLCPP_INFO(logger, "Moving");
        //arm_move_group_interface.setNamedTarget("pick");
        //planAndExecute(arm_move_group_interface, plan, logger);
        //std::this_thread::sleep_for(3s);        

        //RCLCPP_INFO(logger, "Moving");
        //arm_move_group_interface.setNamedTarget("pick1");
        //planAndExecute(arm_move_group_interface, plan, logger);
        //std::this_thread::sleep_for(3s);

        RCLCPP_INFO(logger, "Moving");
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

        RCLCPP_INFO(logger, "Pick and place cycle completed");

    } else {
        // 액션 실행이 실패하거나 중단된 경우
        RCLCPP_ERROR(logger, "Action execution failed or interrupted. Returning to home position.");
        arm_move_group_interface.setNamedTarget("home");
        planAndExecute(arm_move_group_interface, plan, logger);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    signal(SIGINT, signalHandler);

    auto node = std::make_shared<rclcpp::Node>("coordjoint_test_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("coordjoint_test_node");

    auto arm_move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "mycobot_arm");
    auto hand_move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "hand");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto objectname = planning_scene_interface.getKnownObjectNames();
    planning_scene_interface.removeCollisionObjects(objectname);

    auto collision_objects = addCollisionObjects(arm_move_group_interface);

    for (auto& obj : collision_objects) {
        planning_scene_interface.applyCollisionObject(obj);
    }

    while (rclcpp::ok() && !g_signal_shutdown) {
        RCLCPP_INFO(logger, "Executing pick and place cycle");
        executePickAndPlace(arm_move_group_interface, hand_move_group_interface, plan, logger, node);
        std::this_thread::sleep_for(5s);
    }

    arm_move_group_interface.setNamedTarget("home");
    planAndExecute(arm_move_group_interface, plan, logger);
    std::this_thread::sleep_for(5s);

    rclcpp::shutdown();
    return 0;
}
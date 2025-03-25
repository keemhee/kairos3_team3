#include <rclcpp/rclcpp.hpp> // ROS 2의 C++ 라이브러리인 rclcpp를 포함
#include <moveit/move_group_interface/move_group_interface.h> // MoveIt의 MoveGroup 인터페이스를 포함하여 로봇 팔 제어 가능
#include <moveit/planning_scene_interface/planning_scene_interface.h> // MoveIt의 Planning Scene 인터페이스를 포함하여 환경 설정 가능
#include <geometry_msgs/msg/pose.hpp> // ROS 2에서 위치와 방향을 나타내는 Pose 메시지 타입 포함
#include <sensor_msgs/msg/joint_state.hpp> // 로봇의 관절 상태를 나타내는 JointState 메시지 타입 포함
#include <moveit/kinematic_constraints/utils.h> // MoveIt의 운동학적 제약 조건 유틸리티 포함
#include <thread> // 스레드 기능을 사용하기 위한 C++ 표준 라이브러리 포함
#include <chrono> // 시간 관련 기능을 사용하기 위한 C++ 표준 라이브러리 포함
#include <csignal> // 신호 처리를 위한 C 라이브러리 포함 (예: SIGINT)
#include <string> // 문자열 처리를 위한 C++ 표준 라이브러리 포함
#include <vector> // 동적 배열을 위한 C++ 표준 라이브러리 포함
#include <tuple> // 여러 값을 묶어 저장하기 위한 C++ 표준 라이브러리 포함
#include <rclcpp_action/rclcpp_action.hpp> // ROS 2의 액션 서버 및 클라이언트를 위한 라이브러리 포함
#include <cmath> // 수학 함수 (예: sin, cos)를 사용하기 위한 C++ 표준 라이브러리 포함
#include <algorithm> // 알고리즘 함수 (예: min, max)를 사용하기 위한 C++ 표준 라이브러리 포함

// Action 인터페이스 헤더 (패키지 이름 kairos_interfaces, 액션 이름 PlcCobot)
#include "kairos_interfaces/action/plc_cobot.hpp" // 사용자 정의 액션 인터페이스 PlcCobot 포함
#include <std_msgs/msg/bool.hpp> // ROS 2의 기본 Bool 메시지 타입 포함

#define CAMFOCUS_X 380 // 카메라 초점의 X 좌표를 380으로 정의
#define CAMFOCUS_Y 600 // 카메라 초점의 Y 좌표를 600으로 정의

#define m_per_pixel 0.00037665 // 픽셀당 미터 단위를 정의 (카메라 좌표 변환에 사용)

#define cal_x -0.233454 // X축 보정값 정의
#define cal_y 0.177755 // Y축 보정값 정의

using namespace std::chrono_literals; // 시간 리터럴 (예: 1s, 2ms)을 쉽게 사용하기 위해 네임스페이스 지정
using namespace std; // 표준 C++ 네임스페이스 사용

vector<tuple<float,float,float>> cube_drop_point_1; // 서울 지역의 큐브 드롭 포인트를 저장하는 벡터
vector<tuple<float,float,float>> cube_drop_point_2; // 부산 지역의 큐브 드롭 포인트를 저장하는 벡터
vector<tuple<float,float,float>> cube_drop_point_3; // 경기 지역의 큐브 드롭 포인트를 저장하는 벡터

int seoul_count = 0; // 서울 지역의 드롭 포인트 사용 횟수를 카운트
int busan_count = 0; // 부산 지역의 드롭 포인트 사용 횟수를 카운트
int gyeonggi_count = 0; // 경기 지역의 드롭 포인트 사용 횟수를 카운트

// 전역 종료 플래그
bool g_signal_shutdown = false; // 프로그램 종료 신호를 나타내는 전역 플래그
void signalHandler(int /*signum*/) { // 신호 핸들러 함수 정의 (SIGINT 처리)
  RCLCPP_INFO(rclcpp::get_logger("pickplace_test_node"), "Interrupt signal received"); // 인터럽트 신호 수신 로그 출력
  g_signal_shutdown = true; // 종료 플래그를 true로 설정
}

//==================== Helper Functions ====================//
bool planAndExecute(moveit::planning_interface::MoveGroupInterface & move_group_interface, // 계획 및 실행 헬퍼 함수 정의
                    moveit::planning_interface::MoveGroupInterface::Plan & plan, // MoveIt의 계획 객체 참조
                    rclcpp::Logger logger) // 로거 객체 참조
{
  bool success = static_cast<bool>(move_group_interface.plan(plan)); // 계획 생성 성공 여부를 확인
  if (success) { // 계획이 성공한 경우
    move_group_interface.execute(plan); // 계획을 실행
    //RCLCPP_INFO(logger, "Execution successful!"); // 실행 성공 로그 (주석 처리됨)
    return true; // true 반환
  } else { // 계획이 실패한 경우
    RCLCPP_ERROR(logger, "Planning failed!"); // 계획 실패 에러 로그 출력
    return false; // false 반환
  }
}

void initSetting(){ // 초기 설정 함수 정의
  //busan
  cube_drop_point_2.push_back({-0.200, -0.150, 0.280}); // 부산 드롭 포인트 1 추가
  cube_drop_point_2.push_back({-0.200, -0.205, 0.280}); // 부산 드롭 포인트 2 추가
  cube_drop_point_2.push_back({-0.200, -0.150, 0.310}); // 부산 드롭 포인트 3 추가
  cube_drop_point_2.push_back({-0.200, -0.200, 0.310}); // 부산 드롭 포인트 4 추가

  //seoul
  cube_drop_point_1.push_back({-0.100, -0.285, 0.282}); // 서울 드롭 포인트 1 추가
  cube_drop_point_1.push_back({-0.100, -0.231, 0.280}); // 서울 드롭 포인트 2 추가
  cube_drop_point_1.push_back({-0.100, -0.285, 0.317}); // 서울 드롭 포인트 3 추가
  cube_drop_point_1.push_back({-0.100, -0.230, 0.313}); // 서울 드롭 포인트 4 추가

  //gyeonggi
  cube_drop_point_3.push_back({0.050, -0.280, 0.281}); // 경기 드롭 포인트 1 추가
  cube_drop_point_3.push_back({0.050, -0.220, 0.280}); // 경기 드롭 포인트 2 추가
  cube_drop_point_3.push_back({0.050, -0.280, 0.313}); // 경기 드롭 포인트 3 추가
  cube_drop_point_3.push_back({0.050, -0.216, 0.313}); // 경기 드롭 포인트 4 추가
}

std::vector<moveit_msgs::msg::CollisionObject> addCollisionObjects(const moveit::planning_interface::MoveGroupInterface & move_group_interface) // 충돌 객체 추가 함수 정의
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects; // 충돌 객체 벡터 선언
  std::string frame_id = move_group_interface.getPlanningFrame(); // MoveIt의 플래닝 프레임 ID 가져오기

  // Conveyor
  {
    moveit_msgs::msg::CollisionObject collision_object; // 컨베이어 충돌 객체 생성
    collision_object.header.frame_id = frame_id; // 프레임 ID 설정
    collision_object.id = "conveyor"; // 객체 ID를 "conveyor"로 설정

    shape_msgs::msg::SolidPrimitive primitive; // 기본 도형 객체 생성
    primitive.type = primitive.BOX; // 도형 타입을 박스로 설정
    primitive.dimensions.resize(3); // 크기 배열 크기를 3으로 설정 (X, Y, Z)
    primitive.dimensions[primitive.BOX_X] = 0.3; // 박스의 X 크기 설정
    primitive.dimensions[primitive.BOX_Y] = 0.13; // 박스의 Y 크기 설정
    primitive.dimensions[primitive.BOX_Z] = 0.115; // 박스의 Z 크기 설정

    geometry_msgs::msg::Pose box_pose; // 박스의 위치와 방향 설정
    box_pose.orientation.w = 1.0; // 쿼터니언의 w 값을 1로 설정 (기본 방향)
    box_pose.position.x = 0.385; // X 위치 설정
    box_pose.position.y = -0.10; // Y 위치 설정
    box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2; // Z 위치 설정 (박스 높이의 절반)

    collision_object.primitives.push_back(primitive); // 도형을 충돌 객체에 추가
    collision_object.primitive_poses.push_back(box_pose); // 위치를 충돌 객체에 추가
    collision_object.operation = collision_object.ADD; // 객체 추가 작업 설정
    collision_objects.push_back(collision_object); // 충돌 객체 벡터에 추가
  }

  // Left side
  {
    moveit_msgs::msg::CollisionObject collision_object; // 왼쪽 벽 충돌 객체 생성
    collision_object.header.frame_id = frame_id; // 프레임 ID 설정
    collision_object.id = "left_side"; // 객체 ID를 "left_side"로 설정

    shape_msgs::msg::SolidPrimitive primitive; // 기본 도형 객체 생성
    primitive.type = primitive.BOX; // 도형 타입을 박스로 설정
    primitive.dimensions.resize(3); // 크기 배열 크기를 3으로 설정
    primitive.dimensions[primitive.BOX_X] = 0.25; // 박스의 X 크기 설정
    primitive.dimensions[primitive.BOX_Y] = 0.015; // 박스의 Y 크기 설정
    primitive.dimensions[primitive.BOX_Z] = 0.12; // 박스의 Z 크기 설정

    geometry_msgs::msg::Pose box_pose; // 박스의 위치와 방향 설정
    box_pose.orientation.w = 1.0; // 쿼터니언의 w 값을 1로 설정
    box_pose.position.x = 0.36; // X 위치 설정
    box_pose.position.y = 0.0; // Y 위치 설정
    box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2; // Z 위치 설정 (박스 높이의 절반)

    collision_object.primitives.push_back(primitive); // 도형을 충돌 객체에 추가
    collision_object.primitive_poses.push_back(box_pose); // 위치를 충돌 객체에 추가
    collision_object.operation = collision_object.ADD; // 객체 추가 작업 설정
    collision_objects.push_back(collision_object); // 충돌 객체 벡터에 추가
  }

  // Right side
  {
    moveit_msgs::msg::CollisionObject collision_object; // 오른쪽 벽 충돌 객체 생성
    collision_object.header.frame_id = frame_id; // 프레임 ID 설정
    collision_object.id = "right_side"; // 객체 ID를 "right_side"로 설정

    shape_msgs::msg::SolidPrimitive primitive; // 기본 도형 객체 생성
    primitive.type = primitive.BOX; // 도형 타입을 박스로 설정
    primitive.dimensions.resize(3); // 크기 배열 크기를 3으로 설정
    primitive.dimensions[primitive.BOX_X] = 0.25; // 박스의 X 크기 설정
    primitive.dimensions[primitive.BOX_Y] = 0.015; // 박스의 Y 크기 설정
    primitive.dimensions[primitive.BOX_Z] = 0.12; // 박스의 Z 크기 설정

    geometry_msgs::msg::Pose box_pose; // 박스의 위치와 방향 설정
    box_pose.orientation.w = 1.0; // 쿼터니언의 w 값을 1로 설정
    box_pose.position.x = 0.36; // X 위치 설정
    box_pose.position.y = -0.20; // Y 위치 설정
    box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2; // Z 위치 설정 (박스 높이의 절반)

    collision_object.primitives.push_back(primitive); // 도형을 충돌 객체에 추가
    collision_object.primitive_poses.push_back(box_pose); // 위치를 충돌 객체에 추가
    collision_object.operation = collision_object.ADD; // 객체 추가 작업 설정
    collision_objects.push_back(collision_object); // 충돌 객체 벡터에 추가
  }

  // Camera frame
  {
    moveit_msgs::msg::CollisionObject collision_object; // 카메라 프레임 충돌 객체 생성
    collision_object.header.frame_id = frame_id; // 프레임 ID 설정
    collision_object.id = "camera_frame"; // 객체 ID를 "camera_frame"로 설정

    shape_msgs::msg::SolidPrimitive primitive; // 기본 도형 객체 생성
    primitive.type = primitive.BOX; // 도형 타입을 박스로 설정
    primitive.dimensions.resize(3); // 크기 배열 크기를 3으로 설정
    primitive.dimensions[primitive.BOX_X] = 0.02; // 박스의 X 크기 설정
    primitive.dimensions[primitive.BOX_Y] = 0.32; // 박스의 Y 크기 설정
    primitive.dimensions[primitive.BOX_Z] = 0.32; // 박스의 Z 크기 설정

    geometry_msgs::msg::Pose box_pose; // 박스의 위치와 방향 설정
    box_pose.orientation.w = 1.0; // 쿼터니언의 w 값을 1로 설정
    box_pose.position.x = 0.433; // X 위치 설정
    box_pose.position.y = -0.08; // Y 위치 설정
    box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2 + 0.155; // Z 위치 설정 (박스 높이의 절반 + 추가 높이)

    collision_object.primitives.push_back( дрож

    collision_object.primitive_poses.push_back(box_pose); // 위치를 충돌 객체에 추가
    collision_object.operation = collision_object.ADD; // 객체 추가 작업 설정
    collision_objects.push_back(collision_object); // 충돌 객체 벡터에 추가
  }

  // Pick-up box (주석 처리된 코드)
  /*{
    moveit_msgs::msg::CollisionObject collision_object; // 픽업 박스 충돌 객체 생성
    collision_object.header.frame_id = frame_id; // 프레임 ID 설정
    collision_object.id = "pick_up_box"; // 객체 ID를 "pick_up_box"로 설정

    shape_msgs::msg::SolidPrimitive primitive; // 기본 도형 객체 생성
    primitive.type = primitive.BOX; // 도형 타입을 박스로 설정
    primitive.dimensions.resize(3); // 크기 배열 크기를 3으로 설정
    primitive.dimensions[primitive.BOX_X] = 0.04; // 박스의 X 크기 설정
    primitive.dimensions[primitive.BOX_Y] = 0.02; // 박스의 Y 크기 설정
    primitive.dimensions[primitive.BOX_Z] = 0.06; // 박스의 Z 크기 설정

    geometry_msgs::msg::Pose box_pose; // 박스의 위치와 방향 설정
    box_pose.orientation.w = 1.0; // 쿼터니언의 w 값을 1로 설정
    box_pose.position.x = 0.275; // X 위치 설정
    box_pose.position.y = -0.1; // Y 위치 설정
    box_pose.position.z = 0.15; // Z 위치 설정

    collision_object.primitives.push_back(primitive); // 도형을 충돌 객체에 추가
    collision_object.primitive_poses.push_back(box_pose); // 위치를 충돌 객체에 추가
    collision_object.operation = collision_object.ADD; // 객체 추가 작업 설정
    collision_objects.push_back(collision_object); // 충돌 객체 벡터에 추가
  }*/

  return collision_objects; // 충돌 객체 벡터 반환
}

void executePickAndPlace(moveit::planning_interface::MoveGroupInterface & arm_move_group_interface, // Pick and Place 실행 함수 정의
                         moveit::planning_interface::MoveGroupInterface & hand_move_group_interface, // 팔과 손 MoveGroup 인터페이스 참조
                         moveit::planning_interface::MoveGroupInterface::Plan & plan, // MoveIt 계획 객체 참조
                         double target_x, double target_y, double target_theta, string region, int package_id, // 목표 위치, 각도, 지역, 패키지 ID
                         rclcpp::Logger logger, const std::shared_ptr<rclcpp_action::ServerGoalHandle<kairos_interfaces::action::PlcCobot>> goal_handle) // 로거와 액션 서버 핸들
{

  auto feed_back = std::make_shared<kairos_interfaces::action::PlcCobot::Feedback>(); // 액션 피드백 객체 생성

  RCLCPP_INFO(logger, "Gripper open"); // 그리퍼 열기 로그 출력
  hand_move_group_interface.setNamedTarget("release"); // 그리퍼를 "release" 상태로 설정
  planAndExecute(hand_move_group_interface, plan, logger); // 계획 생성 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기

  // RCLCPP_INFO(logger, "Moving"); // 이동 로그 (주석 처리됨)
  // arm_move_group_interface.setNamedTarget("place"); // 팔을 "place" 위치로 이동 (주석 처리됨)
  // planAndExecute(arm_move_group_interface, plan, logger); // 계획 생성 및 실행 (주석 처리됨)
  // std::this_thread::sleep_for(3s); // 3초 대기 (주석 처리됨)

  //RCLCPP_INFO(logger, "Moving to pick position"); // 픽 위치로 이동 로그 (주석 처리됨)
  arm_move_group_interface.setNamedTarget("layover"); // 팔을 "layover" 위치로 이동
  planAndExecute(arm_move_group_interface, plan, logger); // 계획 생성 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기

  feed_back->package_id = package_id; // 피드백에 패키지 ID 설정
  feed_back->region = region; // 피드백에 지역 설정
  feed_back->process = 1; // 피드백에 프로세스 상태 1 설정
  goal_handle->publish_feedback(feed_back); // 피드백 발행
  
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
  sensor_msgs::msg::JointState arm_target_joint; // 목표 관절 상태 객체 생성
  arm_target_joint.name = jointname; // 관절 이름 설정
  arm_target_joint.position = jointstate; // 관절 위치 설정
  arm_move_group_interface.setJointValueTarget(arm_target_joint); // 목표 관절 값 설정
  planAndExecute(arm_move_group_interface, plan, logger); // 계획 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기

  current_pose = arm_move_group_interface.getCurrentPose().pose; // 현재 자세 다시 가져오기
  float end_Y = arm_move_group_interface.getCurrentRPY()[2]; // 현재 팔의 회전 각도(Yaw) 가져오기
  //float dx=(target_x + CAMFOCUS_X) * m_per_pixel; // X 거리 계산 (주석 처리됨)
  float dx=(CAMFOCUS_X - target_x) * m_per_pixel; // 카메라 초점에서 목표 X까지의 거리를 미터로 변환
  //float dy=(CAMFOCUS_Y - target_y) * m_per_pixel; // Y 거리 계산 (주석 처리됨)
  float dy=(target_y - CAMFOCUS_Y) * m_per_pixel; // 카메라 초점에서 목표 Y까지의 거리를 미터로 변환

  //float position_x = 0.233454 - dy; // 목표 X 위치 계산 (주석 처리됨)
  float position_x = 0.243454 - dy; // 목표 X 위치 계산 (기본값에 dy를 빼서 보정)
  float position_y = -0.085796 + dx; // 목표 Y 위치 계산 (기본값에 dx를 더해 보정)

  RCLCPP_INFO(logger, "from center Cube pose x,y,theta: %f %f %f",dy, dx, end_Y); // 계산된 큐브 위치와 각도 로그 출력
  RCLCPP_INFO(logger, "Current pose: %f %f %f", current_pose.position.x, current_pose.position.y, current_pose.position.z); // 현재 자세 로그 출력
  RCLCPP_INFO(logger, "target pose xyz: %f %f %f", position_x, position_y, current_pose.position.z); // 목표 자세 로그 출력

  //Cartesian path plan   
  std::vector<geometry_msgs::msg::Pose> waypoints; // 경로 점(waypoints) 벡터 생성

  current_pose = arm_move_group_interface.getCurrentPose().pose; // 현재 자세 다시 가져오기
  target_pose=current_pose; // 목표 자세를 현재 자세로 초기화

  target_pose.position.x = position_x; // 목표 X 위치 설정
  target_pose.position.y = position_y; // 목표 Y 위치 설정
  target_pose.position.z -= 0.010; // Z축을 0.01미터 낮춤 (접근 준비)
  waypoints.push_back(target_pose); // 첫 번째 경로 점 추가

  target_pose.position.z -= 0.010; // Z축을 추가로 0.01미터 낮춤 (최종 접근)
  waypoints.push_back(target_pose); // 두 번째 경로 점 추가

  moveit_msgs::msg::RobotTrajectory trajectory; // 로봇 궤적 객체 생성
  const double jump_threshold = 0.0; // 점프 임계값 설정 (0으로 설정하여 점프 방지)
  const double eef_step = 0.001; // 엔드 이펙터 이동 간격 설정 (0.001미터 단위)
  arm_move_group_interface.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory); // Cartesian 경로 계산
  arm_move_group_interface.execute(trajectory); // 계산된 궤적 실행

  std::this_thread::sleep_for(4s); // 4초 대기

  current_pose = arm_move_group_interface.getCurrentPose().pose; // 현재 자세 다시 가져오기
  RCLCPP_INFO(logger, "Current pose2: %f %f %f", current_pose.position.x, current_pose.position.y, current_pose.position.z); // 현재 자세 로그 출력

  //RCLCPP_INFO(logger, "Grasping object"); // 물체 잡기 로그 (주석 처리됨)
  hand_move_group_interface.setNamedTarget("hold"); // 그리퍼를 "hold" 상태로 설정 (물체 잡기)
  planAndExecute(hand_move_group_interface, plan, logger); // 계획 생성 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기
  
  //RCLCPP_INFO(logger, "Moving to place position"); // 놓기 위치로 이동 로그 (주석 처리됨)
  arm_move_group_interface.setNamedTarget("pickup"); // 팔을 "pickup" 위치로 이동
  planAndExecute(arm_move_group_interface, plan, logger); // 계획 생성 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기

  //RCLCPP_INFO(logger, "Moving to place position"); // 놓기 위치로 이동 로그 (주석 처리됨)
  arm_move_group_interface.setNamedTarget("place_ready"); // 팔을 "place_ready" 위치로 이동
  planAndExecute(arm_move_group_interface, plan, logger); // 계획 생성 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기

  feed_back->process = 2; // 피드백에 프로세스 상태 2 설정 (잡기 완료)
  goal_handle->publish_feedback(feed_back); // 피드백 발행

  current_pose = arm_move_group_interface.getCurrentPose().pose; // 현재 자세 다시 가져오기
  target_pose = current_pose; // 목표 자세를 현재 자세로 초기화
  waypoints.clear(); // 경로 점 벡터 초기화

  // Release point move
  if (region == "서울") { // 지역이 "서울"인 경우
    target_pose.position.x = get<0>(cube_drop_point_1[seoul_count]); // 서울 드롭 포인트 X 좌표 설정
    target_pose.position.y = get<1>(cube_drop_point_1[seoul_count]); // 서울 드롭 포인트 Y 좌표 설정
    target_pose.position.z = get<2>(cube_drop_point_1[seoul_count]); // 서울 드롭 포인트 Z 좌표 설정
  } else if (region == "부산") { // 지역이 "부산"인 경우
    target_pose.position.x = get<0>(cube_drop_point_2[busan_count]); // 부산 드롭 포인트 X 좌표 설정
    target_pose.position.y = get<1>(cube_drop_point_2[busan_count]); // 부산 드롭 포인트 Y 좌표 설정
    target_pose.position.z = get<2>(cube_drop_point_2[busan_count]); // 부산 드롭 포인트 Z 좌표 설정
  } else if (region == "경기") { // 지역이 "경기"인 경우
    target_pose.position.x = get<0>(cube_drop_point_3[gyeonggi_count]); // 경기 드롭 포인트 X 좌표 설정
    target_pose.position.y = get<1>(cube_drop_point_3[gyeonggi_count]); // 경기 드롭 포인트 Y 좌표 설정
    target_pose.position.z = get<2>(cube_drop_point_3[gyeonggi_count]); // 경기 드롭 포인트 Z 좌표 설정
  }
  waypoints.push_back(target_pose); // 드롭 포인트 경로 점 추가

  //일정 비율로 높이 조정(수정해야 함)
  target_pose.position.z -= 0.065; // Z축을 0.065미터 낮춤 (드롭 준비)
  waypoints.push_back(target_pose); // 낮춘 위치 경로 점 추가

  arm_move_group_interface.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory); // Cartesian 경로 계산
  arm_move_group_interface.execute(trajectory); // 계산된 궤적 실행

  std::this_thread::sleep_for(3s); // 3초 대기

  //RCLCPP_INFO(logger, "Releasing object"); // 물체 놓기 로그 (주석 처리됨)
  hand_move_group_interface.setNamedTarget("release"); // 그리퍼를 "release" 상태로 설정 (물체 놓기)
  planAndExecute(hand_move_group_interface, plan, logger); // 계획 생성 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기

  current_pose = arm_move_group_interface.getCurrentPose().pose; // 현재 자세 다시 가져오기
  target_pose = current_pose; // 목표 자세를 현재 자세로 초기화
  waypoints.clear(); // 경로 점 벡터 초기화

  target_pose.position.z += 0.200; // Z축을 0.2미터 올림 (원위치로 복귀 준비)
  waypoints.push_back(target_pose); // 올린 위치 경로 점 추가

  arm_move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // Cartesian 경로 계산
  arm_move_group_interface.execute(trajectory); // 계산된 궤적 실행
  std::this_thread::sleep_for(3s); // 3초 대기

  //RCLCPP_INFO(logger, "Returning to initial position"); // 초기 위치로 복귀 로그 (주석 처리됨)
  arm_move_group_interface.setNamedTarget("place_ready"); // 팔을 "place_ready" 위치로 이동
  planAndExecute(arm_move_group_interface, plan, logger); // 계획 생성 및 실행
  std::this_thread::sleep_for(3s); // 3초 대기

  feed_back->process = 3; // 피드백에 프로세스 상태 3 설정 (드롭 완료)
  goal_handle->publish_feedback(feed_back); // 피드백 발행

  RCLCPP_INFO(logger, "Pick and place cycle completed"); // Pick and Place 사이클 완료 로그 출력
}

//==================== Action Server ====================//
class PickPlaceActionServer : public rclcpp::Node // PickPlace 액션 서버 클래스 정의 (ROS 2 노드 상속)
{
public:
  using PlcCobot = kairos_interfaces::action::PlcCobot; // PlcCobot 액션 타입 별칭 설정
  using GoalHandlePlcCobot = rclcpp_action::ServerGoalHandle<PlcCobot>; // 목표 핸들 타입 별칭 설정

  PickPlaceActionServer() // 생성자 정의
  : Node("pickplace_action_server") // 노드 이름 "pickplace_action_server"로 초기화
  {
    // 생성된 노드를 non-owning shared_ptr로 변환 (이미 외부에서 관리중)
    auto node_ptr = rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){}); // 노드를 스마트 포인터로 변환 (메모리 관리 생략)

    // MoveIt 인터페이스 생성 (팔과 그리퍼)
    arm_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "mycobot_arm"); // 팔 MoveGroup 인터페이스 생성
    hand_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "hand"); // 손 MoveGroup 인터페이스 생성

    // Planning Scene 설정 및 충돌 객체 적용
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(); // Planning Scene 인터페이스 생성
    auto object_names = planning_scene_interface_->getKnownObjectNames(); // 기존 충돌 객체 이름 목록 가져오기
    if (!object_names.empty()) { // 기존 객체가 존재하면
      planning_scene_interface_->removeCollisionObjects(object_names); // 기존 충돌 객체 제거
    }
    auto collision_objects = addCollisionObjects(*arm_move_group_interface_); // 새로운 충돌 객체 생성
    planning_scene_interface_->applyCollisionObjects(collision_objects); // 충돌 객체 적용
    // for (auto & obj : collision_objects) { // 각 충돌 객체를 개별 적용 (주석 처리됨)
    //   planning_scene_interface_->applyCollisionObject(obj); // 개별 객체 적용 (주석 처리됨)
    // }

    // Action Server 생성
    action_server_ = rclcpp_action::create_server<PlcCobot>( // 액션 서버 생성
      this, // 현재 노드
      "pick_place_action", // 액션 이름
      std::bind(&PickPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2), // 목표 수신 콜백
      std::bind(&PickPlaceActionServer::handle_cancel, this, std::placeholders::_1), // 취소 요청 콜백
      std::bind(&PickPlaceActionServer::handle_accepted, this, std::placeholders::_1) // 목표 수락 콜백
    );

    subscription_ = this->create_subscription<std_msgs::msg::Bool>( // "cobot_init" 토픽 구독 설정
      "cobot_init", // 토픽 이름
      10, // 큐 크기
      std::bind(&PickPlaceActionServer::cobot_init_callback, this, std::placeholders::_1) // 콜백 함수 바인딩
    );
      

    RCLCPP_INFO(this->get_logger(), "PickPlace Action Server started."); // 액션 서버 시작 로그 출력
  }

private:
  rclcpp_action::Server<PlcCobot>::SharedPtr action_server_; // 액션 서버 스마트 포인터
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_; // 구독 스마트 포인터
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_interface_; // 팔 MoveGroup 인터페이스 스마트 포인터
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_interface_; // 손 MoveGroup 인터페이스 스마트 포인터
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_; // Planning Scene 인터페이스 스마트 포인터
  moveit::planning_interface::MoveGroupInterface::Plan plan_; // MoveIt 계획 객체

  // 충돌 객체를 추가하는 함수
  auto addCollisionBox(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string region) { // 충돌 박스 추가 함수 정의
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects; // 충돌 객체 벡터 생성
    moveit_msgs::msg::CollisionObject collision_object; // 충돌 객체 생성
    // world 프레임을 사용 (주석 처리됨)
    //collision_object.header.frame_id = "world"; // 프레임 ID를 "world"로 설정 (주석 처리됨)
    collision_object.header.frame_id = move_group_interface.getPlanningFrame(); // 플래닝 프레임 ID 설정

    float x = 0.0; // X 좌표 초기화
    float y = 0.0; // Y 좌표 초기화
    float z = 0.0; // Z 좌표 초기화
    std::string region_id = ""; // 지역 ID 초기화
    int id_count = 0; // ID 카운터 초기화

    if (region == "서울" ) { // 지역이 "서울"인 경우
      x = get<0>(cube_drop_point_1[seoul_count]); // 서울 드롭 포인트 X 좌표 가져오기
      y = get<1>(cube_drop_point_1[seoul_count]); // 서울 드롭 포인트 Y 좌표 가져오기
      //z = get<2>(cube_drop_point_1[seoul_count]) - 0.280; // Z 좌표 계산 (주석 처리됨)
      if (seoul_count > 1) // 두 번째 이상 드롭이면
        z = 0.03; // Z를 0.03으로 설정
      region_id = "seoul"; // 지역 ID 설정
      id_count = seoul_count; // 카운터 설정
      seoul_count++; // 서울 카운터 증가
    } else if (region == "부산") { // 지역이 "부산"인 경우
      x = get<0>(cube_drop_point_2[busan_count]); // 부산 드롭 포인트 X 좌표 가져오기
      y = get<1>(cube_drop_point_2[busan_count]); // 부산 드롭 포인트 Y 좌표 가져오기
      if (busan_count > 1) // 두 번째 이상 드롭이면
        z = 0.03; // Z를 0.03으로 설정
      region_id = "busan"; // 지역 ID 설정
      id_count = busan_count; // 카운터 설정
      busan_count++; // 부산 카운터 증가
    } else if (region == "경기") { // 지역이 "경기"인 경우
      x = get<0>(cube_drop_point_3[gyeonggi_count]); // 경기 드롭 포인트 X 좌표 가져오기
      y = get<1>(cube_drop_point_3[gyeonggi_count]); // 경기 드롭 포인트 Y 좌표 가져오기
      if (gyeonggi_count > 1) // 두 번째 이상 드롭이면
        z = 0.03; // Z를 0.03으로 설정
      region_id = "gyeonggi"; // 지역 ID 설정
      id_count = gyeonggi_count; // 카운터 설정
      gyeonggi_count++; // 경기 카운터 증가
    }
    
    // 고유한 ID 생성
    std::string id = "box_" + region_id + "_" + std::to_string(id_count); // 고유한 충돌 객체 ID 생성
    collision_object.id = id; // 객체 ID 설정
    
    // 박스 설정
    shape_msgs::msg::SolidPrimitive primitive; // 기본 도형 객체 생성
    primitive.type = primitive.BOX; // 도형 타입을 박스로 설정
    primitive.dimensions.resize(3); // 크기 배열 크기를 3으로 설정
    primitive.dimensions[primitive.BOX_X] = 0.04; // 박스의 X 크기 설정
    primitive.dimensions[primitive.BOX_Y] = 0.04; // 박스의 Y 크기 설정
    primitive.dimensions[primitive.BOX_Z] = 0.03; // 박스의 Z 크기 설정

    geometry_msgs::msg::Pose box_pose; // 박스 위치와 방향 설정
    box_pose.orientation.w = 1.0; // 쿼터니언의 w 값을 1로 설정 (기본 방향)
    box_pose.position.x = x; // X 위치 설정
    box_pose.position.y = y; // Y 위치 설정
    box_pose.position.z = z; // Z 위치 설정

    collision_object.primitives.push_back(primitive); // 도형을 충돌 객체에 추가
    collision_object.primitive_poses.push_back(box_pose); // 위치를 충돌 객체에 추가
    collision_object.operation = collision_object.ADD; // 객체 추가 작업 설정

    collision_objects.push_back(collision_object); // 충돌 객체 벡터에 추가

    // std::vector<moveit_msgs::msg::CollisionObject> collision_objects; // 충돌 객체 벡터 선언 (중복, 주석 처리됨)
    // collision_objects.push_back(collision_object); // 충돌 객체 추가 (중복, 주석 처리됨)
    // planning_scene_interface_->addCollisionObjects(collision_objects); // 충돌 객체 추가 (주석 처리됨)

    RCLCPP_INFO(this->get_logger(), "Collision box added at x=%f, y=%f, z=%f with ID: %s", // 충돌 박스 추가 로그 출력
                x, y, z, id.c_str());
      
    return collision_objects; // 충돌 객체 벡터 반환
  }

  // Goal 수신 콜백
  rclcpp_action::GoalResponse handle_goal( // 목표 수신 콜백 함수 정의
      const rclcpp_action::GoalUUID & /*uuid*/, // 목표 UUID (사용 안 함)
      std::shared_ptr<const PlcCobot::Goal> /*goal*/) // 목표 객체 (사용 안 함)
  {
    //RCLCPP_INFO(this->get_logger(), "Received PlcCobot goal."); // 목표 수신 로그 (주석 처리됨)
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // 목표를 수락하고 즉시 실행
  }

  // 취소 요청 콜백
  rclcpp_action::CancelResponse handle_cancel( // 취소 요청 콜백 함수 정의
      const std::shared_ptr<GoalHandlePlcCobot> goal_handle) // 목표 핸들
  {
    //RCLCPP_INFO(this->get_logger(), "Cancel request received."); // 취소 요청 로그 (주석 처리됨)
    (void)goal_handle; // 컴파일러 경고 방지 (사용 안 함)
    return rclcpp_action::CancelResponse::ACCEPT; // 취소 요청 수락
  }

  // Goal 수락 후 별도 스레드에서 실행
  void handle_accepted(const std::shared_ptr<GoalHandlePlcCobot> goal_handle) // 목표 수락 후 처리 함수 정의
  {
    std::thread{std::bind(&PickPlaceActionServer::execute, this, std::placeholders::_1), goal_handle}.detach(); // 별도 스레드에서 실행
  }

  void cobot_init_callback(const std_msgs::msg::Bool::SharedPtr msg) // 코봇 초기화 콜백 함수 정의
  {
    if(msg->data == true){ // 메시지가 true인 경우
      RCLCPP_INFO(this->get_logger(), "Init signal received."); // 초기화 신호 수신 로그 출력
      seoul_count = 0; // 서울 카운터 초기화
      busan_count = 0; // 부산 카운터 초기화
      gyeonggi_count = 0; // 경기 카운터 초기화
    }
  }

  // 실제 Pick and Place 사이클 실행 및 결과 전송
  void execute(const std::shared_ptr<GoalHandlePlcCobot> goal_handle) // Pick and Place 실행 함수 정의
  {
    //RCLCPP_INFO(this->get_logger(), "Executing Pick and Place cycle..."); // 사이클 실행 로그 (주석 처리됨)
    const auto goal = goal_handle->get_goal(); // 목표 객체 가져오기
    const auto package_id = goal->package_id; // 패키지 ID 가져오기
    const auto region = goal->region; // 지역 가져오기
    const auto x = goal->x; // X 좌표 가져오기
    const auto y = goal->y; // Y 좌표 가져오기
    const auto theta = goal->theta; // 각도 가져오기
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%f, y=%f, theta=%f", x, y, theta); // 목표 정보 로그 출력
    executePickAndPlace(*arm_move_group_interface_, *hand_move_group_interface_, plan_, x, y, theta, region, package_id, this->get_logger(), goal_handle); // Pick and Place 실행

    // 충돌 객체 추가
    auto collision_objects = addCollisionBox(*arm_move_group_interface_, region); // 충돌 박스 추가
    planning_scene_interface_->addCollisionObjects(collision_objects); // 충돌 객체 적용

    auto result = std::make_shared<PlcCobot::Result>(); // 액션 결과 객체 생성
    result->plc_result_action = true; // 결과 성공으로 설정
    //result->plc_feedback_action = true; // 피드백 결과 (주석 처리됨)
    goal_handle->succeed(result); // 결과 성공 전송
    //RCLCPP_INFO(this->get_logger(), "Cycle completed. Result sent."); // 사이클 완료 및 결과 전송 로그 (주석 처리됨)
  }
};

int main(int argc, char* argv[]) // 메인 함수 정의
{
  rclcpp::init(argc, argv); // ROS 2 초기화
  initSetting(); // 초기 설정 함수 호출
  signal(SIGINT, signalHandler); // SIGINT 신호 핸들러 등록

  // Action Server 노드 생성 후 spin
  auto server_node = std::make_shared<PickPlaceActionServer>(); // 액션 서버 노드 생성
  
  while (rclcpp::ok()){ // ROS가 정상 작동 중일 때 반복
    if(g_signal_shutdown){ // 종료 신호가 true이면
      RCLCPP_INFO(rclcpp::get_logger("pickplace_test_node"), "Shutdown signal received, shutting down..."); // 종료 로그 출력
      rclcpp::shutdown(); // ROS 종료
      break; // 루프 탈출
    }
    rclcpp::spin_some(server_node); // 노드 스핀 (이벤트 처리)
  }
  
  return 0; // 프로그램 종료
}

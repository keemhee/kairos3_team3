#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <thread>
#include <chrono>
#include <csignal>

using namespace std::chrono_literals;

bool g_signal_shutdown = false;

void signalHandler(int /*signum*/) {
    RCLCPP_INFO(rclcpp::get_logger("pickplace_test_node"), "Interrupt signal received");
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

    // Pick-up box
    {
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
    }

    return collision_objects;
}

void executePickAndPlace(moveit::planning_interface::MoveGroupInterface& arm_move_group_interface, 
                         moveit::planning_interface::MoveGroupInterface& hand_move_group_interface, 
                         moveit::planning_interface::MoveGroupInterface::Plan& plan, 
                         rclcpp::Logger logger) {

    RCLCPP_INFO(logger, "gripper open");
    hand_move_group_interface.setNamedTarget("release");
    planAndExecute(hand_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    //RCLCPP_INFO(logger, "Moving");
    //arm_move_group_interface.setNamedTarget("layover");
    //planAndExecute(arm_move_group_interface, plan, logger);
    //std::this_thread::sleep_for(3s);

    // Pick 위치로 이동
    RCLCPP_INFO(logger, "Moving to pick position");
    arm_move_group_interface.setNamedTarget("pick");
    planAndExecute(arm_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    // Hold (그리퍼로 물체 잡기)
    RCLCPP_INFO(logger, "Grasping object");
    hand_move_group_interface.setNamedTarget("hold");
    planAndExecute(hand_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    //RCLCPP_INFO(logger, "Moving");
    //arm_move_group_interface.setNamedTarget("layover");
    //planAndExecute(arm_move_group_interface, plan, logger);
    //std::this_thread::sleep_for(3s);

    // Place 위치로 이동
    RCLCPP_INFO(logger, "Moving to place position");
    arm_move_group_interface.setNamedTarget("place");
    planAndExecute(arm_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    // Release (그리퍼 열어서 물체 놓기)
    RCLCPP_INFO(logger, "Releasing object");
    hand_move_group_interface.setNamedTarget("release");
    planAndExecute(hand_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    // 초기 위치로 복귀
    RCLCPP_INFO(logger, "Returning to initial position");
    arm_move_group_interface.setNamedTarget("home");
    planAndExecute(arm_move_group_interface, plan, logger);
    std::this_thread::sleep_for(3s);

    RCLCPP_INFO(logger, "Pick and place cycle completed");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    signal(SIGINT, signalHandler);

    auto node = std::make_shared<rclcpp::Node>("pickplace_test_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("pickplace_test_node");

    auto arm_move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "mycobot_arm");
    auto hand_move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "hand");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto objectname = planning_scene_interface.getKnownObjectNames();
    for (auto& elem : objectname) std::cout << elem << " \n";
    planning_scene_interface.removeCollisionObjects(objectname);

    auto collision_objects = addCollisionObjects(arm_move_group_interface);
    
    // 각 CollisionObject를 하나씩 적용
    for (auto& obj : collision_objects) {
        planning_scene_interface.applyCollisionObject(obj);
    }

    while (rclcpp::ok() && !g_signal_shutdown) {
        rclcpp::spin_some(node); 
        
        RCLCPP_INFO(logger, "Executing pick and place cycle");

        executePickAndPlace(arm_move_group_interface, hand_move_group_interface, plan, logger);
        
        std::this_thread::sleep_for(5s);
    }

    arm_move_group_interface.setNamedTarget("home");
    planAndExecute(arm_move_group_interface, plan, logger);
    std::this_thread::sleep_for(5s);

    rclcpp::shutdown();
    return 0;
}

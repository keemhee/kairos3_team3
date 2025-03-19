#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_cpp_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
/*#include <iostream>
#include <vector>
#include <tuple>
#include <string>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        arm_move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("arm");
        hand_move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("hand");
        planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

        box_detection_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/box_detection", 10, std::bind(&RobotController::boxDetectionCallback, this, std::placeholders::_1));
    }

    void run() {
        setupEnvironment();
        moveToInitialState();
        processDetectedBoxes();
    }

private:
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_interface_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_interface_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr box_detection_subscriber_;
    std::vector<std::tuple<float, float, float, float>> box_detected_;

    void setupEnvironment() {
        auto collision_object = addCollision(*arm_move_group_interface_);
        planning_scene_interface_->applyCollisionObject(collision_object);
    }

    void moveToInitialState() {
        arm_move_group_interface_->setNamedTarget("home");
        if (!planAndExecute(*arm_move_group_interface_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to home position");
            return;
        }
        std::this_thread::sleep_for(2s);
        
        hand_move_group_interface_->setNamedTarget("release");
        if (!planAndExecute(*hand_move_group_interface_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move hand to release position");
            return;
        }
        std::this_thread::sleep_for(1s);
    }

    void processDetectedBoxes() {
        if (box_detected_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No boxes detected");
            return;
        }

        for (const auto& [x, y, z, _] : box_detected_) {
            auto box_collision = addCubeCollision(*arm_move_group_interface_, x, y, z);
            planning_scene_interface_->applyCollisionObject(box_collision);

            pickAndPlaceBox(x, y, z);
        }
    }

    void pickAndPlaceBox(float x, float y, float z) {
        arm_move_group_interface_->setNamedTarget("pick");
        if (!planAndExecute(*arm_move_group_interface_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to pick position");
            return;
        }
        
        hand_move_group_interface_->setNamedTarget("hold");
        if (!planAndExecute(*hand_move_group_interface_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to close gripper");
            return;
        }

        arm_move_group_interface_->setNamedTarget("place");
        if (!planAndExecute(*arm_move_group_interface_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to place position");
            return;
        }
        
        hand_move_group_interface_->setNamedTarget("release");
        if (!planAndExecute(*hand_move_group_interface_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open gripper");
            return;
        }

        moveToInitialState();
    }

    bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface.plan(plan));
        if (success) {
            move_group_interface.execute(plan);
            RCLCPP_INFO(this->get_logger(), "Execution successful!");
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            return false;
        }
    }

    moveit_msgs::msg::CollisionObject addCollision(moveit::planning_interface::MoveGroupInterface& move_group_interface) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface.getPlanningFrame();
        collision_object.id = "conveyor";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.3;
        primitive.dimensions[primitive.BOX_Y] = 0.13;
        primitive.dimensions[primitive.BOX_Z] = 0.09;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.383;
        box_pose.position.y = -0.10;
        box_pose.position.z = primitive.dimensions[primitive.BOX_Z] / 2;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);

        shape_msgs::msg::SolidPrimitive left_side;
        shape_msgs::msg::SolidPrimitive right_side;
        left_side.type = left_side.BOX;
        left_side.dimensions.resize(3);
        left_side.dimensions[primitive.BOX_X] = 0.145;
        left_side.dimensions[primitive.BOX_Y] = 0.04;
        left_side.dimensions[primitive.BOX_Z] = 0.155;
        right_side = left_side;

        // Left side pose
        box_pose.position.x = 0.461;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.0775;
        collision_object.primitives.push_back(left_side);
        collision_object.primitive_poses.push_back(box_pose);

        // Right side pose
        box_pose.position.x = 0.461;
        box_pose.position.y = -0.20;
        box_pose.position.z = 0.0775;
        collision_object.primitives.push_back(right_side);
        collision_object.primitive_poses.push_back(box_pose);

        // Camera frame pose
        shape_msgs::msg::SolidPrimitive camera_frame;
        camera_frame.type = camera_frame.BOX;
        camera_frame.dimensions.resize(3);
        camera_frame.dimensions[primitive.BOX_X] = 0.02;
        camera_frame.dimensions[primitive.BOX_Y] = 0.32;
        camera_frame.dimensions[primitive.BOX_Z] = 0.32;
        box_pose.position.x = 0.433;
        box_pose.position.y = -0.08;
        box_pose.position.z = 0.315;
        collision_object.primitives.push_back(camera_frame);
        collision_object.primitive_poses.push_back(box_pose);

        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        return collision_object;
    }

    moveit_msgs::msg::CollisionObject addCubeCollision(moveit::planning_interface::MoveGroupInterface& move_group_interface, double cube_x, double cube_y, double cube_z) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface.getPlanningFrame();
        collision_object.id = "box";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.03;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.05;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.04;

        geometry_msgs::msg::Pose box_pose; 
        box_pose.orientation.w = 1.0;
        box_pose.position.x = cube_x;
        box_pose.position.y = cube_y;
        box_pose.position.z = cube_z + primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] / 2;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        return collision_object;
    }

    void boxDetectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        // Implementation of box detection callback
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
*/
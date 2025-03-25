# kairos_project 🤖  
25.01 ~ 25.03.20

## 📦📦📦📦📦📦📦📦📦📦📦
- 🚚 myAGV : trace yellow line -> detect red -> rear parking
      (ros2 run line_tracing_agv agv_node)    
- ⚡ PLC : receive agv parking signal -> operate conveyor
      (ros2 run plc new_plc)
- 📸 camera : recognize QR code
-     (ros2 run QR_detector detect_send)
- 🦾 myCobot : place item at delivery location based on QR code when detected by the sensor -> send completion signal to AGV when all items are delivered
      (ros2 run moveit_cobot_pkg pick_place_action_server_test)
      (ros2 launch mycobot_with_gripper demo.launch.py)

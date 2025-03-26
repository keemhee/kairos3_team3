# kairos_project 🤖  
25.01 ~ 25.03.20
# Automated Logistics System

This repository implements a ROS 2-based logistics automation system featuring AGV navigation, QR code detection, PLC coordination, robotic arm control, and optional web-based monitoring.

## 📦📦📦📦📦📦📦📦📦📦📦
### web viewer💻
- Monitors AGV, PLC, and cobot status along with package processing logs via a web interface.
- Subscribes to `/agv_ready`, `/plc_complate`, and `/packages_status` topics, displaying real-time data using Flask.
- `ros2 run web_viewer web_viewer_node`

### plc⚡
- Communicates with the PLC, receives AGV parking signals, and operates the conveyor system.
- Receives `/agv_ready` signal, controls conveyor via Modbus, coordinates with cobot, and publishes `/plc_complate`.
- `ros2 run plc new_plc`

### agv🚚
- Controls the AGV to trace yellow lines, detect red lines, and perform rear parking.
- Uses camera-based line tracing, publishes `/agv_ready` upon parking, and subscribes to `/plc_complate` to resume navigation.
- `ros2 run line_tracing_agv agv_node`

### camera📸
- Recognizes QR codes using a camera and provides package information.
- Detects QR codes, communicates with a server via socket, and serves `/package_info` while subscribing to `/package_delete`.
- `ros2 run QR_detector detect_send`

### myCobot with MoveIt🦾
- Controls the myCobot robotic arm to pick and place items based on QR code data.
- Runs `/pick_place_action` action server, uses MoveIt for arm control, places items at delivery locations, and subscribes to `/cobot_init`.
- `ros2 run moveit_cobot_pkg pick_place_action_server_test`
- **Supporting Nodes**
  - `moveit_sub`: Subscribes to MoveIt-related topics (`ros2 run mycobot_pkg moveit_sub`).
  - `demo.launch.py`: Launches myCobot with gripper configuration (`ros2 launch mycobot_with_gripper demo.launch.py`).

## 2. System Workflow

This system automates logistics by integrating an AGV for package transport, a camera for QR code detection, a PLC for conveyor control, and a myCobot arm for item placement, with optional web monitoring.

### Step 1: AGV Navigation and Parking
- **myAGV (`agv_node`)**: Traces a yellow line, detects a red line, performs rear parking, and publishes `/agv_ready=True`.
- **PLC (`new_plc`)**: Receives `/agv_ready` and prepares conveyor operation.

### Step 2: QR Code Detection
- **Camera (`detect_send`)**: Scans QR codes, retrieves package details (e.g., delivery location) from a server, and provides them via `/package_info`.
- **PLC**: Calls `/package_info` to fetch QR data and triggers cobot actions.

### Step 3: Cobot Item Placement
- **myCobot (`pick_place_action_server_test`)**:
  - Receives `/pick_place_action` goal from PLC.
  - Picks up items and places them at designated delivery locations (e.g., Seoul, Busan, Gyeonggi) based on QR data.
  - Sends completion feedback.
- **PLC**: Monitors cobot progress; upon completion of all items, publishes `/plc_complate=True`.

### Step 4: Cycle Completion and Monitoring
- **myAGV**: Resumes navigation upon receiving `/plc_complate=True`.
- **Web Viewer (Optional)**: Displays real-time status via `/agv_ready`, `/plc_complate`, and `/packages_status` if enabled.

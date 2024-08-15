Here's the updated README with the additional command included in the third step for AprilTag detection:

---

# NVIDIA Isaac Sim and Isaac ROS Integration on Jetson Orin Nano

This repository provides a comprehensive guide on integrating NVIDIA Isaac Sim with Isaac ROS on the Jetson Orin Nano for AprilTag following. This setup enables real-time AprilTag detection and dynamic robot control, enhancing robotics workflows.

## Overview

This project demonstrates how to:

- **Simulate and Detect AprilTags:** Utilize NVIDIA Isaac Sim to simulate a robotics environment and detect AprilTags.
- **Follow AprilTags with Isaac ROS:** Implement a ROS2 node to process AprilTag data and control robot movement based on detected tags.
- **Visualize in RViz:** Use RViz for real-time visualization of AprilTag detections and robot movements.
- **Leverage JetPack 6:** Optimize performance on the Jetson Orin Nano.
- **Implement Hardware-in-the-Loop (HIL) Testing:** Connect the Jetson Orin Nano with a laptop for enhanced testing and validation.

## Requirements

- **NVIDIA Jetson Orin Nano:** Ensure your device is set up with JetPack 6.
- **NVIDIA Isaac Sim:** For simulating the robotics environment.
- **Isaac ROS:** Installed on the Jetson Orin Nano.
- **Hardware-in-the-Loop (HIL) Setup:** For connecting the Jetson Orin Nano with a laptop.
  ![Image segmentation(4)](https://github.com/user-attachments/assets/4bc1b81a-ce2f-44b8-9e28-24491ab53482)
  *Figure 1: Hardware setup for Isaac ROS on the Jetson Orin Nano.*

## Installation and Setup

### 1. Setting Up Jetson Orin Nano

1. **Flash JetPack 6:** Use the NVIDIA SDK Manager to flash JetPack 6 on your Jetson Orin Nano.
   - [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)

2. **Install Isaac ROS:** Follow the instructions to install Isaac ROS on the Jetson Orin Nano.
   - [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)

### 2. Configuring Network and ROS DDS Domain ID

1. **Network Configuration:** Connect your Jetson Orin Nano to the network. Optionally, configure a static IP address.

2. **Set Up ROS DDS Domain ID:**
   - Edit your ROS 2 workspace setup file to include the `ROS_DOMAIN_ID` environment variable.
   - Source the setup file to apply changes.

### 3. Integrating Isaac Sim with Isaac ROS for AprilTag Following

1. **Simulate AprilTags in Isaac Sim:** Run your simulation in Isaac Sim to detect and visualize AprilTags.
   ![Screenshot from 2024-08-15 17-57-32](https://github.com/user-attachments/assets/33b7a741-7ee2-4ea6-b493-fb1a80678122)
   *Figure 2: AprilTag detection in Isaac Sim.*

2. **Process AprilTag Data with Isaac ROS:**
   - **Clone the Isaac ROS AprilTag Repository:**
     ```bash
     git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
     ```
     Navigate to the cloned repository directory if needed.
   - **Launch Isaac ROS Nodes:** Start the AprilTag detection node and other necessary nodes:
     ```bash
     ros2 launch isaac_ros_apriltag isaac_ros_apriltag_following.launch.py
     ```
   - **Run AprilTag Detection Node:** Execute the AprilTag detector to start processing:
     ```bash
     ros2 run apriltag_detection apriltag_detector
     ```
   - **Implement Movement Logic:** Develop a ROS2 node to subscribe to the AprilTag detection topic and control robot movement based on detected tags.

3. **Visualize in RViz:**
   - Start RViz to visualize AprilTag detections and robot movements:
     ```bash
     rviz2
     ```

### 4. Implementing Hardware-in-the-Loop (HIL) Testing

1. **Connect to Laptop:** Set up HIL testing to link the Jetson Orin Nano with a laptop for enhanced validation and real-time testing.

## Figures

- **Figure 1:** Illustrates the setup for Isaac ROS on the Jetson Orin Nano using JetPack 6.
- **Figure 2:** Shows the Isaac Sim environment with AprilTag detection.
- **Figure 3:** Displays the data flow from Isaac Sim to Isaac ROS and visualization in RViz.

## Launching Commands

To launch the simulation and data processing, use the following commands:

1. **Start Isaac Sim:**
    ```bash
    isaac-sim --start
    ```

2. **Launch Isaac ROS Nodes for AprilTag Following:**
    ```bash
    ros2 launch isaac_ros_apriltag isaac_ros_apriltag_following.launch.py
    ```

3. **Run AprilTag Detection Node:**
    ```bash
    ros2 run apriltag_detection apriltag_detector
    ```

4. **Start RViz for Visualization:**
    ```bash
    rviz2
    ```

## Special Thanks

Special thanks to Ninad for the invaluable guidance on implementing this solution.

## Learn More

For detailed steps and additional information, visit the [Medium Post](https://medium.com/@kabilankb2003/hardware-in-the-loop-with-nvidia-jetson-orin-nano-using-isaac-sim-and-isaac-ros-apriltag-a59d78a7f146).

## License

This project is licensed under the [MIT License](LICENSE).


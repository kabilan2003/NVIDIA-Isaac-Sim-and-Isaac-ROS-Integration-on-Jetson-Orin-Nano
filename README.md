# NVIDIA-Isaac-Sim-and-Isaac-ROS-Integration-on-Jetson-Orin-Nano
This repository provides a detailed guide on integrating NVIDIA Isaac Sim with Isaac ROS on the Jetson Orin Nano, utilizing JetPack 6 for efficient AprilTag detection. This setup enhances robotics workflows by enabling real-time AprilTag detection and visualization.
# Overview

This project demonstrates how to:

   1. Simulate and Detect AprilTags: Use NVIDIA Isaac Sim to simulate a robotics environment and detect AprilTags.
   2. Process Data with Isaac ROS: Transfer AprilTag data to Isaac ROS on the Jetson Orin Nano.
   3. Visualize in RViz: Use RViz for real-time visualization of AprilTag detections.
   4. Leverage JetPack 6: Optimize performance on the Jetson Orin Nano.
   5. Implement Hardware-in-the-Loop (HIL) Testing: Connect the Jetson Orin Nano with a laptop for enhanced testing and validation.

# Requirements

   1. NVIDIA Jetson Orin Nano: Ensure your device is set up with JetPack 6.
   2. NVIDIA Isaac Sim: For simulating the robotics environment.
   3. Isaac ROS: Installed on the Jetson Orin Nano.
   4. Hardware-in-the-Loop (HIL) Setup: For connecting the Jetson Orin Nano with a laptop.

      ![Image segmentation(4)](https://github.com/user-attachments/assets/57f29373-cb81-42e2-b4e2-68bc84ad5ded)

# Installation and Setup
Setting Up Jetson Orin Nano

   1. Flash JetPack 6: Use the NVIDIA SDK Manager to flash JetPack 6 on your Jetson Orin Nano.
        NVIDIA SDK Manager

    2. Install Isaac ROS: Follow the instructions to install Isaac ROS on the Jetson Orin Nano.
        Isaac ROS Documentation
      
Here's a full README for your GitHub repository:

---

# NVIDIA Isaac Sim and Isaac ROS Integration on Jetson Orin Nano

This repository provides a detailed guide on integrating NVIDIA Isaac Sim with Isaac ROS on the Jetson Orin Nano, utilizing JetPack 6 for efficient AprilTag detection. This setup enhances robotics workflows by enabling real-time AprilTag detection and visualization.

## Overview

This project demonstrates how to:

- **Simulate and Detect AprilTags**: Use NVIDIA Isaac Sim to simulate a robotics environment and detect AprilTags.
- **Process Data with Isaac ROS**: Transfer AprilTag data to Isaac ROS on the Jetson Orin Nano.
- **Visualize in RViz**: Use RViz for real-time visualization of AprilTag detections.
- **Leverage JetPack 6**: Optimize performance on the Jetson Orin Nano.
- **Implement Hardware-in-the-Loop (HIL) Testing**: Connect the Jetson Orin Nano with a laptop for enhanced testing and validation.

## Requirements

- **NVIDIA Jetson Orin Nano**: Ensure your device is set up with JetPack 6.
- **NVIDIA Isaac Sim**: For simulating the robotics environment.
- **Isaac ROS**: Installed on the Jetson Orin Nano.
- **Hardware-in-the-Loop (HIL) Setup**: For connecting the Jetson Orin Nano with a laptop.

## Installation and Setup

### 1. Setting Up Jetson Orin Nano

1. **Flash JetPack 6**: Use the NVIDIA SDK Manager to flash JetPack 6 on your Jetson Orin Nano.
   - [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)

2. **Install Isaac ROS**: Follow the instructions to install Isaac ROS on the Jetson Orin Nano.
   - [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)

### 2. Configuring Network and ROS DDS Domain ID

1. **Network Configuration**: Connect your Jetson Orin Nano to the network. Optionally, configure a static IP address.

2. **Set Up ROS DDS Domain ID**:
   - Edit your ROS 2 workspace setup file to include the `ROS_DOMAIN_ID` environment variable.
   - Source the setup file to apply changes.

### 3. Integrating Isaac Sim with Isaac ROS

1. **Simulate AprilTags in Isaac Sim**: Run your simulation and detect AprilTags.

2. **Share Data with Isaac ROS**: Use ROS topics to publish AprilTag detection data from Isaac Sim to Isaac ROS on the Jetson Orin Nano.

3. **Visualize in RViz**: Subscribe to the relevant ROS topics in RViz to view AprilTag detections.

### 4. Implementing Hardware-in-the-Loop (HIL) Testing

1. **Connect to Laptop**: Set up HIL testing to link the Jetson Orin Nano with a laptop for enhanced validation.

## Figures

- **Figure 4**: Shows the data flow from Isaac Sim to Isaac ROS and visualization in RViz.
- **Figure 5**: Illustrates the setup for Isaac ROS on the Jetson Orin Nano using JetPack 6.

## Special Thanks

Special thanks to Ninad for the incredible guidance on implementing this solution.

## Learn More

For detailed steps and additional information, visit the [Medium Post](https://medium.com/@kabilankb2003/hardware-in-the-loop-with-nvidia-jetson-orin-nano-using-isaac-sim-and-isaac-ros-apriltag-a59d78a7f146).

## License

This project is licensed under the [MIT License](LICENSE).

---

Feel free to adjust any sections as needed or add more specific instructions based on your project’s requirements.

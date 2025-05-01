# BMS_for_UGV

> **_NOTE:_** This repo is a part of my bachelor diploama in CS

[![ROS2 CI](https://github.com/PHentosh/battery_monitoring_system/actions/workflows/ros2.yml/badge.svg)](https://github.com/PHentosh/battery_monitoring_system/actions/workflows/ros2.yml)

This ROS2 package implements a battery monitoring system that estimates the State of Charge (SoC) and State of Health (SoH) of a battery. It utilizes two estimation methods: the Extended Kalman Filter (EKF) and Coulomb counting.

## Overview

The system subscribes to a single ROS2 topic, `/a200_1056/platform/mcu/status/power`(**please change it according to your system**), to receive raw battery data. Upon processing this data, it publishes the estimated SoC and SoH values to two separate topics:

* **`coulomb_bms_state`**: Contains the estimated SoC based on the Coulomb counting method.
* **`kalman_bms_state`**: Contains the estimated SoC and SoH based on the Extended Kalman Filter (EKF).

This allows users to compare the results obtained from these two different estimation techniques.

> **_NOTE:_** Coulomb counting method is using EKF predicted state after 10min as an initial value and corrects itself every hour with new estimated by EKF battery state. 

## ROS2 Topics

### Subscribed Topics

* **`/my_topic_1`**: 
    * QOS - ``rclpy.qos.qos_profile_sensor_data``
    * Message type - ``clearpath_platform_msgs.msg.Power``

### Published Topics

* **`/coulomb_bms_state`**:
    * QOS - ``rclpy.qos.qos_profile_sensor_data``
    * Message type - ``sensor_msgs.msg.BatteryState``

* **`/kalman_bms_state`**:
    * QOS - ``rclpy.qos.qos_profile_sensor_data``
    * Message type - ``sensor_msgs.msg.BatteryState``

## Estimation Methods

* **Coulomb Counting**: This method estimates the SoC by integrating the current flowing into or out of the battery over time. It provides a relatively simple way to track the charge but can be susceptible to cumulative errors and requires accurate initial SoC knowledge. In current implementation this mathod only used to estimate SoC value.

* **Extended Kalman Filter (EKF)**: The EKF is a powerful state estimation algorithm that can provide more robust SoC and SoH estimates by incorporating a dynamic model of the battery and noisy measurements. It uses a prediction-correction cycle to filter out noise and account for system uncertainties. In this project I used two EKFs, one to determine battery model state (SoC value) and the other for battery model parameters.

## Getting Started

### Prerequisites

* ROS 2 (Jazzy recommended)
* Colcon build tool
* Additional python dependencies are listed in [reqirenments.txt](battery_monitor/reqirenments.txt)

### Installation

1.  **Clone the repository into your ROS 2 workspace's `src` directory:**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/PHentosh/BMS_for_UGV.git
    ```

2.  **Build the package using Colcon:**
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3.  **Source the environment:**
    ```bash
    source install/setup.bash
    ```

## Usage

1.  **Run the battery monitoring system node:**
    ```bash
    ros2 launch battery_monitor bms_launch.py
    ```

3.  **Monitor the estimated SoC and SoH by subscribing to the output topics:**
    ```bash
    ros2 topic echo /coulomb_bms_state
    ros2 topic echo /kalman_bms_state
    ```

## Configuration

Battery configuration parameters are loaded from the `cfg/param.yaml` file. This file should contain the following parameters:

```yaml
battery:
  R0: <value>f     # Battery internal resistance (e.g., 0.05)
  Q0: <value>f     # Battery rated capacity (e.g., 5.0)
  Vmax: <value>f   # Fully charged voltage (e.g., 13.2)
  Vmin: <value> f  # Fully empty voltage (e.g., 10.8)

```

## System Architecture UML diagram

![System Architecture UML diagram](<./bms_architecture_vert.png>)

## System Architecture UML diagram

[MeRos diagrams](https://drive.google.com/drive/folders/1bGfbzHJYCIrVJPJDub3AFSESvCCbN18t?usp=sharing) for Odysseus Husky

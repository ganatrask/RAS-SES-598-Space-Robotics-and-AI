# **First-Order Boustrophedon Navigator**

## **Overview**
The **First-Order Boustrophedon Navigator** is a path-planning algorithm designed for complete area coverage. The system follows a structured **back-and-forth** pattern (similar to how a farmer plows a field) while minimizing cross-track error and optimizing velocity profiles. 

By fine-tuning **Proportional-Derivative (PD) controllers**, the system achieves smooth, stable motion while maintaining coverage efficiency. A **custom ROS2 message package** is integrated to monitor performance metrics in real-time.

---

## **Final Optimized Parameters**
After extensive testing and tuning, the following control parameters were determined to be the most effective for achieving optimal coverage and motion stability.

| Parameter | Value | Description |
|-----------|------|------------------------------------------------|
| **Kp_linear** | 8.0 | Controls the strength of forward motion correction. |
| **Kd_linear** | 0.5 | Dampens rapid linear velocity changes to prevent overshooting. |
| **Kp_angular** | 10.0 | Controls the robot's turning responsiveness. |
| **Kd_angular** | 0.01 | Ensures smooth corner transitions without excessive oscillations. |
| **Spacing** | 0.5 | Defines the distance between adjacent paths to ensure full coverage. |

These values were selected based on their ability to **minimize cross-track error, enhance trajectory stability, and optimize coverage quality**.

---

## **Parameter Testing and Results**
| **Kp_linear** | **Kd_linear** | **Kp_angular** | **Kd_angular** | **Avg. Error** |
|--------------|--------------|--------------|--------------|--------------|
| 10 | 0.1 | 0.1 | 0.2 | 0.296 |
| 5  | 1.0 | 0.1 | 10.0 | 0.292 |
| 10 | 1.0 | 0.1 | 10.0 | 0.274 |
| 10 | 2.0 | 0.2 | 10.0 | 0.310 |
| 10 | 2.0 | 0.1 | 10.0 | 0.301 |
| 10 | 0.05 | 0.1 | 10.0 | 0.312 |
| 10 | 0.1 | 0.1 | 5.0 | 0.298 |
| 10 | 2.0 | 0.1 | 5.0 | 0.290 |
| 10 | 2.0 | 0.2 | 5.0 | 0.274 |
| 5.0 | 2.0 | 0.2 | 5.0 | 0.294 |
| 12 | 6.0 | 0.3 | 6.0 | 0.288 |
| 15.0 | 6.0 | 0.3 | 7.0 | 0.266 |
| 15.0 | 6.0 | 0.3 | 6.0 | 0.309 |

---

![image](https://github.com/ganatrask/RAS-SES-598-Space-Robotics-and-AI/blob/main/assignments/first_order_boustrophedon_navigator/assets/image.png)


## **Custom ROS2 Message for Performance Monitoring**
A **custom ROS2 message package (`parameter_interfaces`)** was developed to provide real-time monitoring of the robot's performance metrics. This allows for continuous tracking of the **cross-track error, velocity profiles, and overall navigation efficiency**.

### **ROS2 Message Package: `parameter_interfaces`**
This package defines a custom message type to log key performance parameters.

### **Message Definition (`ParametersMsg.msg`)**
The `ParametersMsg.msg` file defines the key metrics monitored during the robot’s operation:
```
# Performance metrics for boustrophedon navigation
float64 cross_track_error          # Lateral deviation from the intended path
float64 linear_velocity            # Current forward speed
float64 angular_velocity           # Current rotational speed
float64 distance_to_next_waypoint  # Distance remaining to next target point
float64 completion_percentage      # Progress of navigation as a percentage
```

### **How the ROS2 Message Works**
1. The robot computes **real-time performance metrics** during navigation.
2. These metrics are **published as a ROS2 topic (`/parameters`)**.
3. Users can **monitor the robot’s performance** by subscribing to this topic.

### **Command to Monitor Performance in Real-Time**
```bash
ros2 topic echo /parameters
```
This command displays **live updates** of cross-track error, velocity changes, and completion percentage.

### **Integration with Navigation System**
The **BoustrophedonController** class continuously **publishes performance data** while navigating:

1. **Computes key parameters** (cross-track error, velocity, angular error).
2. **Publishes the metrics using `ParametersMsg.msg`**.
3. **Enables real-time monitoring and logging**.

---
## **Performance Metrics Summary**
- **Cross-Track Error Analysis**
  - **Average Cross-Track Error:** **0.06**
  - Error remained consistently **below the 0.2 threshold**.

- **Coverage Quality**
  - **Spacing:** 0.5 units ensured **full coverage without gaps**.
  - **Precise turns and smooth motion** maintained consistency.

---
## **Challenges & Solutions**
### **1. Parameter Tuning**
- **Problem:** Finding the right balance between control gains was difficult.
- **Solution:** Systematic **tuning and real-time monitoring** with `rqt_reconfigure`.

### **2. System Stability**
- **Problem:** Early tests showed **oscillations** in angular velocity.
- **Solution:** Optimized **Kd_angular (0.01)** to smooth transitions.

---
## **Lessons Learned**
- **Fine-tuning PD gains** is essential for stability.
- **Custom ROS2 messages enable real-time debugging.**
- **Precise spacing control ensures complete area coverage.**




![2024-11-19 21-40-52 的屏幕截图](https://github.com/user-attachments/assets/0a31a672-6cf0-4bed-957f-a5e6c50b889c)



This package demonstrates ESDF-based path planning for both **drones** and **cars** using ROS.


## Build Instructions

Make sure your workspace is properly set up. Then build the workspace using:

```bash
cd your_ws/
git clone https://github.com/9woods123/esdf_map_generater.git -b path_planning_demo
catkin_make
```

## Launch Instructions

### 🛸 Drone Example

To launch the path planning demo for a **drone**, use:

```bash
roslaunch esdf_map_generator path_planning_drone_demo.launch
```

### 🚗 Car Example

To launch the path planning demo for a **car**, use:

```bash
roslaunch esdf_map_generator path_planning_car_demo.launch
```

> Note: Ensure that map parameters and vehicle configurations are set appropriately in the corresponding launch files.

## Notes

- This project uses an ESDF map for global path planning.
- Both drones and cars rely on the same backend planner with vehicle-specific dynamics or constraints.


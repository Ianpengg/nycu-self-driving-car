## How to run
Terminal1 :  
```bash
roslaunch robot_localization ekf_template.launch
```
or  
```bash
roslaunch robot_localization ukf_template.launch
```
## Set the parameters
- ros1_noetic_ws/src/robot_localization/params
  - `ekf_template.yaml` and `ukf_template.yaml`
- ros1_noetic_ws/src/robot_localization/launch  
Change `yourpath` to where you store the bag and rviz config.
  - `<node pkg= "rviz" type="rviz" name="rviz" args="-d yourpath/sdc_hw3.rviz"/>`  
  - `<node pkg= "rosbag" type="play" name="rosbagplay" args="yourpath/sdc_hw3_noetic.bag --clock"/>` 
### Radar only:  
- comment the `pose0` part
### GPS only:
- comment the `odom0` part 

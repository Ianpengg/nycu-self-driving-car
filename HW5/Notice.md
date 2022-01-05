# Record my progress when doing this homework

## Problems
---
### Package Needed
- filterpy
$pip install filterpy
- scipy  
$pip install scipy==1.5.2
- scikit-learn 0.19.2  (this version works)
$pip install scikit-learn==0.19.2

### catkin_make argo_tracking
- fatal error: json/json.h: No such file or directory  
### solution :
sudo apt-get install libjsoncpp-dev 
sudo ln -s /usr/include/jsoncpp/json/ /usr/include/json

## Run run_ab3dmot.py
```
python run_ab3dmot.py --split test --dets_dataroot /home/ee904/argoverse_cbgs_kf_tracker/detections_v1.1b/argoverse_detections_2020 --raw_data_dir /home/ee904/Downloads/tracking_test_v1.1/argoverse-tracking/test
```
## Eval run_ab3dmot.py using validation data
```
python run_ab3dmot.py --split val --dets_dataroot /home/ee904/argoverse_cbgs_kf_tracker/detections_v1.1b/argoverse_detections_2020 --raw_data_dir /home/ee904/Downloads/tracking_val_v1.1/argoverse-tracking/val
```


## argo_tracking
Link: https://drive.google.com/file/d/1ArJpFNwtsepa2_CYwWEUjXoB9UIqbisS/view

## catkin_make

error : `usr/bin/ld: viz_track_result.cpp:(.text+0xfbe): undefined reference to `Json::Value::asString[abi:cxx11]() const'`  

Solution:
- Add following to CMakelist
```
set(lib_DIR /usr/lib/x84_64-linux-gnu)
link_directories(${lib_DIR})
target_link_libraries(viz_track_result ${catkin_LIBRARIES} libjsoncpp.a)
```




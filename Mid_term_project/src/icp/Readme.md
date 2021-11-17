# How to use
First `catkin_make` to complie the package

Second run the 3 ICP_localization nodes with launch file
```
$roslaunch icp ICP_localization_*.launch

```
The results .csv file would store in the results folder.

## Additional
You can run the GPU acceleration function through uncomment the marked code in CmakeList file.
Carefully choose the CUDA ARCH below (in CMakeList.txt) You can search it with Google.
```
set(CUDA_ARCH "sm_61") //for GTX 1080 Graphic Card
```
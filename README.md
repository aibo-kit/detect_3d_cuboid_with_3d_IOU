# Description #
This package is based on cube-salm and adds the 3d IOU module, which improves the accuracy of the 3d bounding box detection and verifies the accuracy of cube-slam detection model.
This package contains to single image cuboid detection in C++. Given 2D object detection, it generates many 3D cuboid proposal and selects the best one. It matches a [matlab implementation](https://github.com/shichaoy/matlab_cuboid_detect). Due to different canny edge and distancen transform, the final output might be slightly differently. For understanding and debuging purpose, it is suggested to use matlab implementation.

Modified by Aibo Wang

# How to run.
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./detect_3d_cuboid_node ..

then you can see the result of 3d Bounding box detection
See the main file ```src/main.cpp``` for more details.

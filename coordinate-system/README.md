## Conformal Coordinate Transformation

Creating transformation matrix based on collecting arbitrary number of points coordinated in two different coordinate system. <br/>

Collect some points in both coordinate systems: $pt1, pt2, ..., ptN$ <br/>
<br/>
**ptsA:** points `Eigen::Vector3d` in the first coordinate system ($A$), `ptsA: std::vector<Eigen::Vector3d>` <br/>
**ptsB:** points `Eigen::Vector3d` in the first coordinate system ($B$), `ptsB: std::vector<Eigen::Vector3d>` <br/>
<br/>
The code calculates transformation, rotation angles, and the overall error (MSE): <br/>
$B = T * A$ <br/>
<br/>
To use this package, add Eigen library's path to the makefile and create `coordinateTransform(ptsA, ptsB)` object. An example added to the main.cpp file. <br/>

Based on this paper: [Automatic Calculation of a Transformation Matrix Between Two Frames](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8271986&tag=1)
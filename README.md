# Laboratory of Applied Robotics Student Interface

## Loading and saving image

```
void loadImage(cv::Mat& img_out, const std::string& config_folder)
```
```
void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder)
```

## Extrinsic calibration

Calibration of the camera, by choosing 4 external points and performing an extrinsic calibration

```
void mouseCallback(int event, int x, int y, int, void* p)
```
```
std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
```
```
bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder)
```

OpenCV functions:
```
cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec)
```
> Finds an object pose from 3D-2D point correspondences. This function returns the rotation and the translation vectors that transform a 3D point expressed in the object coordinate frame to the camera coordinate frame.

## Undistort, Unwarp and Plane Transform

Methods used for undistorting and unwarping the image in order to correct lens distortion

```
void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out,
const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder)
```
```
void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec,
const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane,
const std::vector<cv::Point2f>& dest_image_points_plane,
cv::Mat& plane_transf, const std::string& config_folder)
```
```
void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
const std::string& config_folder)
```

OpenCV functions:

```
cv::undistort(img_in, img_out, cam_matrix, dist_coeffs)
```
> Transforms an image to compensate for lens distortion.

```
cv::getPerspectiveTransform(image_points, dest_image_points_plane)
```
> Calculates a perspective transform from four pairs of the corresponding points.

```
cv::warpPerspective(img_in, img_out, transf, img_in.size())
```
> Applies a perspective transformation to an image.

## Obstracle-victim-gate-robot detection / Arena map generation

Detection of obstacles and victims in the map and calculation of robot position and orientation

```
bool processVictimsGate(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate)
```
```
bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list)
```
```
bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder)
```
```
bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder)
```

  > Detecting green victims and gate

![Victim-gate flow](images/gate_victim.jpg)


  > Detecting red obstracles

![Obstracle flow](images/obstracle_list.png)

  > Detecting blue robot, and its 'yaw'

![Robot flow](images/robot.jpg)


## Victim rank detection ( digit recognition )

### Using template matching

### Tesseract-OCR

# kinect_azure_programming

### record rosbag

```bash
# record all the topics（not recommended, the data volume is large）
rosbag record -O xxx.bag -a

# record the specified topic
rosbag record -O xxx.bag /depth_to_rgb/image_raw /rgb/image_raw
```

### parse rosbag
1. extract the timestamp, rgb image & depth image from the rosbag file, save the info into .txt file

- first, run the command 
```bash
rosrun save_rgbd_from_k4a save_rgbd_from_k4a
```
- then, run the rosbag play command
```bash
# -r: specify the play speed
rosbag play xxx.bag -r 0.5
```

Note that converting CV_16UC1 to CV_32FC1 requires to divide by 1000 to keep the accuracy.

https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/src/k4a_ros_device.cpp

```cpp
k4a_result_t K4AROSDevice::renderDepthToROS(sensor_msgs::ImagePtr& depth_image, k4a::image& k4a_depth_frame)
{
    cv::Mat depth_frame_buffer_mat(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_16UC1, k4a_depth_frame.get_buffer());
    cv::Mat new_image(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_32FC1);

    depth_frame_buffer_mat.convertTo(new_image, CV_32FC1, 1.0 / 1000.0f);

    depth_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, new_image).toImageMsg();

  return K4A_RESULT_SUCCEEDED;
}
```

2. associate the rgb image and depth image according to the timestamp, generate the associate.txt file, find the corresponding rgb & depth, rename and copy them into the **matched** folder 
```bash
python3 associate.py
```
Note: this is modified according to TUM RGB-D Benchmark [associate.py](https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/associate.py)
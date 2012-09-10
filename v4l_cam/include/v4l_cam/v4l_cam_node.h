/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) <year>  <name of author>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef V4R_CAM_NODE_H
#define V4R_CAM_NODE_H

#include <ros/ros.h>
#include <v4l_cam/v4l_cam.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <v4l_cam/CameraParametersConfig.h>

	
class V4RCamNode : public V4LCam
{
public:
  static const int CONVERT_RAW = 0;
  static const int CONVERT_YUV422toRGB = 1;
  static const int CONVERT_YUV422toBGR = 2;
  static const int CONVERT_YUV422toGray = 3;
  
    V4RCamNode(ros::NodeHandle &n);
    ~V4RCamNode();
    void publishCamera();
    void processQueue();
	void callbackParameters(v4l_cam::CameraParametersConfig &config, uint32_t level);
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    image_transport::ImageTransport  imageTransport_;
	image_transport::CameraPublisher cameraPublisher_;
    dynamic_reconfigure::Server<v4l_cam::CameraParametersConfig> reconfigureServer_;
    dynamic_reconfigure::Server<v4l_cam::CameraParametersConfig>::CallbackType reconfigureFnc_;
    sensor_msgs::CameraInfo cameraInfo_;
    sensor_msgs::Image cameraImage_;
	bool generate_dynamic_reconfigure_;
	bool show_camera_image_;
    boost::thread showCameraImageThread_;
private:
    void readInitParams();   
    void readV4lParams();     
    void writeV4lParams();     
    void loopCamera();
    void showCameraImage();
	std::string queueLoadConfiguration_;
	std::string queueSaveConfiguration_;
	int convert_image_first_;
	
    /**
     * reads and updates all local control values
     * @param controls control parameters
     **/
    void readCameraControls();
    /**
     * writes does parameter which are different to the current ones to the camera
     * @param control control parameters
     **/
    void writeCameraControls();
	
    /**
     * reads a control entr
     * @param entry control entry
     **/
	void readControlEntryInfo(ControlEntry *entry);
    /**
     * updates a control entr
     * @param entry control entry
     **/
	void updateControlEntry(ControlEntry *entry);
    /**
     * generates a new reconfigure file
     **/
    void updateDynamicReconfigureFile(const char* filename) const;
};

#endif // V4R_CAM_NODE_H

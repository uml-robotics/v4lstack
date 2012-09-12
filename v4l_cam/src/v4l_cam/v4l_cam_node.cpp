/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2012  Markus Bader

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

#include <camera_info_manager/camera_info_manager.h>
#include <v4l_cam/v4l_cam_defaults.h>
#include <v4l_cam/v4l_cam_node.h>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include "luvcview/v4l2uvc.h"

typedef boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> Lock;

extern "C" {
    unsigned int Pyuv422torgb24(unsigned char *input_ptr, unsigned char *output_ptr, unsigned int image_width, unsigned int image_height);
    unsigned int Pyuv422tobgr24(unsigned char *input_ptr, unsigned char *output_ptr, unsigned int image_width, unsigned int image_height);
    unsigned int Pyuv422togray8(unsigned char *input_ptr, unsigned char *output_ptr, unsigned int image_width, unsigned int image_height);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "v4l_cam");
    ros::NodeHandle n;
    V4RCamNode v4r_cam(n);
    ros::Rate rate(100);
    while(ros::ok() && v4r_cam.grab()) {
        v4r_cam.processQueue();
        v4r_cam.publishCamera();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void V4RCamNode::processQueue()
{
    if(!queueLoadConfiguration_.empty()) {
        load_controls(queueLoadConfiguration_);
        if(hasInfoMsg()) ROS_INFO_STREAM(pullInfoMsg());
        if(hasErrorMsg()) ROS_ERROR_STREAM(pullErrorMsg());
        writeV4lParams();
        queueLoadConfiguration_ = std::string();
    } else if(!queueSaveConfiguration_.empty()) {
        save_controls(queueSaveConfiguration_);
        if(hasInfoMsg()) ROS_INFO_STREAM(pullInfoMsg());
        if(hasErrorMsg()) ROS_ERROR_STREAM(pullErrorMsg());
        queueLoadConfiguration_ = std::string();
    }
}

void V4RCamNode::callbackParameters(v4l_cam::CameraParametersConfig &config, uint32_t level)
{
    readV4lParams();
    if(show_camera_image_ != config.show_camera_image) {
        if(config.show_camera_image) {
            showCameraImageThread_ = boost::thread(&V4RCamNode::showCameraImage, this);
            ROS_INFO("start show camera image thread");
        }
        show_camera_image_ = config.show_camera_image;
    }
    if(camera_control_action_ != config.camera_control_action) {
        if(config.camera_control_action == LOAD) {
            queueLoadConfiguration_ = config.camera_parameters_file;
        }
        if(config.camera_control_action == SAVE) {
            queueSaveConfiguration_ = config.camera_parameters_file;
        }
        camera_control_action_ = config.camera_control_action;
    }
    convert_image_first_ = config.convert_image_first;
}

V4RCamNode::~V4RCamNode()
{
    show_camera_image_ = false;
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

V4RCamNode::V4RCamNode(ros::NodeHandle &n)
    : n_(n), n_param_("~"), imageTransport_(n_param_), generate_dynamic_reconfigure_(false), show_camera_image_(false), queueLoadConfiguration_(""), queueSaveConfiguration_("")
{
    cameraPublisher_ = imageTransport_.advertiseCamera("image_raw", 1);
    readInitParams();
    readV4lParams();
    initCamera();
    detectControlEnties();
    for(unsigned int  i = 0; i  < controlEntries_.size(); i++) {
        ROS_INFO_STREAM(controlEntries_[i]->getQueryCtrlInfo());
    }
    reconfigureFnc_ = boost::bind(&V4RCamNode::callbackParameters, this,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
    readV4lParams();
    if(show_camera_image_) {
        showCameraImageThread_ = boost::thread(&V4RCamNode::showCameraImage, this);
    }
}

void V4RCamNode::readV4lParams()
{
    for(unsigned int  i = 0; i < controlEntries_.size(); i++) {
        if(controlEntries_[i]->queryctrl->type == V4L2_CTRL_TYPE_BOOLEAN) {
            bool tmp;
            n_param_.param<bool>(controlEntries_[i]->varName, tmp, (bool) controlEntries_[i]->queryctrl->default_value);
            controlEntries_[i]->targetValue = tmp;
        } else {
            n_param_.param<int>(controlEntries_[i]->varName, controlEntries_[i]->targetValue, controlEntries_[i]->queryctrl->default_value);
        }
        v4lset(controlEntries_[i]);
        if(controlEntries_[i]->hasErrorMsg()) ROS_ERROR_STREAM(controlEntries_[i]->varName << ": " << controlEntries_[i]->pullErrorMsg());
        if(controlEntries_[i]->hasInfoMsg()) ROS_INFO_STREAM(controlEntries_[i]->varName << ": " << controlEntries_[i]->pullInfoMsg());
    }
}
void V4RCamNode::writeV4lParams()
{
    for(unsigned int  i = 0; i < controlEntries_.size(); i++) {
        v4lget(controlEntries_[i]);
        if(controlEntries_[i]->queryctrl->type == V4L2_CTRL_TYPE_BOOLEAN) {
            n_param_.setParam(controlEntries_[i]->varName, (bool) controlEntries_[i]->currentValue);
            std::cout << controlEntries_[i]->varName << ": bool " << controlEntries_[i]->currentValue << std::endl;
        } else {
            n_param_.setParam(controlEntries_[i]->varName, (int) controlEntries_[i]->currentValue);
            std::cout << controlEntries_[i]->varName << ": int  " << controlEntries_[i]->currentValue << std::endl;
        }
        if(controlEntries_[i]->hasErrorMsg()) ROS_ERROR_STREAM(controlEntries_[i]->varName << ": " << controlEntries_[i]->pullErrorMsg());
        if(controlEntries_[i]->hasInfoMsg()) ROS_INFO_STREAM(controlEntries_[i]->varName << ": " << controlEntries_[i]->pullInfoMsg());
    }
}
void V4RCamNode::readInitParams()
{
    ROS_INFO("V4RCamNode::readParams()");
    n_param_.param<bool>("show_camera_image", show_camera_image_, DEFAULT_SHOW_CAMERA_IMAGE);
    ROS_INFO("show_camera_image: %s", ((show_camera_image_) ? "true" : "false"));
    n_param_.param<std::string>("frame_id", cameraInfo_.header.frame_id, DEFAULT_FRAME_ID);
    ROS_INFO("frame_id: %s", cameraInfo_.header.frame_id.c_str());
    n_param_.param<std::string>("video_device", videoDevice_, DEFAULT_VIDEODEVICE);
    ROS_INFO("video_device: %s", videoDevice_.c_str());
    n_param_.param<std::string>("avi_filename", aviFilename_, DEFAULT_AVIFILENAME);
    ROS_INFO("avi_filename: %s", aviFilename_.c_str());
    n_param_.param<std::string>("camera_parameters_file", queueLoadConfiguration_, "");
    ROS_INFO("camera_parameters_file: %s", queueLoadConfiguration_.c_str());
    n_param_.param<int>("convert_image_first", convert_image_first_, DEFAULT_CONVERT_IMAGE_FIRST);
    ROS_INFO("convert_image_first: %i", convert_image_first_);

    std::string camera_info_url;
    if(n_param_.getParam("camera_info_url", camera_info_url)) {
        camera_info_manager::CameraInfoManager cinfo(n_param_);
        if(cinfo.validateURL(camera_info_url)) {
            cinfo.loadCameraInfo(camera_info_url);
            cameraInfo_ = cinfo.getCameraInfo();
        } else {
            ROS_FATAL("camera_info_url not valid.");
            n_param_.shutdown();
            return;
        }
    } else {
        XmlRpc::XmlRpcValue double_list;
        n_param_.getParam("K", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 9)) {
            for(int i = 0; i < 9; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.K[i] = double_list[i];
            }
        }

        n_param_.getParam("D", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 5)) {
            for(int i = 0; i < 5; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.D[i] = double_list[i];
            }
        }

        n_param_.getParam("R", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 9)) {
            for(int i = 0; i < 9; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.R[i] = double_list[i];
            }
        }

        n_param_.getParam("P", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 12)) {
            for(int i = 0; i < 12; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.P[i] = double_list[i];
            }
        }
    }
    ROS_INFO("tf_camera_id: %s", cameraInfo_.header.frame_id.c_str());
}

void V4RCamNode::publishCamera()
{
    cameraInfo_.header.stamp.sec = timeLastFrame_.tv_sec, cameraInfo_.header.stamp.nsec = timeLastFrame_.tv_usec * 1000;
    cameraImage_.header = cameraInfo_.header;
    cameraImage_.height = cameraInfo_.height = height_;
    cameraImage_.width = cameraInfo_.width = width_;
    cameraImage_.is_bigendian = true;
    cameraImage_.step = cameraInfo_.width * 2;
    switch(convert_image_first_) {
    case CONVERT_RAW:
        cameraImage_.encoding = "yuv422";
        cameraImage_.data.resize(width_ * height_ * 2);
        memcpy(&cameraImage_.data[0], pVideoIn_->framebuffer, cameraImage_.data.size());
        break;
    case CONVERT_YUV422toRGB:
        cameraImage_.encoding = "rgb8";
        cameraImage_.data.resize(width_ * height_ * 3);
        Pyuv422torgb24(pVideoIn_->framebuffer, &cameraImage_.data[0], width_, height_);
        break;
    case CONVERT_YUV422toBGR:
        cameraImage_.encoding = "bgr8";
        cameraImage_.data.resize(width_ * height_ * 3);
        Pyuv422tobgr24(pVideoIn_->framebuffer, &cameraImage_.data[0], width_, height_);
        break;
    case CONVERT_YUV422toGray:
        cameraImage_.encoding = "mono8";
        cameraImage_.data.resize(width_ * height_);
        Pyuv422togray8(pVideoIn_->framebuffer, &cameraImage_.data[0], width_, height_);
        break;
    default:
        cameraImage_.encoding = "yuv422";
        cameraImage_.data.resize(width_ * height_ * 2);
        memcpy(&cameraImage_.data[0], pVideoIn_->framebuffer, cameraImage_.data.size());  
    }
    cameraPublisher_.publish(cameraImage_, cameraInfo_);
}


void V4RCamNode::showCameraImage()
{
    timeval lastShownFrame = timeLastFrame_;
    double lastDuration = durationLastFrame_;
    cv::Mat img;
    int key = -1;
    do {
        if((lastShownFrame.tv_sec != timeLastFrame_.tv_sec) || (lastShownFrame.tv_usec != timeLastFrame_.tv_usec)) {
            img.create(height_, width_, CV_8UC3);
            lastShownFrame = timeLastFrame_;
            std::stringstream ss;
            ss << 1000.0 / ((durationLastFrame_ + lastDuration) / 2.0);
            lastDuration = durationLastFrame_;
            Lock myLock(mutexImage_);
            Pyuv422tobgr24(pVideoIn_->framebuffer, img.data, width_, height_);
            cv::putText(img, ss.str(), cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0));
            cv::imshow("Camera", img);
        }
        key =  cv::waitKey(50);
    } while((key == -1) && show_camera_image_);
    cv::destroyWindow("Camera");
    cv::waitKey(10);
    if(key != -1) {
        pVideoIn_->signalquit = 0;
    }
}

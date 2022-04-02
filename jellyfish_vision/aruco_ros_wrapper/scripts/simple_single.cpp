/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
/**
* @file simple_single.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <mutex>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros_wrapper/aruco_ros_utils.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros_wrapper/ArucoThresholdConfig.h>
#include <aruco_ros_wrapper/PixelToPosition.h>

#include <iostream>

using namespace aruco;

class ArucoSimple {
 private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  aruco::MarkerDetector::Params mParams;
  geometry_msgs::Transform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  std::map<int, MarkerPoseTracker> mTracker;
  std::vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub;
  ros::Publisher position_pub;
  ros::Publisher marker_pub; //rviz visualization marker
  ros::Publisher pixel_pub;
  ros::ServiceServer pixels_server;
  std::string marker_frame;
  std::string camera_frame;

  std::mutex marker_mutex;
  double marker_size;
  std::string dictionary;
  int marker_id;

  bool rotate_marker_axis_;
  bool publish_tf_;
  bool use_pose_tracker_;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  tf2_ros::TransformBroadcaster br;

  dynamic_reconfigure::Server<aruco_ros_wrapper::ArucoThresholdConfig> dyn_rec_server;

 public:
  ArucoSimple()
      : cam_info_received(false),
        nh("~"),
        it(nh),
        _tfListener(_tfBuffer) {
    
    //set mDetector parameters
    std::string refinementMethod;
    nh.param<std::string>("corner_refinement", refinementMethod, "CORNER_LINES");
    mParams.getCornerRefinementMethodFromString(refinementMethod);

    ROS_INFO_STREAM("Corner refinement method: " << mParams.cornerRefinementM);
    ROS_INFO_STREAM("Threshold method: " << mParams.thresMethod);
    
    mDetector.setParameters(mParams);

    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
    pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);

    pixels_server = nh.advertiseService("pixel_to_position",
                                        &ArucoSimple::pixelToPositionCallback, this);

    nh.param<double>("marker_size", marker_size, 0.041);
    nh.param<std::string>("dictionary", dictionary, "ARUCO_MIP_36h12");
    nh.param<int>("marker_id", marker_id, 0);
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "aruco_marker");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);
    nh.param<bool>("rotate_marker_axis", rotate_marker_axis_, false);
    nh.param<bool>("publish_tf", publish_tf_, false);
    nh.param<bool>("use_pose_tracker", use_pose_tracker_, true);

    mDetector.setDictionary(dictionary);

    ROS_ASSERT(!camera_frame.empty() && !marker_frame.empty());

    ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d",
             marker_size, marker_id);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             camera_frame.c_str(), marker_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
  }

  void image_callback(const sensor_msgs::ImageConstPtr &msg) {
    if ((image_pub.getNumSubscribers() == 0) &&
        (debug_pub.getNumSubscribers() == 0) &&
        (pose_pub.getNumSubscribers() == 0) &&
        (transform_pub.getNumSubscribers() == 0) &&
        (position_pub.getNumSubscribers() == 0) &&
        (marker_pub.getNumSubscribers() == 0) &&
        (pixel_pub.getNumSubscribers() == 0)) {
      ROS_DEBUG("No subscribers, not looking for aruco markers");
      return;
    }

    if (cam_info_received) {
      ros::Time curr_stamp(ros::Time::now());
      cv_bridge::CvImagePtr cv_ptr;

      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        // detection results will go into "markers"
        {
          std::lock_guard<std::mutex> lock(marker_mutex);
          markers = mDetector.detect(inImage, camParam, marker_size);
        }

        if (use_pose_tracker_) {
          for (auto &marker : markers) {
            mTracker[marker.id].estimatePose(marker, camParam, marker_size);
          }
        }

        bool success = false;

        //for each marker, draw info and its boundaries in the image
        for (size_t i = 0; i < markers.size(); ++i) {
          // only publishing the selected marker
          if (markers[i].id == marker_id) {
            success = true;
            markers[i].calculateExtrinsics(marker_size, camParam, true);
            tf2::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);
            
            geometry_msgs::TransformStamped stampedTransform;
            stampedTransform.header.stamp = curr_stamp;
            stampedTransform.header.frame_id = camera_frame;
            stampedTransform.child_frame_id = marker_frame; 
            stampedTransform.transform = tf2::toMsg(transform);
            
            if (publish_tf_) 
              br.sendTransform(stampedTransform);
            
              
            //publish for easier debugging
            transform_pub.publish(stampedTransform);
            
            geometry_msgs::PoseStamped poseMsg;
            tf2::convert(stampedTransform, poseMsg);
            pose_pub.publish(poseMsg);
            
            geometry_msgs::Vector3Stamped positionMsg;
            positionMsg.header = stampedTransform.header;
            positionMsg.vector = stampedTransform.transform.translation;
            position_pub.publish(positionMsg);
            
            geometry_msgs::PointStamped pixelMsg;
            pixelMsg.header = stampedTransform.header;
            pixelMsg.point.x = markers[i].getCenter().x;
            pixelMsg.point.y = markers[i].getCenter().y;
            pixelMsg.point.z = 0;
            pixel_pub.publish(pixelMsg);
            
            
            //Publish rviz marker representing the ArUco marker patch
            visualization_msgs::Marker visMarker;
            visMarker.header = stampedTransform.header;
            visMarker.id = 1;
            visMarker.type = visualization_msgs::Marker::CUBE;
            visMarker.action = visualization_msgs::Marker::ADD;
            visMarker.pose = poseMsg.pose;
            visMarker.scale.x = marker_size;
            visMarker.scale.y = 0.001;
            visMarker.scale.z = marker_size;
            visMarker.color.r = 1.0;
            visMarker.color.g = 0;
            visMarker.color.b = 0;
            visMarker.color.a = 1.0;
            visMarker.lifetime = ros::Duration(3.0);
            marker_pub.publish(visMarker);
          }

          // but drawing all the detected markers
          markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
        }

        if (!success) {
          ROS_WARN("Marker not detected in current frame!");
        } else {
          ROS_INFO("Marker detected");
        }

        //draw a 3d cube in each marker if there is 3d info
        if (camParam.isValid() && marker_size > 0) {
          for (size_t i = 0; i < markers.size(); ++i) {
            CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        if (image_pub.getNumSubscribers() > 0) {
          //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }
        else{
          ROS_DEBUG("No subscribers, not gna give any images");
        }

        if (debug_pub.getNumSubscribers() > 0) {
          //show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg) {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.translation.x = -msg.P[3] / msg.P[0];
    rightToLeft.translation.y = -msg.P[7] / msg.P[5];
    rightToLeft.translation.z = 0.0;

    cam_info_received = true;
    cam_info_sub.shutdown();
    if (camParam.isValid()) {
      ROS_INFO("Received camera info");
    } else {
      ROS_WARN("INVALID CAMERA INFO");
    }
    
  }

  // shifts the last found marker's center to the pixel requested
  // and calculate extrinsics
  bool pixelToPositionCallback(
      aruco_ros_wrapper::PixelToPosition::Request &req,
      aruco_ros_wrapper::PixelToPosition::Response &resp) {
    std::lock_guard<std::mutex> lock(marker_mutex);
    if (markers.empty()) {
      return false;
    }
    for (auto & marker : markers) {
      if (marker.id == marker_id) {
        resp.positions.resize(req.pixels.size());
        Marker tmp_marker;
        for (size_t i = 0; i < req.pixels.size(); ++i) {
          marker.copyTo(tmp_marker);
          double diff_x = marker.getCenter().x - req.pixels[i].x;
          double diff_y = marker.getCenter().y - req.pixels[i].y;
          for (int idx = 0; idx < tmp_marker.size(); ++idx) {
            tmp_marker[idx].x -= diff_x;
            tmp_marker[idx].y -= diff_y;
          }
          tmp_marker.calculateExtrinsics(marker_size, camParam, false);
          if (tmp_marker.isPoseValid()) {
            tf2::Transform transform = aruco_ros::arucoMarker2Tf(tmp_marker, rotate_marker_axis_);
            resp.positions[i].x = transform.getOrigin().x();
            resp.positions[i].y = transform.getOrigin().y();
            resp.positions[i].z = transform.getOrigin().z();
          } else {
            return false;
          }
        }
        resp.header.stamp = ros::Time::now();
        resp.header.frame_id = camera_frame;
        return true;
      }
    }
    return false;
  }

  void reconf_callback(aruco_ros_wrapper::ArucoThresholdConfig &config, uint32_t level) {
    ROS_INFO("In dynamic reconfigure callback");
    mParams.setCornerRefinementMethod(static_cast<aruco::CornerRefinementMethod>(config.corner_refinement_method));
    mParams.setDetectionMode(static_cast<aruco::DetectionMode>(config.detection_mode), config.min_marker_size);
  }
  
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}

/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */

// PAL headers
#include <tiago_pcl_tutorial/pcl_filters.hpp>
#include <tiago_pcl_tutorial/geometry.h>
#include <tiago_pcl_tutorial/tf_transforms.hpp>

// PCL headers
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
// ROS headers
#include <ros/ros.h>

#include <ros/callback_queue.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

// Eigen headers
#include <Eigen/Core>

namespace pal {

class CylinderDetector {

public:

  CylinderDetector(ros::NodeHandle& nh,
                   ros::NodeHandle& pnh);

  virtual ~CylinderDetector();

  void run();

protected:

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
               const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
               const Eigen::Matrix4d& transform,
               const Eigen::Matrix4d& transform_1,
               double cylinderHeight,
               const std_msgs::Header& header);

  void publishPose(const geometry_msgs::Pose& pose,
                   const std_msgs::Header& header);
  void publishPose_1(const geometry_msgs::Pose& pose,
                   const std_msgs::Header& header);

  void print(pcl::ModelCoefficients::Ptr cylinderCoefficients_1, pcl::ModelCoefficients::Ptr cylinderCoefficients_2,
                             int numberOfPoints_1,int numberOfPoints_2,int numberOfPoints_1_f,int numberOfPoints_2_f);

  /**
   * @brief getPointAndVector Given a point cloud in which a cylinder has been fit, it computes
   *        the main axis of the cylinder and provides the centroid of the point cloud projected on the
   *        main axis and the main axis director vector
   * @param cylinderCloud
   * @param linePoint
   * @param lineVector
   */
  void getPointAndVector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                         const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                         Eigen::Vector3d& linePoint,
                         Eigen::Vector3d& lineVector);

  /**
   * @brief computeHeight calculate height of cylinder provided its point cloud and the main axis vector
   *        and the cylinder centroid.
   * @param cylinderCloud
   * @param centroid
   * @param mainAxis
   * @return
   */
  double computeHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                       const Eigen::Vector3d& centroid,
                       const Eigen::Vector3d& mainAxis);

  void projectPointToLine(const Eigen::Vector3d& linePoint,
                          const Eigen::Vector3d& lineVector,
                          const Eigen::Vector3d& pointToProject,
                          Eigen::Vector3d& projectedPoint);

  void start();
  void stop();

  ros::NodeHandle& _nh, _pnh;
  ros::CallbackQueue _cbQueue;
  bool _enabled;
  double _rate;

  // ROS interfaces
  ros::Subscriber _cloudSub;
  ros::Publisher  _cylinderCloudPub;
  ros::Publisher  _cylinderPosePub;
  ros::Publisher  _cylinderPosePub_1;
  ros::Publisher  _cylinderMarkerPub;
  ros::Publisher  _objectPub;
};


CylinderDetector::CylinderDetector(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh):
  _nh(nh),
  _pnh(pnh),
  _enabled(false),
  _rate(5.0)
{
  _nh.setCallbackQueue(&_cbQueue);

  pnh.param<double>("rate", _rate, _rate);

  _cylinderCloudPub  = _pnh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("cylinder_cloud", 1);
  _cylinderPosePub   = _pnh.advertise< geometry_msgs::PoseStamped >("cylinder_pose", 1);
  _cylinderPosePub_1   = _pnh.advertise< geometry_msgs::PoseStamped >("cylinder_pose_1", 1);
  _cylinderMarkerPub = _pnh.advertise<visualization_msgs::Marker>( "marker", 1 );
  _objectPub         = _pnh.advertise<object_recognition_msgs::RecognizedObjectArray>("recognized_objects",1);
}

CylinderDetector::~CylinderDetector()
{

}

void CylinderDetector::print(pcl::ModelCoefficients::Ptr cylinderCoefficients_1, pcl::ModelCoefficients::Ptr cylinderCoefficients_2,
                             int numberOfPoints_1,int numberOfPoints_2,int numberOfPoints_1_f,int numberOfPoints_2_f)
{
  std::stringstream ss;
  ss << std::endl << "Cylinder_1 found with " << numberOfPoints_1 << " points before filtered and  " << numberOfPoints_1_f <<" points after filtered:" << std::endl;
  ss << "\tRadius:      " << cylinderCoefficients_1->values[6] << " m" << std::endl;
  ss << "\tPoint:       (" <<
        cylinderCoefficients_1->values[0] << ", " <<
        cylinderCoefficients_1->values[1] << ", " <<
        cylinderCoefficients_1->values[2] << ")" << std::endl;
  ss << "\tAxis:        (" <<
        cylinderCoefficients_1->values[3] << ", " <<
        cylinderCoefficients_1->values[4] << ", " <<
        cylinderCoefficients_1->values[5] << ")" << std::endl;
  ss << std::endl << "Cylinder_2 found with " << numberOfPoints_2 << " points before filtered and  " << numberOfPoints_2_f <<" points after filtered:"<<std::endl;
  ss << "\tRadius:      " << cylinderCoefficients_2->values[6] << " m" << std::endl;
  ss << "\tPoint:       (" <<
        cylinderCoefficients_2->values[0] << ", " <<
        cylinderCoefficients_2->values[1] << ", " <<
        cylinderCoefficients_2->values[2] << ")" << std::endl;
  ss << "\tAxis:        (" <<
        cylinderCoefficients_2->values[3] << ", " <<
        cylinderCoefficients_2->values[4] << ", " <<
        cylinderCoefficients_2->values[5] << ")" << std::endl;
		
  ROS_INFO_STREAM(ss.str());
}

void CylinderDetector::projectPointToLine(const Eigen::Vector3d& linePoint,
                                          const Eigen::Vector3d& lineVector,
                                          const Eigen::Vector3d& pointToProject,
                                          Eigen::Vector3d& projectedPoint)
{
  Eigen::Vector3d pointToLineVector;
  pointToLineVector = pointToProject - linePoint;
  projectedPoint = linePoint + (pointToLineVector.dot(lineVector) / lineVector.dot(lineVector) ) * lineVector;
}

void CylinderDetector::getPointAndVector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                                         const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                                         Eigen::Vector3d& linePoint,
                                         Eigen::Vector3d& lineVector)
{
  //compute cylinder centroid
  Eigen::Vector4d centroid;
  centroid.setZero();
  pcl::compute3DCentroid<pcl::PointXYZRGB>(*cylinderCloud, centroid);

  linePoint(0,0) = cylinderCoefficients->values[0];
  linePoint(1,0) = cylinderCoefficients->values[1];
  linePoint(2,0) = cylinderCoefficients->values[2];

  lineVector(0,0) = cylinderCoefficients->values[3];
  lineVector(1,0) = cylinderCoefficients->values[4];
  lineVector(2,0) = cylinderCoefficients->values[5];

  Eigen::Vector3d projectedCentroid;

  projectPointToLine(linePoint, lineVector,
                     centroid.head<3>(),
                     projectedCentroid);

  linePoint = projectedCentroid;
}

double CylinderDetector::computeHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                                       const Eigen::Vector3d& centroid,
                                       const Eigen::Vector3d& mainAxis)
{
  double height = 0;

  Eigen::Vector3d projectedPoint;
  //project all points into the main axis of the cylinder
  for (unsigned int i = 0; i < cylinderCloud->points.size(); ++i)
  {
    Eigen::Vector3d point(cylinderCloud->points[i].x,
                          cylinderCloud->points[i].y,
                          cylinderCloud->points[i].z);
    //take the largest distance to the centroid as the height of the cylinder
    projectPointToLine(centroid, mainAxis, point, projectedPoint);

    double distance = sqrt( (projectedPoint(0,0) - centroid(0,0))*(projectedPoint(0,0) - centroid(0,0)) +
                            (projectedPoint(1,0) - centroid(1,0))*(projectedPoint(1,0) - centroid(1,0)) +
                            (projectedPoint(2,0) - centroid(2,0))*(projectedPoint(2,0) - centroid(2,0)) );

    if ( 2*distance > height )
      height = 2*distance;
  }

  return height;
}

void CylinderDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{  
  if ( (cloud->width * cloud->height) == 0)
    return;
  int x1=-1.8,x2=1.8,y1=-1.8,y2=1.8,z1=0,z2=10;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>), pclCloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PCDWriter writer_1,writer_2;
  pcl::fromROSMsg(*cloud, *pclCloud_1);
  //pcl::io::savePCDFileASCII("~/test_1.pcd",*pclCloud);
  //writer_1.write<pcl::PointXYZRGB>("~/test_1.pcd",*pclCloud,false);
  for(int i=0;i<(pclCloud_1->points.size());i++)
    {
			if ((pclCloud_1->points[i].x>x1)&&(pclCloud_1->points[i].x<x2))
				if((pclCloud_1->points[i].y>y1)&&(pclCloud_1->points[i].y<y2))
				    if((pclCloud_1->points[i].z>z1)&&(pclCloud_1->points[i].z<z2))
							pclCloud->points.push_back(pclCloud_1->points[i]);
	}
  int n_cloud_1,n_cloud_1_f,n_cloud_2,n_cloud_2_f;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCylinderCloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCylinderCloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr cylinderCoefficients_1(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr cylinderCoefficients_2(new pcl::ModelCoefficients);
  bool found = pal::cylinderSegmentation<pcl::PointXYZRGB>(pclCloud,
                                                           pclCylinderCloud_1,
                                                           pclCylinderCloud_2,
                                                           10,
                                                           0.015, 0.08,
                                                           cylinderCoefficients_1,
                                                           cylinderCoefficients_2);
  //pcl::io::savePCDFileASCII("~/test_2.pcd",*pclCylinderCloud);
  //filter outliers in the cylinder cloud
  n_cloud_1=(int)pclCylinderCloud_1->points.size();
  n_cloud_2=(int)pclCylinderCloud_2->points.size();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclFilteredCylinderCloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclFilteredCylinderCloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
  if ( (pclCylinderCloud_1->empty() )&&(pclCylinderCloud_2->empty() ))
	{
    pclFilteredCylinderCloud_1 = pclCylinderCloud_1;
    pclFilteredCylinderCloud_2 = pclCylinderCloud_2;
    }
  else if ( (!pclCylinderCloud_1->empty() )&&(pclCylinderCloud_2->empty() ))
	{
    pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclCylinderCloud_1, 25, 1.0,pclFilteredCylinderCloud_1);
    pclFilteredCylinderCloud_2 = pclCylinderCloud_2;
	}
  else if ( (pclCylinderCloud_1->empty() )&&(!pclCylinderCloud_2->empty() ))
	{
	pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclCylinderCloud_2, 25, 1.0,pclFilteredCylinderCloud_2);
    pclFilteredCylinderCloud_1 = pclCylinderCloud_1;
	}
  else
	{
	pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclCylinderCloud_1, 25, 1.0,pclFilteredCylinderCloud_1);
	pal::statisticalOutlierRemoval<pcl::PointXYZRGB>(pclCylinderCloud_2, 25, 1.0,pclFilteredCylinderCloud_2);
	}
	
	
  if ( found )
  {
    print(cylinderCoefficients_1,cylinderCoefficients_2, (int)pclCylinderCloud_1->points.size(),(int)pclCylinderCloud_2->points.size(),(int)pclFilteredCylinderCloud_1->points.size(),(int)pclFilteredCylinderCloud_2->points.size());
    //print(m);

    Eigen::Vector3d projectedCentroid_1,projectedCentroid_2, lineVector_1,lineVector_2;
    getPointAndVector(pclFilteredCylinderCloud_2,
                      cylinderCoefficients_2,
                      projectedCentroid_2,
                      lineVector_2);
    getPointAndVector(pclFilteredCylinderCloud_1,
                      cylinderCoefficients_1,
                      projectedCentroid_1,
                      lineVector_1);
;

    double cylinderHeight_2 = computeHeight(pclFilteredCylinderCloud_2,
                                          projectedCentroid_2,
                                          lineVector_2);
    double cylinderHeight_1 = computeHeight(pclFilteredCylinderCloud_1,
                                          projectedCentroid_1,
                                          lineVector_1);


//    ROS_INFO_STREAM("The cylinder centroid is (" << centroid.head<3>().transpose() <<
//                    ") and projected to its axis is (" << projectedCentroid.transpose() << ")");

    Eigen::Matrix4d cylinderTransform_1,cylinderTransform_2;
    //create a frame given the cylinder parameters (point and vector)
    pal::pointAndLineTransform(lineVector_2,
                               projectedCentroid_2,
                               cylinderTransform_2);
    pal::pointAndLineTransform(lineVector_1,
                               projectedCentroid_1,
                               cylinderTransform_1);


    publish(pclFilteredCylinderCloud_1,
            cylinderCoefficients_1,
            cylinderTransform_1,
            cylinderTransform_2,
            cylinderHeight_1,
            cloud->header);

  }
}

void CylinderDetector::publishPose(const geometry_msgs::Pose& pose,
                                   const std_msgs::Header& header)
{
  if ( _cylinderPosePub.getNumSubscribers() > 0 )
  {
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.pose   = pose;
    poseMsg.header = header;
    _cylinderPosePub.publish(poseMsg);
  }
}

void CylinderDetector::publishPose_1(const geometry_msgs::Pose& pose,
                                   const std_msgs::Header& header)
{
  if ( _cylinderPosePub_1.getNumSubscribers() > 0 )
  {
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.pose   = pose;
    poseMsg.header = header;
    _cylinderPosePub_1.publish(poseMsg);
  }
}



void CylinderDetector::publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinderCloud,
                               const pcl::ModelCoefficients::Ptr& cylinderCoefficients,
                               const Eigen::Matrix4d& transform,
                               const Eigen::Matrix4d& transform_1,
                               double cylinderHeight,
                               const std_msgs::Header& header)
{
  if ( _cylinderCloudPub.getNumSubscribers() > 0 )
  {
    pcl_conversions::toPCL(header, cylinderCloud->header);
    _cylinderCloudPub.publish(cylinderCloud);
  }

  geometry_msgs::Pose pose,pose_1;
  pal::convert(transform, pose);
  pal::convert(transform_1, pose_1);

  if ( _cylinderPosePub.getNumSubscribers() > 0 )
    publishPose(pose, header);
  publishPose_1(pose_1, header);

  if ( _cylinderMarkerPub.getNumSubscribers() > 0 )
  {    
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose = pose;
    marker.scale.x = cylinderCoefficients->values[6]*2;
    marker.scale.y = cylinderCoefficients->values[6]*2;
    marker.scale.z = cylinderHeight;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    _cylinderMarkerPub.publish(marker);
  }

  if ( _objectPub.getNumSubscribers() > 0 )
  {
    object_recognition_msgs::RecognizedObjectArray objects;
    objects.header = header;
    object_recognition_msgs::RecognizedObject object;
    object.pose.pose.pose = pose;
    object.pose.header = header;
    objects.objects.push_back(object);
    _objectPub.publish(objects);
  }
}


void CylinderDetector::start()
{
  _cloudSub = _nh.subscribe("cloud", 1, &CylinderDetector::cloudCallback, this);
  _enabled = true;
}

void CylinderDetector::stop()
{
  _cloudSub.shutdown();
  _enabled = false;
}

void CylinderDetector::run()
{
  ros::Rate loopRate(_rate);

  double halfPeriod = 0.5*1.0/_rate;

  while ( ros::ok() )
  {
    bool anySubscriber = _cylinderPosePub.getNumSubscribers() > 0 ||
                         _cylinderCloudPub.getNumSubscribers() > 0 ||
                         _cylinderMarkerPub.getNumSubscribers() > 0 ||
                         _objectPub.getNumSubscribers() > 0;


    if ( !_enabled && anySubscriber )
    {
      ROS_INFO("Enabling node because there are subscribers");
      start();
    }
    else if ( _enabled && !anySubscriber )
    {
      ROS_INFO("Disabling node because there are no subscribers");
      stop();
    }

    //check for subscriber's callbacks
    _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

    loopRate.sleep();
  }
}


}

int main(int argc, char**argv)
{
  ros::init (argc, argv, "cylinder_detector");

  ros::NodeHandle nh, pnh("~");

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  pal::CylinderDetector detector(nh, pnh);

  detector.run();

  return 0;
}


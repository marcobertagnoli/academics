/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2010-2011, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Example file for performing people detection on a PCD file.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and k (IROS) 2012, Vilamoura (Portugal), 2012.
 */

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>  

//convenient typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main (int argc, char** argv)
{ 
  int number_of_clouds=4; //Number of inputs
  /// PCL viewer ///
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  /// Dataset Parameters:
  std::string path = "../data/people/";
  std::string extension=".pcd";
  std::string svm_filename = "../data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float min_confidence = -2;
  float min_height = 1.5;
  float max_height = 2.3;
  float voxel_size = 0.06;
  float sampling_factor = 1;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics
  int name;


  for (int schene=0; schene<number_of_clouds; schene++)
  {
    unsigned int k = 0; //number of detected people

    // Read Kinect data:
    PointCloudT::Ptr cloud = PointCloudT::Ptr (new PointCloudT);

    std::stringstream ss; //path to reach the files
    name=schene+1;
    ss << path << name << extension;

    std::cout<<"Computing: " << ss.str() << endl;

    if (pcl::io::loadPCDFile(ss.str(), *cloud) < 0)
    {
      cerr << "Failed to read people point cloud." << endl;
      return (-1);
    } 

    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>), plane_cloud (new pcl::PointCloud<PointT>),
    remaining_cloud (new pcl::PointCloud<PointT>);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (0.01f, 0.01f, 0.01f);
    vox.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
 
    // Extract planes while 30% of the original cloud is still there or people are detected
    while (cloud_filtered->points.size () > 0.3 * nr_points && k == 0)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the inliers
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*plane_cloud);
      std::cerr << "PointCloud representing the planar component: " << plane_cloud->width * plane_cloud->height << " data points." << std::endl;

      // Create the filtering object
      extract.setNegative (true);   // to make filter method to return "outliers" instead of "inliers"
      extract.filter (*remaining_cloud);
      cloud_filtered.swap (remaining_cloud);
      i++;

      // Visualization:
      // Draw output point cloud:
      viewer.removeAllShapes();
      viewer.removeAllPointClouds();
      viewer.setBackgroundColor (0, 0, 0);
      viewer.setCameraPosition(0,0,-2,0,-1,0,0);
      viewer.addCoordinateSystem (0.1);
      viewer.addPointCloud<PointT> (remaining_cloud, "cloud");

      pcl::visualization::PointCloudColorHandlerCustom<PointT> red (plane_cloud, 255, 0, 0);
      viewer.addPointCloud<PointT> (plane_cloud, red, "plane");

      std::cout << "Detected ground visualization...press Q to continue. "<< std::endl;
      viewer.spin ();

    
      // Create classifier for people detection:
      pcl::people::PersonClassifier<pcl::RGB> person_classifier;
      person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

      // People detection app initialization:
      pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
      people_detector.setVoxelSize(voxel_size);                        // set the voxel size
      people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
      people_detector.setClassifier(person_classifier);                // set person classifier
      people_detector.setHeightLimits(min_height, max_height);         // set person classifier
      people_detector.setSamplingFactor(sampling_factor);              // set a downsampling factor to the point cloud (for increasing speed)

      // Ground plane estimation:
      Eigen::VectorXf ground_coeffs;
      ground_coeffs.resize(4);
      //estrazione parametri del piano
      ground_coeffs(0)=coefficients->values[0];
      ground_coeffs(1)=coefficients->values[1];
      ground_coeffs(2)=coefficients->values[2];
      ground_coeffs(3)=coefficients->values[3];


      // Perform people detection on the new cloud:
      std::cout << "Detected people visualization... " ;
      std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
      people_detector.setInputCloud(cloud);
      people_detector.setGround(ground_coeffs);                    // set floor coefficients
      people_detector.compute(clusters);                           // perform people detection
      ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

      // Draw cloud and people bounding boxes in the viewer:
      viewer.removeAllPointClouds();
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
      viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
      viewer.addPointCloud<PointT> (plane_cloud, red, "plane_people");
      
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
        {
          // draw theoretical person bounding box in the PCL viewer:
          it->drawTBoundingBox(viewer, k);
          k++;
        }
      }
      std::cout << k << " people found: " ;
      if(k==0)
      {
        std::cout << "press q to continue with the search" << std::endl;
      }
      else
      {
        std::cout << "Search concluded, press q to change point cloud " << std::endl;    
      }

      viewer.spin();
    }

}

  std::cout<<"Point clouds ended...Closing"<<endl;

  return 0;
}


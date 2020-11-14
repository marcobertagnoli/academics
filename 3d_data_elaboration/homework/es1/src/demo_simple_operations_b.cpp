/*
 * demo_simple_operations.cpp
 *
 *  Created on: Apr 2, 2012
 *      Author: iaslab
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

int
main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset/table_scene_lms400.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from the pcd file. "
			<< std::endl;
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
				<< " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	// Create two new clouds from the original one:
	// we want to put in cloud1 points with y < 0
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	// Pre-allocate the space for all the points:
	pcl::PointXYZRGB p;
    cloud_out->points.resize(cloud->width * cloud->height, p);
    cloud_out->width = cloud->width;
	cloud_out->height = cloud->height;
	
	// For unorganized point clouds:
	for (int j=0; j<cloud->width; j++)
	{ 
		p.x = cloud->at(j).x;
		p.y = cloud->at(j).y;
		p.z = cloud->at(j).z;
		if (cloud->at(j).y < 0)
		{
			p.r = 255;
			p.g = 0;
			p.b = 0;
		}
		else
		{
			p.r = 0;
			p.g = 255;
			p.b = 0;
		}
		cloud_out->at(j) = p;
	}
	
	/*
	// For organized point clouds:
	for (int i=0; i<cloud->height; i++)
	{
		for (int j=0; j<cloud->width; j++)
		{ 
			p.x = cloud->at(j,i).x;
			p.y = cloud->at(j,i).y;
			p.z = cloud->at(j,i).z;
			if (cloud->at(j,i).y < 0)
			{
				p.r = 255;
				p.g = 0;
				p.b = 0;
			}
			else
			{
				p.r = 0;
				p.g = 255;
				p.b = 0;
			}
			cloud_out->at(j,i) = p;
		}
	}
	*/

	// Visualization:
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	// Draw output point cloud:
	viewer.setBackgroundColor (0, 0, 0);
	viewer.addCoordinateSystem (0.1); //, "cloud"
	viewer.addText ("Cloud 1", 10, 10);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_out);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_out, rgb, "cloud");

	// Loop for visualization (so that the visualizers are continuously updated):
	std::cout << "Visualization... "<< std::endl;
	
	viewer.spin ();
	
	return 0;
}

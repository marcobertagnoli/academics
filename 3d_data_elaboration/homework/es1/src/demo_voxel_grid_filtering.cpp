/*
 * demo_voxel_grid_filtering.cpp
 *
 *  Created on: Apr 2, 2012
 *      Author: iaslab
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>					// for voxel grid filtering
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

int
main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

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

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
			<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

	// Save filtered cloud to disk as a .pcd file:
	pcl::io::savePCDFileASCII ("../dataset/table_scene_lms400_filtered.pcd", *cloud_filtered);

	// Visualization:
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	// Draw original point cloud to the left:
	int v1(0);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor (0, 0, 0, v1);
	viewer.addCoordinateSystem (0.1, v1); //, "cloud"
	viewer.addText ("Original cloud", 10, 10, "v1 text", v1);
	viewer.addPointCloud<pcl::PointXYZ> (cloud, "input_cloud", v1);

	// Draw filtered point cloud to the right:
	int v2(0);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor (0.3, 0.3, 0.3, v2);
	viewer.addCoordinateSystem (0.1, v2); //, "cloud"
	viewer.addText ("Filtered cloud", 10, 10, "v2 text", v2);
	viewer.addPointCloud<pcl::PointXYZ> (cloud_filtered, "filtered_cloud", v2);

	// Loop for visualization (so that the visualizers are continuously updated):
	std::cout << "Visualization... "<< std::endl;
	viewer.spin ();
	
	return 0;
}





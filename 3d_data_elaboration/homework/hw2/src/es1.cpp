#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

int main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/table_scene_lms400.pcd", *cloud) == -1) //* load the file
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

	// Create a new cloud from the original one:
	// we want to put in cloud_out points with x < 0 and color them blue
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x < 0)
		{
			pcl::PointXYZRGB p;	
			p.r = 0;
			p.g = 0;
			p.b = 255;
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;

			cloud_out->points.push_back(p);
		}		
	}

	//Store the point cloud created
	std::cout<<"Saving the point cloud... ";
	cloud_out -> width = 1;
    cloud_out -> height = cloud_out->points.size();
	pcl::io::savePCDFileASCII ("../data/es1/cloud_out.pcd", *cloud_out);
	cout<< "done" <<endl;

	// Visualization:
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	// Draw output point cloud:
	viewer.setBackgroundColor (0, 0, 0);
	viewer.addCoordinateSystem (0.1); //, "cloud"
	viewer.addText ("Cloud 1", 10, 10);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_out);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_out, rgb, "cloud");

	std::cout << "Visualization... "<< std::endl;	
	viewer.spin ();
	
	return 0;
}

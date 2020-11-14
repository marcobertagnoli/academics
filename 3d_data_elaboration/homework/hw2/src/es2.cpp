#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>					// for voxel grid filtering
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

int main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZRGB>);//top right cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);//top left cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_3 (new pcl::PointCloud<pcl::PointXYZRGB>);//bottom right cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_4 (new pcl::PointCloud<pcl::PointXYZRGB>);//bottom left cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);//final filtered cloud

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

	//Point cloud partitioning
	for (int i = 0; i < cloud->points.size(); i++)
	{	
		pcl::PointXYZRGB p;
		//top left
		if (cloud->points[i].x < 0 && cloud->points[i].y > 0)
		{
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			p.r = 0;
			p.g = 255;
			p.b = 0;
			cloud_filtered_1->points.push_back(p);
		}
		//top rigt
		else if (cloud->points[i].x > 0 && cloud->points[i].y > 0)
		{
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			p.r = 255;
			p.g = 255;
			p.b = 255;
			cloud_filtered_2->points.push_back(p);
		}
		//bottom left
		else if (cloud->points[i].x < 0 && cloud->points[i].y < 0)
		{
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			p.r = 255;
			p.g = 0;
			p.b = 0;
			cloud_filtered_3->points.push_back(p);
		}
		//bottom right
		else if (cloud->points[i].x > 0 && cloud->points[i].y < 0)
		{
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			p.r = 0;
			p.g = 0;
			p.b = 255;
			cloud_filtered_4->points.push_back(p);
		}
	}

	//FILTERING

	//top left
	pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
	sor1.setInputCloud (cloud_filtered_1);
	sor1.setLeafSize (0.01f, 0.01f, 0.01f);
	sor1.filter (*cloud_filtered_1);
	//top right
	pcl::VoxelGrid<pcl::PointXYZRGB> sor2;
	sor2.setInputCloud (cloud_filtered_2);
	sor2.setLeafSize (0.1f, 0.1f, 0.1f);
	sor2.filter (*cloud_filtered_2);
	//bottom left
	pcl::VoxelGrid<pcl::PointXYZRGB> sor3;
	sor3.setInputCloud (cloud_filtered_3);
	sor3.setLeafSize (0.005f, 0.005f, 0.005f);
	sor3.filter (*cloud_filtered_3);
	//bottom right
	pcl::VoxelGrid<pcl::PointXYZRGB> sor4;
	sor4.setInputCloud (cloud_filtered_4);
	sor4.setLeafSize (0.05f, 0.05f, 0.05f);
	sor4.filter (*cloud_filtered_4);

	//concatenate the 4 filtered clouds
	*cloud_filtered = *cloud_filtered_1 + *cloud_filtered_2;
	*cloud_filtered += *cloud_filtered_3;
	*cloud_filtered += *cloud_filtered_4;



	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
			<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

	// Save filtered cloud to disk as a .pcd file:
	pcl::io::savePCDFileASCII ("../data/es2/es2_filtered_cloud.pcd", *cloud_filtered);

	// Visualization:
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);	

	// Draw original point cloud to the left:
	int v1(0);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.addCoordinateSystem (0.1, v1); 
	viewer.addText ("Original cloud", 10, 10, "v1 text", v1);
	viewer.addPointCloud<pcl::PointXYZ> (cloud, "input_cloud", v1);

	// Draw filtered point cloud to the right:
	int v2(0);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	viewer.addCoordinateSystem (0.1, v2); 
	viewer.setBackgroundColor (0.5, 0.5, 0.5);
	viewer.addText ("Filtered cloud", 10, 10, "v2 text", v2);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "top left", v2);

	std::cout << "Visualization... "<< std::endl;
	viewer.spin ();
	
	return 0;
}





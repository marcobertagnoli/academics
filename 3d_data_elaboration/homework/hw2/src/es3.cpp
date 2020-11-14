#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>	    
#include <pcl/io/pcd_io.h> 		   
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>		

int main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../data/minimouse1_segmented.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from the pcd file. "
			<< std::endl;

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_0002 (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the normals:
	std::cout << "Computing normals...please wait...";
	ne.setNumberOfThreads(4); 	// set number of threads when using OpenMP 
	ne.compute (*cloud_normals);
	std::cout << "done." << std::endl;

	// Use all neighbors in a sphere of radius 0.2cm
	ne.setRadiusSearch (0.002);

	// Compute the normals:
	std::cout << "Computing second normals...please wait...";
	ne.setNumberOfThreads(4); 	// set number of threads when using OpenMP 
	ne.compute (*cloud_normals_0002);
	std::cout << "done." << std::endl;

	// Visualize normals
	int normalsVisualizationStep = 100; // to visualize a normal every normalsVisualizationStep
	float normalsScale = 0.02;

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

	//Visualize the normals with r=3cm on the left
	int v1(0);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.addText ("Cloud's normals 3cm", 10, 10, "v1 text", v1);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "Normals3cm",v1);

	//Visualize the normals with r=0.2cm on the right
	int v2(0);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);	
	viewer.addText ("Cloud's normals 0.2cm", 10, 10, "v2 text", v2);	
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "Normals02cm",v2);

	viewer.addCoordinateSystem (0.1);
	viewer.setBackgroundColor (0, 0, 0.5);
	viewer.initCameraParameters ();

	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals, normalsVisualizationStep, normalsScale, "normals1",v1);
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals_0002, normalsVisualizationStep, normalsScale, "normals2",v2);

	std::cout << "Visualization...: "<< std::endl;
	while (!viewer.wasStopped ())
	{
		viewer.spin ();
	}
	return 0;
}

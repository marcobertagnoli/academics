#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer
#include <pcl/visualization/histogram_visualizer.h>	// for histogram visualization
#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation
#include <pcl/features/fpfh_omp.h>		// for computing FPFH with multi-core implementation
#include <pcl/filters/filter.h>		// for removing NaN from the point cloud
#include <pcl/keypoints/sift_keypoint.h> //for compute key points

struct callbackArgs{
	// structure used to pass arguments to the callback function
	pcl::visualization::PCLHistogramVisualizer *histViewer;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs;
};

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
	// callback function used to visualize feature histograms
	struct callbackArgs* data = (struct callbackArgs *)args;
    if (event.getPointIndex () == -1)
      return;
    std::stringstream windowName;
    windowName << "FPFH for point " << event.getPointIndex();
    data->histViewer->addFeatureHistogram (*(data->fpfhs), "fpfh", event.getPointIndex(), windowName.str(), 640, 200);
}

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

	// Remove NaN from the point cloud:
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	//Detect the keypoints whith SIFT operator
	pcl::SIFTKeypoint<pcl::PointXYZRGB,pcl::PointXYZRGB> kp;
	kp.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_kp (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	kp.setSearchMethod (tree_kp);
	kp.setScales(0.01, 3, 2);
	kp.setMinimumContrast(0);
	std::cout << "Computing key points...please wait...";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr KeyPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	kp.compute(*KeyPoints);
	std::cout << "done." << std::endl;

	int kp_number = KeyPoints->width * KeyPoints->height;

	if (kp_number==0) //if no points are found throw an error
	{
		PCL_ERROR ("No key points founded \n");
		return (-1);
	}

	std::cout << "Selected "
			<< kp_number
			<< " key points from the cloud. "
			<< std::endl;

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given PC (in setSearchSurface).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 1cm
	ne.setRadiusSearch (0.01);

	// Compute the features
	std::cout << "Computing normals...please wait...";
	ne.setNumberOfThreads(4); 	// set number of threads when using OpenMP 
	ne.compute (*normals);	
	std::cout << "done." << std::endl;

	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (KeyPoints);
	fpfh.setInputNormals (normals);
	fpfh.setSearchSurface(cloud); //calcolo le features in base alla point cloud completa
	
	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

	// Use all neighbors in a sphere of radius 3cm
	fpfh.setRadiusSearch (0.03);
	std::cout << "Computing FPFH features...please wait...";
	// Compute the features	
	fpfh.setNumberOfThreads(4); 	// set number of threads when using OpenMP
	fpfh.compute (*fpfhs);
	std::cout << "done." << std::endl;

	// Visualize FPFH:
	int normalsVisualizationStep = 100; // to visualize a normal every normalsVisualizationStep
	float normalsScale = 0.02;			// normals dimension

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.addCoordinateSystem (0.1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbc(cloud);

	//visualize the pointcloud+normals on the left
	int v1(0);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.addText ("Cloud with normals", 10, 10, "v1 text", v1);
	viewer.initCameraParameters ();
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgbc, "Normals",v1);
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, normalsVisualizationStep, normalsScale, "normals1",v1);

	//visualize the key points on the right
	int v2(0);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor (0, 0, 0.5);
	viewer.addText ("Key Points", 10, 10, "v2 text", v2);
	pcl::visualization::PCLHistogramVisualizer histViewer;
	viewer.initCameraParameters ();
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(KeyPoints);
	viewer.addPointCloud<pcl::PointXYZRGB> (KeyPoints, rgb, "input_cloud", v2);
	
	// Create structure containing the parameters for the callback function
	struct callbackArgs histCallbackArgs;
	histCallbackArgs.histViewer = &histViewer;
	histCallbackArgs.fpfhs = fpfhs;

	// Add point picking callback to viewer (for visualizing feature histograms):
	viewer.registerPointPickingCallback (pp_callback, (void*)&histCallbackArgs);

	// Loop for visualization (so that the visualizers are continuously updated):
	std::cout << "Visualization... "<< std::endl;
	while (!viewer.wasStopped ())
	{
		viewer.spin ();
		histViewer.spin();
	}
	return 0;
}

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <regex>
#include <boost/thread/thread.hpp>

#include "Connect3D.h"

using namespace pcl;

void
 visualize (PointCloud<PointXYZRGB>::Ptr cloud)
 {
	visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ()){}
 }


PointCloud<PointXYZRGB>::Ptr filter(PointCloud<PointXYZRGB>::Ptr cloud_in){
	// calculate leaf size for downsampling
	PointXYZRGB min;
	PointXYZRGB max;
	getMinMax3D(*cloud_in, min, max);
	float ls = std::max(max.x-min.x,std::max(max.y-min.y, max.z-min.z))/200; //edit divisor for different finer/rougher downsampling
	
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	VoxelGrid<PointXYZRGB> vg;
	PointCloud<PointXYZRGB>::Ptr cloud_filtered (new PointCloud<PointXYZRGB>);
	vg.setInputCloud (cloud_in);
	vg.setLeafSize (ls, ls, ls);
	vg.filter (*cloud_filtered);
	cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << endl;
	return cloud_filtered;
}


//TODO plane-segmenten farben geben
/*
adjustierbar:
- distance threshold für plane segmentation (momentan: groesste ausdehnung(x,y oder z)/100)
- maximale iterationen für plane segmentation (momentan: 100)
- abbruch der plane-segmentation-schleife (momentan: letzte ebene kleiner als cloud_input->size()/10)
- cluster tolerance f. euclidean clustering (momentan: =distance threshold fuer plane segmentation)
- minimale cluster size (momentan: anzahl input-punkte/500)
- maximale cluster size (momentan: unbegrenzt)
*/
PointCloud<PointXYZRGB>::Ptr planeKnn(PointCloud<PointXYZRGB>::Ptr cloud_in){

	PointCloud<PointXYZRGB>::Ptr cloud_filtered (cloud_in), cloud_f(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud_segmented (new PointCloud<PointXYZRGB>);

	// calculate distance threshold
	PointXYZRGB min;
	PointXYZRGB max;
	getMinMax3D(*cloud_in, min, max);
	float dt = std::max(max.x-min.x,std::max(max.y-min.y, max.z-min.z))/100; //edit divisor for different threshold
	
	// Create the segmentation object for the planar model and set all the parameters
	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers (new PointIndices);
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointCloud<PointXYZRGB>::Ptr cloud_plane (new PointCloud<PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (dt);

	//int i=0, nr_points = (int) cloud_filtered->points.size ();
	//while (cloud_filtered->points.size () > 0.3 * nr_points)
	//int size_last_plane=INT_MAX;
	while (true)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Write the planar inliers to disk
		extract.filter (*cloud_plane);
		
		//!!!!!!BREAK CONDITION!!!!!!!
		if(cloud_plane->size() < cloud_in->points.size()/10) break;
		//size_last_plane = cloud_plane->size();
		
		
		//color points of planar component and add them to segmentation solution
		int r = std::rand()%256;
		int g = std::rand()%256;
		int b = std::rand()%256;
		for(PointCloud<PointXYZRGB>::const_iterator it = cloud_plane->points.begin(); it != cloud_plane->points.end(); ++it){
			PointXYZRGB point(r,g,b);
			point.x=it->x;
			point.y=it->y;
			point.z=it->z;
			cloud_segmented->push_back(point);
		}

		cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered = cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud_filtered);	//hier absturz bei manchen clouds
	vector<PointIndices> cluster_indices;
	EuclideanClusterExtraction<PointXYZRGB> ec;
	ec.setClusterTolerance (dt); // 2cm
	ec.setMinClusterSize (cloud_in->size()/500); //irgendwas
	//ec.setMaxClusterSize (cloud_in->size()/2);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);
  
	int j = 0;
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
    
		int r = std::rand()%256;
		int g = std::rand()%256;
		int b = std::rand()%256;
		int pointcounter=0;
	
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			cloud_filtered->points[*pit];
			PointXYZRGB point(r,g,b);
			point.x=cloud_filtered->points[*pit].x;
			point.y=cloud_filtered->points[*pit].y;
			point.z=cloud_filtered->points[*pit].z;
			cloud_segmented->points.push_back (point); 
			pointcounter++;
		}
      
		cloud_segmented->width = cloud_segmented->points.size ();
		cloud_segmented->height = 1;
		cloud_segmented->is_dense = true;

		cout << "Cluster "<<j<<": " << pointcounter << " data points." << endl;
    
		j++;
	} 

	cout << "segmentation completed" << endl;

	return cloud_segmented;
}




PointCloud<PointXYZRGB>::Ptr planeBc(PointCloud<PointXYZRGB>::Ptr cloud_in){

	PointCloud<PointXYZRGB>::Ptr cloud_filtered (cloud_in); 
	PointCloud<PointXYZRGB>::Ptr cloud_f(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud_segmented (new PointCloud<PointXYZRGB>);

	// calculate distance threshold
	PointXYZRGB min;
	PointXYZRGB max;
	getMinMax3D(*cloud_in, min, max);
	float dt = std::max(max.x-min.x,std::max(max.y-min.y, max.z-min.z))/100; //edit divisor for different threshold
	



	// Create the segmentation object for the planar model and set all the parameters
	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers (new PointIndices);
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointCloud<PointXYZRGB>::Ptr cloud_plane (new PointCloud<PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (dt);


	while (true)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Write the planar inliers to disk
		extract.filter (*cloud_plane);
		
		//!!!!!!BREAK CONDITION!!!!!!!
		if(cloud_plane->size() < cloud_in->points.size()/10) break;
		
		
		//color points of planar component and add them to segmentation solution
		int r = std::rand()%256;
		int g = std::rand()%256;
		int b = std::rand()%256;
		for(PointCloud<PointXYZRGB>::const_iterator it = cloud_plane->points.begin(); it != cloud_plane->points.end(); ++it){
			PointXYZRGB point(r,g,b);
			point.x=it->x;
			point.y=it->y;
			point.z=it->z;
			cloud_segmented->push_back(point);
		}

		cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered = cloud_f;
	}

	Connect3D *bc = new Connect3D(cloud_filtered);
	bc->connect3D();



	/*
	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud_filtered);	//hier absturz bei manchen clouds
	vector<PointIndices> cluster_indices;
	EuclideanClusterExtraction<PointXYZRGB> ec;
	ec.setClusterTolerance (dt); // 2cm
	ec.setMinClusterSize (cloud_in->size()/500); //irgendwas
	//ec.setMaxClusterSize (cloud_in->size()/2);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);
  
	int j = 0;
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
    
		int r = std::rand()%256;
		int g = std::rand()%256;
		int b = std::rand()%256;
		int pointcounter=0;
	
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			cloud_filtered->points[*pit];
			PointXYZRGB point(r,g,b);
			point.x=cloud_filtered->points[*pit].x;
			point.y=cloud_filtered->points[*pit].y;
			point.z=cloud_filtered->points[*pit].z;
			cloud_segmented->points.push_back (point); 
			pointcounter++;
		}
      
		cloud_segmented->width = cloud_segmented->points.size ();
		cloud_segmented->height = 1;
		cloud_segmented->is_dense = true;

		cout << "Cluster "<<j<<": " << pointcounter << " data points." << endl;
    
		j++;
	} 
	*/

	cout << "segmentation completed" << endl;

	return cloud_segmented;
}




int main (int argc, char** argv)
{
  
	if(argc<2){
		cout << "usage:" << endl << "	pcs [filename]" << endl << "	(can only read .pcd and .ply)";
		return(0);
	}
	
	string filename = argv[1];

	//true if input file is .pcd, false if it's .ply
	bool pcd;
	//this will be our original point cloud data set
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(new PointCloud<PointXYZRGB>());
	
	//load file
	if(regex_match(filename, regex("(.*)(\.pcd)"))){
		PCDReader reader;
		reader.read(filename, *cloud_input);
		pcd = true;
		cout << "successfully loaded " << filename << ": " << cloud_input->size() << " points" << endl;
	} else if(regex_match(filename, regex("(.*)(\.ply)"))){
		PLYReader reader;
		reader.read(filename, *cloud_input);
		pcd = false;
		cout << "successfully loaded " << filename << ": " << cloud_input->size() << " points" << endl;
	} else {
		cout << "usage:" << endl << "	pcs [filename]" << endl << "	(can only read .pcd and .ply)";
		return(0);
	}

	//downsampling
	cloud_input=filter(cloud_input);

	
	//these are the different segmented point clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_knn (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_bc (new pcl::PointCloud<pcl::PointXYZRGB>);

	
	//handle user commands
	string in;
	while(true){
		cin >> in;
		
		if(string(in)=="showinput"){
			boost::thread(visualize, cloud_input);
			continue;
		}



		
		if(string(in)=="planeknn"){
			cloud_plane_knn = planeKnn(cloud_input);
			continue;
		}
		if(string(in)=="showplaneknn"){
			boost::thread(visualize, cloud_plane_knn);
			continue;
		}
		if(string(in)=="saveplaneknn"){
			if(pcd){
				PCDWriter writer;
				stringstream ss;
				ss << "planeKnn_" << filename;
				writer.write<PointXYZRGB> (ss.str (), *cloud_plane_knn, false);
			} else{
				PLYWriter writer;
				stringstream ss;
				ss << "planeKnn_" << filename;
				writer.write<PointXYZRGB> (ss.str (), *cloud_plane_knn, false);
			}
			cout << "segmented cloud has been saved to planeKnn_" << filename << endl;
			continue;
		}




		if(string(in)=="planebc"){
			cloud_plane_bc = planeBc(cloud_input);
			continue;
		}
		if(string(in)=="showplanebc"){
			boost::thread(visualize, cloud_plane_bc);
			continue;
		}
		if(string(in)=="saveplanebc"){
			if(pcd){
				PCDWriter writer;
				stringstream ss;
				ss << "planeBc_" << filename;
				writer.write<PointXYZRGB> (ss.str (), *cloud_plane_bc, false);
			} else{
				PLYWriter writer;
				stringstream ss;
				ss << "planeBc_" << filename;
				writer.write<PointXYZRGB> (ss.str (), *cloud_plane_bc, false);
			}
			cout << "segmented cloud has been saved to planeBc_" << filename << endl;
			continue;
		}




		if(string(in)=="exit"){
			break;
		}
			
		cout << "valid commands: " << endl <<
			"	showinput" << endl <<

			"	planeknn" << endl <<
			"	showplaneknn" << endl <<
			"	saveplaneknn" << endl <<

			"	planebc" << endl <<
			"	showplanebc" << endl <<
			"	saveplanebc" << endl <<

			"	exit" << endl;
	}

  return (0);
}
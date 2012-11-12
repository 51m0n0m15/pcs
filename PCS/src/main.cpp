#include <pcl/point_types.h>
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

#include "BoundaryComplex.h"
#include "config.h"

using namespace pcl;

//this will be our original point cloud data set
pcl::PointCloud<PointXYZRGB>::Ptr cloud_input(new PointCloud<PointXYZRGB>());

//these are the different segmented point clouds
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_knn;//(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_bc;//(new pcl::PointCloud<pcl::PointXYZRGB>);

//for cluster-labelling
vector<int> *cl_plane_knn;
vector<int> *cl_plane_bc;

//just good to know
bool plane_knn_done = false;
bool plane_bc_done = false;

//construct the boundary complex for filtered input data
BoundaryComplex *bc;


void visualize (PointCloud<PointXYZRGB>::Ptr cloud) {
	visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ()){}
}


PointCloud<PointXYZRGB>::Ptr filter(PointCloud<PointXYZRGB>::Ptr cloud_in){
	// calculate leaf size for downsampling
	PointXYZRGB min;
	PointXYZRGB max;
	getMinMax3D(*cloud_in, min, max);
	float ls = std::max(max.x-min.x,std::max(max.y-min.y, max.z-min.z))/config::filterLeafSize;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	VoxelGrid<PointXYZRGB> vg;
	PointCloud<PointXYZRGB>::Ptr cloud_filtered (new PointCloud<PointXYZRGB>);
	vg.setInputCloud (cloud_in);
	vg.setLeafSize (ls, ls, ls);
	vg.filter (*cloud_filtered);
	cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << endl;
	return cloud_filtered;
}


/*
- maximale iterationen für plane segmentation (momentan: 100)
- abbruch der plane-segmentation-schleife (momentan: letzte ebene kleiner als cloud_input->size()/10)
*/
void planeKnn(){

	if(plane_knn_done){
		cout << "You already did that." << endl;
		return;
	}

	int clusterNo=1;

	PointCloud<PointXYZRGB>::Ptr cloud_filtered (cloud_input), cloud_f(new PointCloud<PointXYZRGB>);

	// calculate distance threshold
	PointXYZRGB min;
	PointXYZRGB max;
	getMinMax3D(*cloud_input, min, max);
	float maxExp = std::max(max.x-min.x,std::max(max.y-min.y, max.z-min.z));

	// Create the segmentation object for the planar model and set all the parameters
	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers (new PointIndices);
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointCloud<PointXYZRGB>::Ptr cloud_plane (new PointCloud<PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (maxExp/config::planeDistThreshold);

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
		if(cloud_plane->size() < cloud_input->points.size()/10) break;
		//size_last_plane = cloud_plane->size();


		//add planar component to segmentation solution
		int offset=0;
		int pos=0;
		vector<int> *indices = extract.getIndices().get();
		for(int i=0; i<cloud_plane->size(); i++){
			//update the cluster-vector..
			while(pos <= indices->at(i)+offset){
				if(cl_plane_knn->at(pos)!=0) offset++;
				pos++;
			}
			cl_plane_knn->at(indices->at(i)+offset)=clusterNo;
		}

		cout << "Cluster " << clusterNo << ", planar component: " << cloud_plane->points.size () << " data points." << endl;
		clusterNo++;

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
	ec.setClusterTolerance (maxExp/config::clusterDistThreshold);
	ec.setMinClusterSize (cloud_input->size()/config::minClusterSize);
	//ec.setMaxClusterSize (cloud_in->size()/2);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	vector<int> cl_euclid_part(cloud_filtered->size());
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		int pointcounter=0;
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			pointcounter++;

			//update temporal euclidean clustering vector
			cl_euclid_part[*pit]=clusterNo;
		}

		cout << "Cluster "<<clusterNo<<": " << pointcounter << " data points." << endl;

		clusterNo++;
	} 

	//add euclid segmentation to plane segmentation
	int offset=0;
	int pos=0;
	for(int i=0; i<cl_euclid_part.size(); i++){
		while(pos <= i+offset){
			if(cl_plane_knn->at(pos)!=0) offset++;
			pos++;
		}
		cl_plane_knn->at(i+offset)=cl_euclid_part[i];
	}


	//color the plane_knn-cloud
	vector<vector<int>> colors(clusterNo-1);
	for(int i=0; i<clusterNo-1; i++){
		vector<int> c(3);
		c[0]=std::rand()%256;
		c[1]=std::rand()%256;
		c[2]=std::rand()%256;
		colors[i]=c;
	}

	for(int i=0; i<cl_plane_knn->size(); i++){
		if(cl_plane_knn->at(i) > 0){
			cloud_plane_knn->points[i].r = colors[cl_plane_knn->at(i)-1][0];
			cloud_plane_knn->points[i].g = colors[cl_plane_knn->at(i)-1][1];
			cloud_plane_knn->points[i].b = colors[cl_plane_knn->at(i)-1][2];
		} else {
			cloud_plane_knn->points[i].r = 0;
			cloud_plane_knn->points[i].g = 0;
			cloud_plane_knn->points[i].b = 0;
		}
	}

	plane_knn_done=true;
	cout << "segmentation done" << endl;
}




void planeBc(){

	if(plane_bc_done){
		cout << "You already did that." << endl;
		return;
	}

	int clusterNo = 1;

	PointCloud<PointXYZRGB>::Ptr cloud_filtered (cloud_input), cloud_f(new PointCloud<PointXYZRGB>);

	// calculate distance threshold
	PointXYZRGB min;
	PointXYZRGB max;
	getMinMax3D(*cloud_input, min, max);
	float maxExp = std::max(max.x-min.x,std::max(max.y-min.y, max.z-min.z));

	// Create the segmentation object for the planar model and set all the parameters
	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers (new PointIndices);
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointCloud<PointXYZRGB>::Ptr cloud_plane (new PointCloud<PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (maxExp/config::planeDistThreshold);


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
		if(cloud_plane->size() < cloud_input->points.size()/10) break;
		//size_last_plane = cloud_plane->size();


		//add planar component to segmentation solution
		int offset=0;
		int pos=0;
		vector<int> *indices = extract.getIndices().get();
		for(int i=0; i<cloud_plane->size(); i++){
			//update the cluster-vector..
			while(pos <= indices->at(i)+offset){
				if(cl_plane_bc->at(pos)!=0) offset++;
				pos++;
			}
			cl_plane_bc->at(indices->at(i)+offset)=clusterNo;
		}

		cout << "Cluster " << clusterNo << ", planar component: " << cloud_plane->points.size () << " data points." << endl;
		clusterNo++;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered = cloud_f;
	}

	bc->doClustering(cl_plane_bc, maxExp/config::clusterDistThreshold, clusterNo);



	//color the plane_bc-cloud
	vector<vector<int>> colors(clusterNo-1);
	for(int i=0; i<clusterNo-1; i++){
		vector<int> c(3);
		c[0]=std::rand()%256;
		c[1]=std::rand()%256;
		c[2]=std::rand()%256;
		colors[i]=c;
	}

	for(int i=0; i<cl_plane_bc->size(); i++){
		if(cl_plane_bc->at(i) > 0){
			cloud_plane_bc->points[i].r = colors[cl_plane_bc->at(i)-1][0];
			cloud_plane_bc->points[i].g = colors[cl_plane_bc->at(i)-1][1];
			cloud_plane_bc->points[i].b = colors[cl_plane_bc->at(i)-1][2];
		} else {
			cloud_plane_bc->points[i].r = 0;
			cloud_plane_bc->points[i].g = 0;
			cloud_plane_bc->points[i].b = 0;
		}
	}

	plane_bc_done=true;
	cout << "segmentation done" << endl;
}




int main (int argc, char** argv)
{

	config::readConfig();

	if(argc<2){
		cout << "usage:" << endl << "	pcs [filename]" << endl << "	(can only read .pcd and .ply)";
		return(0);
	}

	string filename = argv[1];

	//true if input file is .pcd, false if it's .ply
	bool pcd;

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

	//copy for different cluster solutions (and color all points black)
	cloud_plane_knn = *(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>(*cloud_input)));
	cloud_plane_bc = *(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>(*cloud_input)));

	//for cluster-labelling
	cl_plane_knn = new vector<int>(cloud_input->size(), 0);
	cl_plane_bc = new vector<int>(cloud_input->size(), 0);

	//construct the boundary complex for filtered input data
	bc = new BoundaryComplex(cloud_input);

	//handle user commands
	string in;
	while(true){
		cin >> in;

		if(string(in)=="showinput"){
			boost::thread(visualize, cloud_input);
			continue;
		}




		if(string(in)=="planeknn"){
			planeKnn();
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
			planeBc();
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
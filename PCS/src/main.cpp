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
#include <pcl/features/principal_curvatures.h>
#include <regex>
#include <boost/thread/thread.hpp>

#include "BoundaryComplex.h"
#include "Solution.h"
#include "config.h"

using namespace pcl;


//just for convenience
int pointCount;

Solution *input;
Solution *plane_knn;
Solution *plane_bc;
Solution *region_knn;
Solution *region_bc;

BoundaryComplex *bc;


void visualize (Solution *s) {
	visualization::CloudViewer viewer (s->name);
	viewer.showCloud (s->cloud);
	while (!viewer.wasStopped ()){}
}

/*
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
}*/


/*
noch variierbar:
- maximale iterationen für plane segmentation (momentan: 100)
- abbruch der plane-segmentation-schleife (momentan: letzte ebene kleiner als cloud_input->size()/10)
*/
void planeKnn(){

	if(plane_knn->clustering_done){
		cout << "You already did that." << endl;
		return;
	}

	int clusterNo=1;

	PointCloud<PointXYZRGB>::Ptr cloud_filtered (plane_knn->cloud), cloud_f(new PointCloud<PointXYZRGB>);

	// Create the segmentation object for the planar model and set all the parameters
	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers (new PointIndices);
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointCloud<PointXYZRGB>::Ptr cloud_plane (new PointCloud<PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (plane_knn->max_exp/config::plane_dist_threshold);

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
		if(cloud_plane->size() < pointCount/10) break;
		//size_last_plane = cloud_plane->size();


		//add planar component to segmentation solution
		int offset=0;
		int pos=0;
		vector<int> *indices = extract.getIndices().get();
		for(int i=0; i<cloud_plane->size(); i++){
			//update the cluster-vector..
			while(pos <= indices->at(i)+offset){
				if(plane_knn->clustering->at(pos)!=0) offset++;
				pos++;
			}
			plane_knn->clustering->at(indices->at(i)+offset)=clusterNo;
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
	ec.setClusterTolerance (plane_knn->dist_threshold);
	ec.setMinClusterSize (pointCount/config::min_cluster_size);
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
			if(plane_knn->clustering->at(pos)!=0) offset++;
			pos++;
		}
		plane_knn->clustering->at(i+offset)=cl_euclid_part[i];
	}

	plane_knn->cluster_count = clusterNo-1;
	plane_knn->clustering_done=true;

	plane_knn->color_cloud_from_clustering();

	cout << "Segmentation done." << endl;
}




void planeBc(){

	if(plane_bc->clustering_done){
		cout << "You already did that." << endl;
		return;
	}

	int clusterNo = 1;

	PointCloud<PointXYZRGB>::Ptr cloud_filtered (plane_bc->cloud), cloud_f(new PointCloud<PointXYZRGB>);


	// Create the segmentation object for the planar model and set all the parameters
	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers (new PointIndices);
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointCloud<PointXYZRGB>::Ptr cloud_plane (new PointCloud<PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (plane_bc->max_exp/config::plane_dist_threshold);


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
		if(cloud_plane->size() < pointCount/10) break;
		//size_last_plane = cloud_plane->size();


		//add planar component to segmentation solution
		int offset=0;
		int pos=0;
		vector<int> *indices = extract.getIndices().get();
		for(int i=0; i<cloud_plane->size(); i++){
			//update the cluster-vector..
			while(pos <= indices->at(i)+offset){
				if(plane_bc->clustering->at(pos)!=0) offset++;
				pos++;
			}
			plane_bc->clustering->at(indices->at(i)+offset)=clusterNo;
		}

		cout << "Cluster " << clusterNo << ", planar component: " << cloud_plane->points.size () << " data points." << endl;
		clusterNo++;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered = cloud_f;
	}

	//boundary-complex-clustering
	plane_bc->cluster_count=clusterNo;
	bc->doClustering(plane_bc);

	plane_bc->clustering_done=true;

	plane_bc->color_cloud_from_clustering();


	cout << "Segmentation done." << endl;
}


void regionKnn(){

	//compute normals
	search::Search<PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);
	PointCloud <Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	NormalEstimation<PointXYZRGB, Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (region_knn->cloud);
	normal_estimator.setKSearch (config::normal_est_k);
	normal_estimator.compute (*normals);

	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZRGB>::Ptr kdTree (new search::KdTree<pcl::PointXYZRGB>);
	kdTree->setInputCloud (region_knn->cloud);

	set<int> unlabeled;
	for(int i=0; i<region_knn->clustering->size(); i++)
		if(region_knn->clustering->at(i)==0)
			unlabeled.insert(i);
	
	region_knn->cluster_count=1;

	while(!unlabeled.empty()){
		list<int> queue;
		set<int> cluster;

		
		int seed=0;
		float min_curvature=FLT_MAX;
		for(set<int>::iterator iter=unlabeled.begin(); iter!=unlabeled.end(); iter++){
			if(normals->at(*iter).curvature<min_curvature){
				seed=*iter;
				min_curvature = normals->at(*iter).curvature;
			}
		}

		queue.push_back(seed);
		cluster.insert(seed);
		unlabeled.erase(seed);
		
		while(!queue.empty()){
			list<int>::iterator curPoint = queue.begin();
			vector<int> neighbors;
			vector<float> distances;
			kdTree->nearestKSearch(region_knn->cloud->at(*curPoint), config::region_growing_k, neighbors, distances); 
			queue.pop_front();

			for(vector<int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++){
				set<int>::iterator neighbor = unlabeled.find(*iter);
				if(neighbor!=unlabeled.end()){
					
					float normal_diff = acos((normals->at(*neighbor).normal_x*normals->at(*curPoint).normal_x+
												normals->at(*neighbor).normal_y*normals->at(*curPoint).normal_y+
												normals->at(*neighbor).normal_z*normals->at(*curPoint).normal_z)
												/
												(sqrt(pow(normals->at(*neighbor).normal_x,2)+
														pow(normals->at(*neighbor).normal_y,2)+
														pow(normals->at(*neighbor).normal_z,2))*
												 sqrt(pow(normals->at(*curPoint).normal_x,2)+
														pow(normals->at(*curPoint).normal_y,2)+
														pow(normals->at(*curPoint).normal_z,2))
														));
					
					if(normal_diff<config::normal_diff_threshold){
						cluster.insert(*neighbor);
						unlabeled.erase(neighbor);
						if(normals->at(*curPoint).curvature<config::curvature_threshold)
							queue.push_back(*neighbor);
					}
				}
			}
		}

		if(cluster.size() >= region_knn->cloud->size()/config::min_cluster_size){
			for(set<int>::iterator iter = cluster.begin(); iter!=cluster.end(); iter++){
				region_knn->clustering->at(*iter)=region_knn->cluster_count;
			}
			cout << "Cluster "<<region_knn->cluster_count<<": " << cluster.size() << " data points." << endl;
			region_knn->cluster_count++;
		}

	}
	
	region_knn->clustering_done=true;

	region_knn->color_cloud_from_clustering();

	cout << "Segmentation done." << endl;
}




void regionBc(){
	//compute normals
	search::Search<PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);
	PointCloud <Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	NormalEstimation<PointXYZRGB, Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (region_bc->cloud);
	normal_estimator.setKSearch (config::normal_est_k);
	normal_estimator.compute (*normals);

	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZRGB>::Ptr kdTree (new search::KdTree<pcl::PointXYZRGB>);
	kdTree->setInputCloud (region_bc->cloud);

	set<int> unlabeled;
	for(int i=0; i<region_bc->clustering->size(); i++)
		if(region_bc->clustering->at(i)==0)
			unlabeled.insert(i);
	
	region_bc->cluster_count=1;

	while(!unlabeled.empty()){
		list<int> queue;
		set<int> cluster;

		
		int seed=0;
		float min_curvature=FLT_MAX;
		for(set<int>::iterator iter=unlabeled.begin(); iter!=unlabeled.end(); iter++){
			if(normals->at(*iter).curvature<min_curvature){
				seed=*iter;
				min_curvature = normals->at(*iter).curvature;
			}
		}

		queue.push_back(seed);
		cluster.insert(seed);
		unlabeled.erase(seed);
		
		while(!queue.empty()){
			list<int>::iterator curPoint = queue.begin();
			set<int> neighbors = bc->getNeighbors(*curPoint);
			queue.pop_front();

			for(set<int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++){
				set<int>::iterator neighbor = unlabeled.find(*iter);
				if(neighbor!=unlabeled.end()){
					
					float normal_diff = acos((normals->at(*neighbor).normal_x*normals->at(*curPoint).normal_x+
												normals->at(*neighbor).normal_y*normals->at(*curPoint).normal_y+
												normals->at(*neighbor).normal_z*normals->at(*curPoint).normal_z)
												/
												(sqrt(pow(normals->at(*neighbor).normal_x,2)+
														pow(normals->at(*neighbor).normal_y,2)+
														pow(normals->at(*neighbor).normal_z,2))*
												 sqrt(pow(normals->at(*curPoint).normal_x,2)+
														pow(normals->at(*curPoint).normal_y,2)+
														pow(normals->at(*curPoint).normal_z,2))
														));
					
					if(normal_diff<config::normal_diff_threshold){
						cluster.insert(*neighbor);
						unlabeled.erase(neighbor);
						if(normals->at(*curPoint).curvature<config::curvature_threshold)
							queue.push_back(*neighbor);
					}
				}
			}
		}

		if(cluster.size() >= region_bc->cloud->size()/config::min_cluster_size){
			for(set<int>::iterator iter = cluster.begin(); iter!=cluster.end(); iter++){
				region_bc->clustering->at(*iter)=region_bc->cluster_count;
			}
			cout << "Cluster "<<region_bc->cluster_count<<": " << cluster.size() << " data points." << endl;
			region_bc->cluster_count++;
		}

	}
	
	region_bc->clustering_done=true;

	region_bc->color_cloud_from_clustering();

	cout << "Segmentation done." << endl;
}


double compare(Solution *a, Solution *b){

	double total_f1measure=0;

	vector<vector<int>> _a(a->cluster_count+1);	//a+1 because clusters start at 1, label 0 means "doesn't belong to cluster"
	vector<vector<int>> _b(b->cluster_count+1);
	for(int i=0; i<pointCount; i++){
		_a[a->clustering->at(i)].push_back(i);
		_b[b->clustering->at(i)].push_back(i);
	}

	int j=1;
	//iteratr over all clusters in a
	for(vector<vector<int>>::iterator iter=++_a.begin(); iter!=_a.end(); iter++){

		//find cluster in b which has most elements in current cluster of a
		vector<int> hist(iter->size(), 0);
		for(vector<int>::iterator it=iter->begin(); it!=iter->end(); it++){
			hist[b->clustering->at(*it)]++;
		}
		int cluster_b=0;
		int a_and_b=0;
		for(int i=1; i<hist.size(); i++){
			if(hist[i]>a_and_b){
				a_and_b=hist[i];
				cluster_b=i;
			}
		}
		double precision = (double)a_and_b / (double)(_b[cluster_b].size());
		double recall = (double)a_and_b / (double)(iter->size());

		double f1measure;
		if(precision+recall==0)	//divide by 0
			f1measure = 0;
		else
			f1measure = 2*(precision*recall/(precision+recall));

		cout << "Cluster "  << j << ": prec " << precision << " rec " 
			<< recall << " F1 " << f1measure << endl;
		j++;

		total_f1measure+= f1measure*((double)(iter->size())/
			((double)(a->clustering->size()-_a[0].size())));	//weighted
	}

	return total_f1measure;
}


int main (int argc, char** argv)
{
	config::readConfig();

	PointCloud<PointXYZRGB>::Ptr input_cloud(new PointCloud<PointXYZRGB>());

	if(argc<2){
		cout << "usage:" << endl << "	pcs [filename]" << endl << "	(can only read .pcd and .ply)";
		return(0);
	}
	string filename = argv[1];

	bool pcd;	//true if input file is .pcd, false if it's .ply
	//load file
	if(regex_match(filename, regex("(.*)(\.pcd)"))){
		PCDReader reader;
		reader.read(filename, *(input_cloud));
		pcd = true;
		cout << "successfully loaded " << filename << ": " << input_cloud->size() << " points" << endl;
	} else if(regex_match(filename, regex("(.*)(\.ply)"))){
		PLYReader reader;
		reader.read(filename, *(input_cloud));
		pcd = false;
		cout << "successfully loaded " << filename << ": " << input_cloud->size() << " points" << endl;
	} else {
		cout << "usage:" << endl << "	pcs [filename]" << endl << "	(can only read .pcd and .ply)";
		return(0);
	}

	//downsampling (also interpolates colors->bad)
	//input_cloud=filter(input_cloud);
	pointCount = input_cloud->points.size();

	//initialize different solutions
	input = new Solution(input_cloud, "input");
	input->cluster_cloud_from_coloring();
	plane_knn = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "planeKnn");
	plane_bc = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "planeBc");
	region_knn = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "regionKnn");
	region_bc = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "regionBc");


	//construct the boundary complex for filtered input data
	bc = new BoundaryComplex(input->cloud);









	//handle user commands
	string in;
	while(true){
		cin >> in;

		if(string(in)=="showinput"){
			boost::thread(visualize, input);
			continue;
		}
		if(string(in)=="showplaneknn"){
			boost::thread(visualize, plane_knn);
			continue;
		}
		if(string(in)=="showplanebc"){
			boost::thread(visualize, plane_bc);
			continue;
		}
		if(string(in)=="showregionknn"){
			boost::thread(visualize, region_knn);
			continue;
		}
		if(string(in)=="showregionbc"){
			boost::thread(visualize, region_bc);
			continue;
		}



		if(string(in)=="planeknn"){
			planeKnn();
			continue;
		}
		if(string(in)=="planebc"){
			planeBc();
			continue;
		}
		if(string(in)=="regionknn"){
			regionKnn();
			continue;
		}
		if(string(in)=="regionbc"){
			regionBc();
			continue;
		}



		if(string(in)=="saveplaneknn"){
			if(pcd){
				PCDWriter writer;
				stringstream ss;
				ss << "planeKnn_" << filename;
				writer.write<PointXYZRGB> (ss.str (), *(plane_knn->cloud), false);
			} else{
				PLYWriter writer;
				stringstream ss;
				ss << "planeKnn_" << filename;
				writer.write<PointXYZRGB> (ss.str (), *(plane_knn->cloud), false);
			}
			cout << "segmented cloud has been saved to planeKnn_" << filename << endl;
			continue;
		}
		if(string(in)=="saveplanebc"){
			if(pcd){
				PCDWriter writer;
				stringstream ss;
				ss << "planeBc_" << filename;
				writer.write<PointXYZRGB> (ss.str (), *(plane_bc->cloud), false);
			} else{
				PLYWriter writer;
				stringstream ss;
				ss << "planeBc_" << filename;
				writer.write<PointXYZRGB> (ss.str (), *(plane_bc->cloud), false);
			}
			cout << "segmented cloud has been saved to planeBc_" << filename << endl;
			continue;
		}


		if(string(in)=="compareplaneknn"){
			if(!plane_knn->clustering_done)
				planeKnn();
			double f1 = compare(input, plane_knn);
			cout << "F1-Measure of compared solutions: " << f1 << endl;
			continue;
		}
		if(string(in)=="compareplanebc"){
			if(!plane_bc->clustering_done)
				planeBc();
			double f1 = compare(input, plane_bc);
			cout << "F1-Measure of compared solutions: " << f1 << endl;
			continue;
		}
		if(string(in)=="compareregionknn"){
			if(!region_knn->clustering_done)
				regionKnn();
			double f1 = compare(input, region_knn);
			cout << "F1-Measure of compared solutions: " << f1 << endl;
			continue;
		}
		if(string(in)=="compareregionbc"){
			if(!region_bc->clustering_done)
				regionBc();
			double f1 = compare(input, region_bc);
			cout << "F1-Measure of compared solutions: " << f1 << endl;
			continue;
		}


		if(string(in)=="exit"){
			break;
		}

		cout << "valid commands: " << endl <<
			"	planeknn" << endl <<
			"	planebc" << endl <<
			"	regionknn" << endl <<
			"	regionbc" << endl <<

			"	showinput" << endl <<
			"	showplaneknn" << endl <<
			"	showplanebc" << endl <<
			"	showregionknn" << endl <<
			"	showregionbc" << endl <<

			"	saveplaneknn" << endl <<
			"	saveplanebc" << endl <<
			"	saveregionknn" << endl <<
			"	saveregionbc" << endl <<

			"	compareplaneknn" << endl <<
			"	compareplanebc" << endl <<
			"	compareregionknn" << endl <<
			"	compareregionbc" << endl <<

			"	exit" << endl;
	}

	return (0);
}
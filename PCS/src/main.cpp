#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
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
Solution *radius;
Solution *bc;
Solution *knn;

BoundaryComplex *boundaryComplex;


void visualize (Solution *s) {
	visualization::CloudViewer viewer (s->name);
	viewer.showCloud (s->cloud);
	while (!viewer.wasStopped ()){}
}


void makeNoisy(Solution *s){
	srand(time(NULL));
	for(PointCloud<PointXYZRGB>::iterator iter=s->cloud->begin(); iter!=s->cloud->end(); iter++){
		iter->x += (float)(rand()%((int)(s->max_exp*2))-(int)(s->max_exp)) / 1000*config::noise_level;
		iter->y += (float)(rand()%((int)(s->max_exp*2))-(int)(s->max_exp)) / 1000*config::noise_level;
		iter->z += (float)(rand()%((int)(s->max_exp*2))-(int)(s->max_exp)) / 1000*config::noise_level;
	}
}


/*
noch variierbar:
- maximale iterationen für plane segmentation (momentan: 100)
- abbruch der plane-segmentation-schleife (momentan: letzte ebene kleiner als cloud_input->size()/10)
*/
PointCloud<PointXYZRGB>::Ptr ransac(Solution *s){

	PointCloud<PointXYZRGB>::Ptr cloud_filtered (s->cloud), cloud_f(new PointCloud<PointXYZRGB>);

	// Create the segmentation object for the planar model and set all the parameters
	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers (new PointIndices);
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointCloud<PointXYZRGB>::Ptr cloud_plane (new PointCloud<PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (s->max_exp/config::plane_dist_threshold);

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
		
		s->cluster_count++;
		
		for(int i=0; i<cloud_plane->size(); i++){
			//update the cluster-vector..
			while(pos <= indices->at(i)+offset){
				if(s->clustering->at(pos)!=0) offset++;
				pos++;
			}
			s->clustering->at(indices->at(i)+offset)=s->cluster_count;
		}

		cout << "Cluster " << s->cluster_count << ", planar component: " << cloud_plane->points.size () << " data points." << endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered = cloud_f;
	}

	return cloud_filtered;
}


void radiusClustering(Solution *s){

	if(s->clustering_done){
		cout << "Clustering of solution '" << s->name << "' already done." << endl;
		return;
	}

	PointCloud<PointXYZRGB>::Ptr cloud_filtered = ransac(s);

	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud_filtered);	//hier absturz bei manchen clouds
	vector<PointIndices> cluster_indices;
	EuclideanClusterExtraction<PointXYZRGB> ec;
	ec.setClusterTolerance (s->max_exp/config::radius_threshold);//TODO: was gscheiteres
	ec.setMinClusterSize (pointCount/config::min_cluster_size);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	vector<int> cl_euclid_part(cloud_filtered->size());
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		s->cluster_count++;
		int pointcounter=0;
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			pointcounter++;

			//update temporal euclidean clustering vector
			cl_euclid_part[*pit]=s->cluster_count;
		}

		cout << "Cluster "<< s->cluster_count <<": " << pointcounter << " data points." << endl;
	} 

	//add euclid segmentation to plane segmentation
	int offset=0;
	int pos=0;
	for(int i=0; i<cl_euclid_part.size(); i++){
		while(pos <= i+offset){
			if(s->clustering->at(pos)!=0) offset++;
			pos++;
		}
		s->clustering->at(i+offset)=cl_euclid_part[i];
	}

	s->clustering_done=true;

	s->color_cloud_from_clustering();

	cout << "Segmentation done." << endl;
}


void bcClustering(Solution *s){

	if(s->clustering_done){
		cout << "Clustering of solution '" << s->name << "' already done." << endl;
		return;
	}

	ransac(s);
	
	set<int> unlabeled;
	for(int i=0; i<s->clustering->size(); i++)
		if(s->clustering->at(i)==0)
			unlabeled.insert(i);
	
	while(!unlabeled.empty()){
		list<int> queue;
		set<int> cluster;

		set<int>::iterator seed = unlabeled.begin();
		queue.push_back(*seed);
		cluster.insert(*seed);
		unlabeled.erase(seed);
		
		while(!queue.empty()){
			list<int>::iterator curPoint = queue.begin();
			set<int> neighbors = boundaryComplex->getNeighbors(*curPoint);
			queue.pop_front();
			
			//find median distance of neighbors to eliminate outliers
			float medianDistance;
			list<float> distances;
			for(set<int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++){
				distances.push_back(sqrt(pow(s->cloud->at(*curPoint).x - s->cloud->at(*iter).x,2)+
											pow(s->cloud->at(*curPoint).y - s->cloud->at(*iter).y,2)+
											pow(s->cloud->at(*curPoint).z - s->cloud->at(*iter).z,2)));
			}
			distances.sort();
			list<float>::iterator median=distances.begin();
			for(int i=0; i<distances.size()/2; i++){
				median++;
			}
			if(distances.size()%2!=0){
				medianDistance=*median;
			} else {
				float a = *median;
				median--;
				medianDistance = (a + *median)/2;
			}
			
			for(set<int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++){
				set<int>::iterator neighbor = unlabeled.find(*iter);
				if(neighbor!=unlabeled.end()){
					float distance = sqrt(pow(s->cloud->at(*curPoint).x - s->cloud->at(*iter).x,2)+
											pow(s->cloud->at(*curPoint).y - s->cloud->at(*iter).y,2)+
											pow(s->cloud->at(*curPoint).z - s->cloud->at(*iter).z,2));
					if(distance<=medianDistance*config::outliers_threshold){
						queue.push_back(*neighbor);
						cluster.insert(*neighbor);
						unlabeled.erase(neighbor);
					}
				}
			}
		}

		if(cluster.size() >= s->cloud->size()/config::min_cluster_size){
			s->cluster_count++;
			for(set<int>::iterator iter = cluster.begin(); iter!=cluster.end(); iter++){
				s->clustering->at(*iter)=s->cluster_count;
			}
			cout << "Cluster "<<s->cluster_count<<": " << cluster.size() << " data points." << endl;
		}

	}

	s->clustering_done=true;

	s->color_cloud_from_clustering();


	cout << "Segmentation done." << endl;
}

//TODO
void knnClustering(Solution *s){
	/*
	if(s->clustering_done){
		cout << "Clustering of solution '" << s->name << "' already done." << endl;
		return;
	}

	//compute normals
	search::Search<PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);
	PointCloud <Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	NormalEstimation<PointXYZRGB, Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (s->cloud);
	normal_estimator.setKSearch (config::normal_est_k);
	normal_estimator.compute (*normals);

	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZRGB>::Ptr kdTree (new search::KdTree<pcl::PointXYZRGB>);
	kdTree->setInputCloud (s->cloud);

	set<int> unlabeled;
	for(int i=0; i<s->clustering->size(); i++)
		if(s->clustering->at(i)==0)
			unlabeled.insert(i);
	
	s->cluster_count=1;

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
			kdTree->nearestKSearch(s->cloud->at(*curPoint), config::region_growing_k, neighbors, distances); 
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

		if(cluster.size() >= s->cloud->size()/config::min_cluster_size){
			for(set<int>::iterator iter = cluster.begin(); iter!=cluster.end(); iter++){
				s->clustering->at(*iter)=s->cluster_count;
			}
			cout << "Cluster "<<s->cluster_count<<": " << cluster.size() << " data points." << endl;
			s->cluster_count++;
		}

	}
	
	s->clustering_done=true;

	s->color_cloud_from_clustering();

	cout << "Segmentation done." << endl;
	*/
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

		//find cluster in b which has largest intersection with current cluster in a
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

	cout << "Total F1-measure: " << total_f1measure << endl;
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

	//load file
	if(regex_match(filename, regex("(.*)(\.pcd)"))){
		PCDReader reader;
		reader.read(filename, *(input_cloud));
		cout << "successfully loaded " << filename << ": " << input_cloud->size() << " points" << endl;
	} else if(regex_match(filename, regex("(.*)(\.ply)"))){
		PLYReader reader;
		reader.read(filename, *(input_cloud));
		cout << "successfully loaded " << filename << ": " << input_cloud->size() << " points" << endl;
	} else {
		cout << "usage:" << endl << "	pcs [filename]" << endl << "	(can only read .pcd and .ply)";
		return(0);
	}

	pointCount = input_cloud->points.size();

	//initialize different solutions
	input = new Solution(input_cloud, "input");
	makeNoisy(input);
	input->cluster_cloud_from_coloring();
	radius = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "radius");
	bc = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "bc");
	knn = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "knn");


	//construct the boundary complex for filtered input data
	boundaryComplex = new BoundaryComplex(input->cloud);



	//handle user commands
	string in;
	while(true){
		cin >> in;

		if(string(in)=="showinput"){
			boost::thread(visualize, input);
			continue;
		}
		if(string(in)=="showradius"){
			boost::thread(visualize, radius);
			continue;
		}
		if(string(in)=="showbc"){
			boost::thread(visualize, bc);
			continue;
		}
		if(string(in)=="showknn"){
			boost::thread(visualize, knn);
			continue;
		}



		if(string(in)=="radius"){
			radiusClustering(radius);
			continue;
		}
		if(string(in)=="bc"){
			bcClustering(bc);
			continue;
		}
		if(string(in)=="knn"){
			knnClustering(knn);
			continue;
		}



		if(string(in)=="saveradius"){
			PLYWriter writer;
			stringstream ss;
			ss << "radius_" << filename;
			writer.write<PointXYZRGB> (ss.str (), *(radius->cloud), false);

			cout << "segmented cloud has been saved to radius_" << filename << endl;
			continue;
		}
		if(string(in)=="savebc"){
			PLYWriter writer;
			stringstream ss;
			ss << "bc_" << filename;
			writer.write<PointXYZRGB> (ss.str (), *(bc->cloud), false);
			
			cout << "segmented cloud has been saved to bc_" << filename << endl;
			continue;
		}
		if(string(in)=="saveknn"){
			PLYWriter writer;
			stringstream ss;
			ss << "knn_" << filename;
			writer.write<PointXYZRGB> (ss.str (), *(knn->cloud), false);

			cout << "segmented cloud has been saved to knn_" << filename << endl;
			continue;
		}
		


		if(string(in)=="compareradius"){
			if(!radius->clustering_done){
				cout << "You have to do the clustering first!" << endl;
				continue;
			}
			double f1 = compare(input, radius);
			continue;
		}
		if(string(in)=="comparebc"){
			if(!radius->clustering_done){
				cout << "You have to do the clustering first!" << endl;
				continue;
			}
			double f1 = compare(input, bc);
			continue;
		}
		if(string(in)=="compareknn"){
			if(!radius->clustering_done){
				cout << "You have to do the clustering first!" << endl;
				continue;
			}
			double f1 = compare(input, knn);
			continue;
		}


		if(string(in)=="exit"){
			break;
		}

		cout << "valid commands: " << endl <<
			"	radius" << endl <<
			"	bc" << endl <<
			"	knn" << endl <<

			"	showinput" << endl <<
			"	showradius" << endl <<
			"	showbc" << endl <<
			"	showknn" << endl <<

			"	saveradius" << endl <<
			"	savebc" << endl <<
			"	saveknn" << endl <<

			"	compareradius" << endl <<
			"	comparebc" << endl <<
			"	compareknn" << endl <<

			"	exit" << endl;
	}

	return (0);
}
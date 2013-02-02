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
	seg.setDistanceThreshold ((s->max_exp/(double)1000)*(double)config::plane_dist_threshold);

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
		if(cloud_plane->size() < pointCount/config::ransac_break) break;
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
	cout << "radius-clustering; r=" << config::radius_threshold << endl;

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
	ec.setClusterTolerance ((s->max_exp/(double)1000)*(double)config::radius_threshold);//TODO: was gscheiteres
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


float bcClustering(Solution *s){
	cout << "bc-clustering; c=" << config::outliers_threshold << endl;
	
	if(s->clustering_done){
		cout << "Clustering of solution '" << s->name << "' already done." << endl;
		return 0;
	}

	ransac(s);
	
	long neighbor_count=0;
	int processed_points=0;
	int outliers_count=0;

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
			neighbor_count+=neighbors.size();
			processed_points++;
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
					} else outliers_count++;
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

	float avg_neighbors = (float)neighbor_count/(float)processed_points;

	cout << "Segmentation done. Average number of neighbors: " << avg_neighbors 
		<<"   outliers detected: " << outliers_count << endl;

	return avg_neighbors;
}


void knnClustering(Solution *s){
	cout << "knn-clustering; k=" << config::k << endl;
	if(s->clustering_done){
		cout << "Clustering of solution '" << s->name << "' already done." << endl;
		return;
	}

	ransac(s);
	
	//create kd-tree for knn-search
	search::KdTree<PointXYZRGB>::Ptr kdTree (new search::KdTree<pcl::PointXYZRGB>);
	kdTree->setInputCloud (s->cloud);

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

			vector<int> neighbors;
			vector<float> distances;
			kdTree->nearestKSearch(s->cloud->at(*curPoint), config::k, neighbors, distances);
			queue.pop_front();
			
			
			for(vector<int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++){
				set<int>::iterator neighbor = unlabeled.find(*iter);
				if(neighbor!=unlabeled.end()){
					queue.push_back(*neighbor);
					cluster.insert(*neighbor);
					unlabeled.erase(neighbor);
				
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


//a=ground truth, b=computed
double compare(Solution *a, Solution *b){

	vector<vector<int>> _a(a->cluster_count+1);	//a+1 because clusters start at 1, label 0 means "doesn't belong to cluster"
	vector<vector<int>> _b(b->cluster_count+1);
	for(int i=0; i<pointCount; i++){
		_a[a->clustering->at(i)].push_back(i);
		_b[b->clustering->at(i)].push_back(i);
	}


	double total_precision=0;
	double total_recall=0;

	//iterate over all clusters in a
	for(vector<vector<int>>::iterator iter=++_a.begin(); iter!=_a.end(); iter++){

		double cluster_precision=0;
		double cluster_recall=0;

		//count intersections of current cluster of a with clusters of b
		vector<int> intersections(_b.size(), 0);
		for(vector<int>::iterator it=iter->begin(); it!=iter->end(); it++){
			intersections[b->clustering->at(*it)]++;
		}
		
		for(int i=1; i<_b.size(); i++){
			//conditions according to unsupervised f1-paper
			if(_a.size()>1)
				cluster_precision+= (1-(double)(_b.at(i).size()-intersections.at(i)) / 
									(double)(a->clustering->size()-iter->size())) * (double)intersections.at(i); 
																					//last thing for weighted accumulation
			else
				cluster_precision+=(double)intersections.at(i);

			if(iter->size()>1)
				cluster_recall+= ((double)(intersections.at(i)-1) / (double)(iter->size()-1)) * (double)intersections.at(i);
			else
				cluster_recall+= (double)intersections.at(i);
		}
		
		total_precision+=cluster_precision;
		total_recall+=cluster_recall;
	}

	total_precision/=a->clustering->size();
	total_recall/=a->clustering->size();

	double f1 = 2*((total_precision*total_recall)/(total_precision+total_recall));

	cout << "F1-measure: " << f1 << endl;
	return f1;
}


void findBestParameters(string filename, PointCloud<PointXYZRGB>::Ptr input_cloud){
	int best_k=1;
	int best_r=1;
	double best_c=1;

	double best_f1=0;
	
	for(int j=1; j<=30; j++){
		config::k=j;
		knn->~Solution();
		knn = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "knn");
		knnClustering(knn);
		double f1knn = compare(input, knn);
		if(f1knn > best_f1){
			best_f1=f1knn;
			best_k=config::k;
		}
	}
	fstream filestr;
	filestr.open ("k.txt", fstream::in | fstream::out | fstream::app);
	filestr << best_k << "	" << filename <<  endl;
	filestr.close();

	best_f1=0;
	for(int j=1; j<=100; j+=5){
		config::radius_threshold=j;
		radius->~Solution();
		radius = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "radius");
		radiusClustering(radius);
		double f1radius = compare(input, radius);
		if(f1radius > best_f1){
			best_f1=f1radius;
			best_r=config::radius_threshold;
		}
	}
	fstream filestr2;
	filestr2.open ("r.txt", fstream::in | fstream::out | fstream::app);
	filestr2 << best_r << "	" << filename <<  endl;
	filestr2.close();

	best_f1=0;
	for(double j=1; j<=5; j+=0.2){
		config::outliers_threshold=j;
		bc->~Solution();
		bc = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "bc");
		bcClustering(bc);
		double f1bc = compare(input, bc);
		if(f1bc > best_f1){
			best_f1=f1bc;
			best_c=config::outliers_threshold;
		}
	}
	fstream filestr3;
	filestr3.open ("c.txt", fstream::in | fstream::out | fstream::app);
	filestr3 << best_c << "	" << filename <<  endl;
	filestr3.close();

	cout << "done" << endl;
}

void automatic(string filename, PointCloud<PointXYZRGB>::Ptr input_cloud){

	fstream filestr;
	filestr.open ("results.txt", fstream::in | fstream::out | fstream::app);
	//for(int i=0; i<=10; i++){
	//	config::noise_level=i;

	//	knn->~Solution();
	//	knn = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "knn");
	//	makeNoisy(knn);
		knnClustering(knn);
		double f1knn = compare(input, knn);

	//	radius->~Solution();
	//	radius = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "radius");
	//	makeNoisy(radius);
		radiusClustering(radius);
		double f1radius = compare(input, radius);

	//	bc->~Solution();
	//	bc = new Solution(*(new PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>((*input_cloud)))), "bc");
	//	makeNoisy(bc);
		float avg_neighbors = bcClustering(bc);
		double f1bc = compare(input, bc);

		filestr << f1knn <<"\t"<< f1radius <<"\t"<< f1bc <<"\t"<< avg_neighbors 
			<<"\t"<< config::noise_level <<"\t"<< filename << endl;
	//}
	filestr.close();
	cout << "done" << endl;
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
			if(!bc->clustering_done){
				cout << "You have to do the clustering first!" << endl;
				continue;
			}
			double f1 = compare(input, bc);
			continue;
		}
		if(string(in)=="compareknn"){
			if(!knn->clustering_done){
				cout << "You have to do the clustering first!" << endl;
				continue;
			}
			double f1 = compare(input, knn);
			continue;
		}


		if(string(in)=="param"){
			findBestParameters(filename, input_cloud);
			break;
		}
		if(string(in)=="auto"){
			automatic(filename, input_cloud);
			break;
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
			
			"	param" << endl <<
			"	auto" << endl <<
			
			"	exit" << endl;
	}

	return (0);
}
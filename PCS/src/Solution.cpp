#include "Solution.h"


Solution::Solution(PointCloud<PointXYZRGB>::Ptr _cloud){
	cloud = _cloud;
	clustering_done=false;
	clustering = new vector<int>(cloud->size(),0);
	cluster_count=0;

	//calculate maximum expansion of cloud
	PointXYZRGB min;
	PointXYZRGB max;
	getMinMax3D(*cloud, min, max);
	max_exp = std::max(max.x-min.x,std::max(max.y-min.y, max.z-min.z));
}

void Solution::color_cloud_from_clustering(){
	if(!clustering_done){
		cout << "Error: Attempted to color cloud before clustering." << endl;
		return;
	}

	vector<vector<int>> colors(cluster_count);
	srand(time(NULL));
	for(int i=0; i<cluster_count; i++){
		vector<int> c(3);
		c[0]=std::rand()%256;
		c[1]=std::rand()%256;
		c[2]=std::rand()%256;
		colors[i]=c;
	}

	for(int i=0; i<clustering->size(); i++){
		if(clustering->at(i) > 0){
			cloud->points[i].r = colors[clustering->at(i)-1][0];
			cloud->points[i].g = colors[clustering->at(i)-1][1];
			cloud->points[i].b = colors[clustering->at(i)-1][2];
		} else {
			cloud->points[i].r = 0;
			cloud->points[i].g = 0;
			cloud->points[i].b = 0;
		}
	}
}

Solution::~Solution(){
}
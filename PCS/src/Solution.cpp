#include "Solution.h"


Solution::Solution(PointCloud<PointXYZRGB>::Ptr _cloud, string _name){
	cloud = _cloud;
	name = _name;
	clustering_done=false;
	clustering = new vector<int>(cloud->size(),0);
	cluster_count=0;

	//calculate maximum expansion of cloud
	PointXYZRGB min;
	PointXYZRGB max;
	getMinMax3D(*cloud, min, max);
	float expX=max.x-min.x;
	float expY=max.y-min.y;
	float expZ=max.z-min.z;
	max_exp = std::max(expX,std::max(expY, expZ));

	//definition of distance threshold adaption according to spacial expansion and point count
	dist_threshold = pow((double)(expX*expY*expZ),(double)(2/3)) / cloud->size() * config::clusterDistThreshold;
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

void Solution::cluster_cloud_from_coloring(){

	vector<float> known_colors;
	known_colors.push_back(0);	//we need black at position 0 for unclustered points
	int i=0;
	for(PointCloud<PointXYZRGB>::iterator iter=cloud->begin(); iter!=cloud->end(); iter++){

		float color=iter->rgb;

		bool found=false;
		for(int j=0; j<known_colors.size(); j++){
			if(known_colors[j]==color){
				clustering->at(i)=j;
				found=true;
			}
		}
		if(!found){
			known_colors.push_back(color);
			clustering->at(i)=known_colors.size()-1;
			cout << color << endl;
		}
		
		i++;
	}
	
	cluster_count = known_colors.size()-1;
	cout << "Input Cloud: " << cluster_count << " clusters." << endl;
	clustering_done=true;
}

Solution::~Solution(){
}
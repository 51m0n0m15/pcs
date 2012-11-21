#ifndef SOLUTION_H
#define SOLUTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include "config.h"

using namespace std;
using namespace pcl;

class Solution{
public:
	PointCloud<PointXYZRGB>::Ptr cloud;
	vector<int> *clustering;
	bool clustering_done;
	int cluster_count;
	float max_exp;
	float dist_threshold;
	string name;
	
	Solution::Solution(PointCloud<PointXYZRGB>::Ptr _cloud, string _name);
	~Solution();
	void color_cloud_from_clustering();
	void cluster_cloud_from_coloring();
};

#endif
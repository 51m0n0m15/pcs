#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <iostream>
using namespace std;

class config
{
public:

	static int filter_leaf_size;
	static int plane_dist_threshold;
	static int cluster_dist_threshold;
	static int min_cluster_size;
	static float normal_diff_threshold;
	static int curvature_threshold;
	static int normal_est_k;
	static int region_growing_k;

	config(void);
	~config(void);

	static void readConfig();

};

#endif
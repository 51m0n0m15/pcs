#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <iostream>
using namespace std;

class config
{
public:
	static int noise_level;
	static int plane_dist_threshold;
	static int min_cluster_size;
	static int radius_threshold;
	static float outliers_threshold;
	static int k;
	static int ransac_break;


	config(void);
	~config(void);

	static void readConfig();

};

#endif
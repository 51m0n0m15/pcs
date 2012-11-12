#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <iostream>
using namespace std;

class config
{
public:

	static int filterLeafSize;
	static int planeDistThreshold;
	static int clusterDistThreshold;
	static int minClusterSize;

	config(void);
	~config(void);

	static void readConfig();

};

#endif
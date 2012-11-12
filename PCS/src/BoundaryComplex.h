#ifndef BOUNDARY_COMPLEX_H
#define BOUNDARY_COMPLEX_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <set>

using namespace std;

class BoundaryComplex{
public:
	BoundaryComplex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
	~BoundaryComplex();
	void doClustering(vector<int> *clustering, float distThreshold, int clusterNo);
	set<int> getNeighbors(int index);

private:
	void connect3D();
};
#endif
/*
 * Connect3D.h
 *
 *  Created on: 26-Jan-2009
 *      Author: stef
 */

#ifndef CONNECT3D_H_
#define CONNECT3D_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Vector3D.h"

using namespace pcl;

class Connect3D {
private:
	int viewWidth, viewHeight;
	float viewRatio;
	int showMode;
	bool timerFirst;

public:
	Connect3D(PointCloud<PointXYZRGB>::Ptr cloud_in);
	virtual ~Connect3D();
	void reshape(int width, int height);
	void update(int *renderTexture, float phi, float theta, float aperture, float scale, float ofsx, float ofsy, float ofsz);
	void raytrace(int *renderTexture, Vector3D viewOrigin, Vector3D view, Vector3D up, Vector3D left);
	void calculateProjectionMatrix(Vector3D eye, Vector3D center, Vector3D up, float fovy, float aspect, float *m);
	void toggleShow();
	int getShowMode();
	void locateTriangles(int x, int y);
	void connect3D();
};

#endif /* CONNECT3D_H_ */

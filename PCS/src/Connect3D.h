/*
 * Connect3D.h
 *
 *  Created on: 26-Jan-2009
 *      Author: stef
 */

#ifndef CONNECT3D_H_
#define CONNECT3D_H_


#include "Vector3D.h"


class Connect3D {

public:
	//Connect3D(PointCloud<PointXYZRGB>::Ptr cloud_in);
	Connect3D();
	virtual ~Connect3D();
	void connect3D();
};

#endif /* CONNECT3D_H_ */

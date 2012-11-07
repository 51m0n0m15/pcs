#ifndef BASE_H_
#define BASE_H_

#include <vector>

using namespace std;

#define SQR(a) ((a)*(a))
#define MIN(a, b) (((a) < (b)) ? a : b)
#define MAX(a, b) (((a) > (b)) ? a : b)

typedef struct _Triangle *TrianglePtr;

typedef struct
{
	Vector3D vec;
	Vector3D n;
	int tcount;	// TODO: unsigned char?
	vector<TrianglePtr> t;	// TODO: could be derived from triangle/edge structure
} _Vertex;

typedef _Vertex* VertexPtr;

struct _Triangle
{
	_Vertex *v[3];
	Vector3D n;
	bool samplingCrit;
	char status;
};

#endif /*BASE_H_*/

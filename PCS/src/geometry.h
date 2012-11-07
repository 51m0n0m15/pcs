/*
 * geometry.h
 *
 *  Created on: 10-Feb-2009
 *      Author: stef
 */

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include "tnt.h"
#include "jama_qr.h"
#include "jama_eig.h"
#include "Vector3D.h"
using namespace TNT;
using namespace JAMA;
#define PI 3.1415926


/*
 * calculate centroid point of triangle
 */
Vector3D calcCentroid(Vector3D p0, Vector3D p1, Vector3D p2)
{
	return (p0 + p1 + p2)*(1.0/3);
}

/*
 * returns if p1 is on same side as p2 of plane orthogonal to edge a-b with p2 forming the plane
 */
bool sameSide(Vector3D p1, Vector3D p2, Vector3D a, Vector3D b)
{
	Vector3D cp1 = Vector3D::crossProduct(b - a, p1 - a);
	Vector3D cp2 = Vector3D::crossProduct(b - a, p2 - a);

	return Vector3D::dotProduct(cp1, cp2) >= 0.0;
}

/*
 * returns true if point is on the right hand side of the plane spanned by p0, p1, p2 at p0
 */
bool onRightHandSideOfPlane(Vector3D &p, Vector3D &p0, Vector3D &p1, Vector3D &p2)
{
	// TODO: verify sign direction

	Vector3D u = p1 - p0;
	Vector3D v = p2 - p0;
	Vector3D n = Vector3D::crossProduct(u, v);

	// project vector p-p0 on normal n
	Vector3D d = p - p0;

	return Vector3D::dotProduct(d, n) < 0.0;
}

/*
 * returns if point is inside the bounds of orthogonal planes to the triangle's edges
 */
bool isPInTriangle(Vector3D px, Vector3D p0, Vector3D p1, Vector3D p2)
{
	return (sameSide(px, p0, p1, p2) && sameSide(px, p1, p0, p2) && sameSide(px, p2, p0, p1));
}

/*
 * intersect ray (o, vec d) with plane (p, vec u, vec v)
 */
Vector3D projectRayOnPlane(Vector3D o, Vector3D d, Vector3D p, Vector3D u, Vector3D v)
{
	Vector3D m[3] = { u, v, -d };
	Vector3D b = o - p;
//	Vector3D result = solve3x3Matrix(m, b);

	int i;

	Array2D<float> arr = Array2D<float>(3, 3);	// construct matrix for plane from its three vectors

	for (i = 0; i < 3; i++)
	{
		arr[i][0] = m[0][i];
		arr[i][1] = m[1][i];
		arr[i][2] = m[2][i];
	}

	QR<float> qr = QR<float>(arr);
	Array1D<float> arr1 = Array1D<float>(3);

	for (i = 0; i < 3; i++)
		arr1[i] = b[i];

	Array1D<float> r = qr.solve(arr1);	// solve linear system
/*
	if (r == NULL)
		return NULL;
*/
	Vector3D result = Vector3D(r);	// calculate result vector (u, v, n)

	return result;
}

/*
 * rotates point in 3D about two angles a, b
 */
Vector3D rotatePoint3D(Vector3D v, float a, float b) {
	Vector3D vrot;

	float sina = sin(a);
	float cosa = cos(a);
	float sinb = sin(b);
	float cosb = cos(b);

	vrot[0] = -sinb*(cosa*v[1] + sina*v[2]) + cosb*v[0];
	vrot[1] = cosb*(cosa*v[1] + sina*v[2]) + sinb*v[0];
	vrot[2] = -sina*v[1] + cosa*v[2];

	return vrot;
}

/*
 * compares 2 points lexicographically (to their coordinate), return p0 < p1
 * for the case of 2 equal points (not handled), false is returned
 */
bool comparePoints(Vector3D p0, Vector3D p1)
{
	return (p0[0] < p1[0]) ||
		((p0[0] == p1[0]) && (p0[1] < p1[1])) ||
		((p0[0] == p1[0]) && (p0[1] == p1[1]) && (p0[2] < p1[2]));
}

/*****************************************************************************/
/*  from comp.graphics.algorithms                                            */
/*  tricircumcenter3d()   Find the circumcenter of a triangle in 3D.         */
/*                                                                           */
/*  The result is returned both in terms of xyz coordinates and xi-eta       */
/*  coordinates, relative to the triangle's point `a' (that is, `a' is       */
/*  the origin of both coordinate systems).  Hence, the xyz coordinates      */
/*  returned are NOT absolute; one must add the coordinates of `a' to        */
/*  find the absolute coordinates of the circumcircle.  However, this means  */
/*  that the result is frequently more accurate than would be possible if    */
/*  absolute coordinates were returned, due to limited floating-point        */
/*  precision.  In general, the circumradius can be computed much more       */
/*  accurately.                                                              */
/*                                                                           */
/*****************************************************************************/

Vector3D calcCircumcenter(Vector3D a, Vector3D b, Vector3D c)
{
	Vector3D center;
  double xba, yba, zba, xca, yca, zca;
  double balength, calength;
  double xcrossbc, ycrossbc, zcrossbc;
  double denominator;
  double xcirca, ycirca, zcirca;

  /* Use coordinates relative to point `a' of the triangle. */
  xba = b[0] - a[0];
  yba = b[1] - a[1];
  zba = b[2] - a[2];
  xca = c[0] - a[0];
  yca = c[1] - a[1];
  zca = c[2] - a[2];
  /* Squares of lengths of the edges incident to `a'. */
  balength = xba * xba + yba * yba + zba * zba;
  calength = xca * xca + yca * yca + zca * zca;

  /* Cross product of these edges. */
  xcrossbc = yba * zca - yca * zba;
  ycrossbc = zba * xca - zca * xba;
  zcrossbc = xba * yca - xca * yba;

  double div = xcrossbc * xcrossbc + ycrossbc * ycrossbc + zcrossbc * zcrossbc;

  /* Calculate the denominator of the formulae. */
  if (div == 0.0)
	  denominator = 0.0;
  else
	  denominator = 0.5 / div;

  /* Calculate offset (from `a') of circumcenter. */
  xcirca = ((balength * yca - calength * yba) * zcrossbc -
            (balength * zca - calength * zba) * ycrossbc) * denominator;
  ycirca = ((balength * zca - calength * zba) * xcrossbc -
            (balength * xca - calength * xba) * zcrossbc) * denominator;
  zcirca = ((balength * xca - calength * xba) * ycrossbc -
            (balength * yca - calength * yba) * xcrossbc) * denominator;
  center[0] = a[0] + xcirca;
  center[1] = a[1] + ycirca;
  center[2] = a[2] + zcirca;

  return center;
}

/*
 * sort points lexicographically to make sure that for 3 given
 * points the circumcenter is always the same
 */
Vector3D calcCircumcenterDet(Vector3D p0, Vector3D p1, Vector3D p2)
{
	// sort points lexicographically (to their coordinate)
	if (comparePoints(p0, p1))
	{
		Vector3D temp = p0;
		p0 = p1;
		p1 = temp;
	}

	if (comparePoints(p0, p2))
	{
		Vector3D temp = p0;
		p0 = p2;
		p2 = temp;
	}

	if (comparePoints(p1, p2))
	{
		Vector3D temp = p1;
		p1 = p2;
		p2 = temp;
	}

	return calcCircumcenter(p0, p1, p2);
}

/*
 * sort points lexicographically to make sure that for 3 given
 * points the circumcenter is always the same
 */
float calcCircumcenterRadiusDet(Vector3D p0, Vector3D p1, Vector3D p2, Vector3D &cc)
{
	cc = calcCircumcenterDet(p0, p1, p2);

	return Vector3D::distance(cc, p0);
}

/*
 * calculate triangle area
 */
float calcTriangleArea(Vector3D p0, Vector3D p1, Vector3D p2)
{
	return 0.5*Vector3D::distance(p0, p1)*Vector3D::distance(p0, p2);
}

/*
 * calculate triangle semi-perimeter
 */
float calcTriangleSemiPerimeter(Vector3D p0, Vector3D p1, Vector3D p2)
{
	return 0.5*(Vector3D::distance(p0, p1) + Vector3D::distance(p1, p2) + Vector3D::distance(p2, p0));
}

/*
 * calculate triangle aspect ratio
 */
float calcTriangleAspectRatio(Vector3D p0, Vector3D p1, Vector3D p2)
{
	float semiperimeter = calcTriangleSemiPerimeter(p0, p1, p2);
	float inradius = calcTriangleArea(p0, p1, p2)/semiperimeter;
	Vector3D cc;
	float circumradius = calcCircumcenterRadiusDet(p0, p1, p2, cc);

	return 0.5*circumradius/inradius;
}

/*****************************************************************************
 To:             compgeom-discuss@research.bell-labs.com
 Subject:        Re: circumsphere
 Date:           Wed, 1 Apr 98 0:34:28 EST
 From:           Jonathan R Shewchuk <jrs+@cs.cmu.edu>
 *                                                                           */
/*  tetcircumcenter()   Find the circumcenter of a tetrahedron.              */
/*                                                                           */
/*  The result is returned both in terms of xyz coordinates and xi-eta-zeta  */
/*  coordinates, relative to the tetrahedron's point `a' (that is, `a' is    */
/*  the origin of both coordinate systems).  Hence, the xyz coordinates      */
/*  returned are NOT absolute; one must add the coordinates of `a' to        */
/*  find the absolute coordinates of the circumcircle.  However, this means  */
/*  that the result is frequently more accurate than would be possible if    */
/*  absolute coordinates were returned, due to limited floating-point        */
/*  precision.  In general, the circumradius can be computed much more       */
/*  accurately.                                                              */
/*                                                                           */
/*  The xi-eta-zeta coordinate system is defined in terms of the             */
/*  tetrahedron.  Point `a' is the origin of the coordinate system.          */
/*  The edge `ab' extends one unit along the xi axis.  The edge `ac'         */
/*  extends one unit along the eta axis.  The edge `ad' extends one unit     */
/*  along the zeta axis.  These coordinate values are useful for linear      */
/*  interpolation.                                                           */
/*                                                                           */
/*  If `xi' is NULL on input, the xi-eta-zeta coordinates will not be        */
/*  computed.                                                                */
/*                                                                           */
/*****************************************************************************/

Vector3D calcTetrahedronCircumcenter(Vector3D p0, Vector3D p1, Vector3D p2, Vector3D p3)
{
	double a[3];
	double b[3];
	double c[3];
	double d[3];
	double circumcenter[3];
#ifdef OBSOLETE
	double xi[3];
	double eta[3];
	double zeta[3];
#endif
	int i;

	for (i = 0; i < 3; i++)
	{
		a[i] = p0[i];
		b[i] = p1[i];
		c[i] = p2[i];
		d[i] = p3[i];
	}

  double xba, yba, zba, xca, yca, zca, xda, yda, zda;
  double balength, calength, dalength;
  double xcrosscd, ycrosscd, zcrosscd;
  double xcrossdb, ycrossdb, zcrossdb;
  double xcrossbc, ycrossbc, zcrossbc;
  double denominator;
  double xcirca, ycirca, zcirca;

  /* Use coordinates relative to point `a' of the tetrahedron. */
  xba = b[0] - a[0];
  yba = b[1] - a[1];
  zba = b[2] - a[2];
  xca = c[0] - a[0];
  yca = c[1] - a[1];
  zca = c[2] - a[2];
  xda = d[0] - a[0];
  yda = d[1] - a[1];
  zda = d[2] - a[2];
  /* Squares of lengths of the edges incident to `a'. */
  balength = xba * xba + yba * yba + zba * zba;
  calength = xca * xca + yca * yca + zca * zca;
  dalength = xda * xda + yda * yda + zda * zda;
  /* Cross products of these edges. */
  xcrosscd = yca * zda - yda * zca;
  ycrosscd = zca * xda - zda * xca;
  zcrosscd = xca * yda - xda * yca;
  xcrossdb = yda * zba - yba * zda;
  ycrossdb = zda * xba - zba * xda;
  zcrossdb = xda * yba - xba * yda;
  xcrossbc = yba * zca - yca * zba;
  ycrossbc = zba * xca - zca * xba;
  zcrossbc = xba * yca - xca * yba;

  /* Calculate the denominator of the formulae. */
#ifdef EXACT
  /* Use orient3d() from http://www.cs.cmu.edu/~quake/robust.html     */
  /*   to ensure a correctly signed (and reasonably accurate) result, */
  /*   avoiding any possibility of division by zero.                  */
  denominator = 0.5 / orient3d(b, c, d, a);
#else
  /* Take your chances with floating-point roundoff. */
  denominator = 0.5 / (xba * xcrosscd + yba * ycrosscd + zba * zcrosscd);
#endif

  /* Calculate offset (from `a') of circumcenter. */
  xcirca = (balength * xcrosscd + calength * xcrossdb + dalength * xcrossbc) *
           denominator;
  ycirca = (balength * ycrosscd + calength * ycrossdb + dalength * ycrossbc) *
           denominator;
  zcirca = (balength * zcrosscd + calength * zcrossdb + dalength * zcrossbc) *
           denominator;
  circumcenter[0] = xcirca;
  circumcenter[1] = ycirca;
  circumcenter[2] = zcirca;

  return Vector3D(a[0] + circumcenter[0], a[1] + circumcenter[1], a[2] + circumcenter[2]);
#ifdef OBSOLETE
  if (xi != (double *) NULL) {
    /* To interpolate a linear function at the circumcenter, define a    */
    /*   coordinate system with a xi-axis directed from `a' to `b',      */
    /*   an eta-axis directed from `a' to `c', and a zeta-axis directed  */
    /*   from `a' to `d'.  The values for xi, eta, and zeta are computed */
    /*   by Cramer's Rule for solving systems of linear equations.       */
    *xi = (xcirca * xcrosscd + ycirca * ycrosscd + zcirca * zcrosscd) *
          (2.0 * denominator);
    *eta = (xcirca * xcrossdb + ycirca * ycrossdb + zcirca * zcrossdb) *
           (2.0 * denominator);
    *zeta = (xcirca * xcrossbc + ycirca * ycrossbc + zcirca * zcrossbc) *
            (2.0 * denominator);
  }
#endif
}

/*
 * assures to return a valid float value (not NaN) for comparison
 */
double acosSafe(double x)
{
	if (x > 1.0)
		x = 1.0;
	else
	if (x < -1.0)
		x = -1.0;

	return acos(x);
}

/*
 * intersect ray (o, vec d) with plane (p, vec u, vec v)
 */
Vector3D intersectRayPlane(Vector3D o, Vector3D d, Vector3D p, Vector3D u, Vector3D v)
{
	Vector3D m[3] = { -d, u, v };
	Vector3D b = o - p;
//	Vector3D result = solve3x3Matrix(m, b);

	int i;

	Array2D<float> *arr = new Array2D<float>(3, 3);	// construct matrix for plane from its three vectors

	for (i = 0; i < 3; i++)
	{
		(*arr)[i][0] = m[0][i];
		(*arr)[i][1] = m[1][i];
		(*arr)[i][2] = m[2][i];
	}

	QR<float> *qr = new QR<float>(*arr);
	Array1D<float> *arr1 = new Array1D<float>(3);

	for (i = 0; i < 3; i++)
		(*arr1)[i] = b[i];

	Array1D<float> r = qr->solve(*arr1);	// solve linear system
	delete arr;
	delete arr1;
	delete qr;

	Vector3D result = Vector3D(r);	// calculate result vector

	return o + result[0]*d;
//	 return p + result[1]*u + result[2]*v;
}

/*
 * returns the dihedral angle (in form of cos angle) so that angles > 90 degrees are negative
 * between triangle #0 (v0-v1-v2) and triangle #1 (v1-v2-v3) across their shared edge v1-v2
 * does not consider direction, cos angle -1..+1 (angles between 0..180 degrees)
 */
float dihedralAngle(Vector3D v0, Vector3D v1, Vector3D v2, Vector3D v3)
{
	Vector3D n0 = Vector3D::crossProduct(v1 - v0, v2 - v0);
	Vector3D n1 = Vector3D::crossProduct(v2 - v3, v1 - v3);

	// TODO: optimize by just return greater/smaller than 90 degrees (no normalization needed)
	n0.normalize();
	n1.normalize();
	float angle = Vector3D::dotProduct(n0, n1);

//	cout << "dihedralAngle: " << (acosSafe(angle)/PI*180.0) << "(" << v0 << "/" << v1 << "/" << v2 << "/" << v3 << ")" << endl;

	return angle;
}

/*
 * returns the dihedral angle considering direction in degrees from 0..360
 * between triangle #0 (v0-v1-v2) and triangle #1 (v1-v2-v3) across their shared edge v1-v2
 * in direction of the normal of triangle #0
 * TODO: very big precision error: cross product/normalize/dot product/acos -> how to simplify?
 */
float dihedralAngleDir(Vector3D v0, Vector3D v1, Vector3D v2, Vector3D v3, bool direction)
{
	Vector3D n0 = Vector3D::crossProduct(v1 - v0, v2 - v0);
	Vector3D n1 = Vector3D::crossProduct(v2 - v3, v1 - v3);
	Vector3D v0v3 = v3 - v0;

	// TODO: optimize by just return greater/smaller than 90 degrees (no normalization needed)
	n0.normalize();
	n1.normalize();
	v0v3.normalize();
	float cosAngle = Vector3D::dotProduct(n0, n1);

//	cout << "dihedralAngle: " << (acosSafe(angle)/PI*180.0) << "(" << v0 << "/" << v1 << "/" << v2 << "/" << v3 << ")" << endl;

	float angle = acosSafe(cosAngle)/PI*180.0;

	// check if v3 is on same side of triangle v0-v1-v2 as its normal vector
	float dot = Vector3D::dotProduct(n0, v0v3);

	// range of angle is 0 for folded to one side over 180 for straight to 360 to folded to other side
	if (dot >= 0.0)
		angle = 180.0 - angle;
	else
		angle += 180.0;

	// inverse depending on direction
	if (direction)
		angle = 360.0 - angle;

	return angle;
}

/*
 * calculate incenter of a triangle
 */
Vector3D calcIncenter(Vector3D p0, Vector3D p1, Vector3D p2)
{
	int i;
	Vector3D incenter;

    /* Using Heron's formula */
	float d01 = Vector3D::distance(p0, p1);
	float d12 = Vector3D::distance(p1, p2);
	float d20 = Vector3D::distance(p2, p0);
	float perimeter = 1.0f/(d01 + d12 + d20);

	for (i = 0; i < 3; i++)
		incenter[i] = p0[i]*d12 + p1[i]*d20 + p2[i]*d01;

	return perimeter*incenter;
}

/*
 * calculates the two vectors u,v in the plane perpendicular to its vector n
 */
void getPlaneVectors(Vector3D n, Vector3D &u, Vector3D &v)
{
	if (n[0] == 0.0)
		u = Vector3D(0.0, n[2], -n[1]);
	else
	if (n[1] == 0.0)
		u = Vector3D(n[2], 0.0, -n[0]);
	else
		u = Vector3D(n[1], -n[0], 0.0);

	v = Vector3D::crossProduct(n, u);
}

/*
 * calculates the intersection of 3 planes (given by point and normal vector)
 */
Vector3D intersect3Planes(Vector3D p0, Vector3D n0, Vector3D p1, Vector3D n1, Vector3D p2, Vector3D n2)
{
	float d0 = Vector3D::dotProduct(n0, p0);
	float d1 = Vector3D::dotProduct(n1, p1);
	float d2 = Vector3D::dotProduct(n2, p2);
	float divisor = Vector3D::dotProduct(n0, Vector3D::crossProduct(n1, n2));

	return (d0*Vector3D::crossProduct(n1, n2) + d1*Vector3D::crossProduct(n2, n0) + d2*Vector3D::crossProduct(n0, n1))/divisor;
}
#endif /* GEOMETRY_H_ */
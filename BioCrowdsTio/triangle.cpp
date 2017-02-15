#include "stdafx.h"
#include "triangle.h"

#include <assert.h>
#include <math.h>

#include "delaunay.h"

Triangle::Triangle(const Vector3 &_p1, const Vector3 &_p2, const Vector3 &_p3)
	: p1(_p1), p2(_p2), p3(_p3),
	e1(_p1, _p2), e2(_p2, _p3), e3(_p3, _p1)
{}

bool Triangle::containsVertex(const Vector3 &v)
{
	return p1 == v || p2 == v || p3 == v;
}

bool Triangle::circumCircleContains(const Vector3 &v)
{
	float ab = (p1.x * p1.x) + (p1.z * p1.z);
	float cd = (p2.x * p2.x) + (p2.z * p2.z);
	float ef = (p3.x * p3.x) + (p3.z * p3.z);

	float circum_x = (ab * (p3.z - p2.z) + cd * (p1.z - p3.z) + ef * (p2.z - p1.z)) / (p1.x * (p3.z - p2.z) + p2.x * (p1.z - p3.z) + p3.x * (p2.z - p1.z)) / 2.f;
	float circum_y = (ab * (p3.x - p2.x) + cd * (p1.x - p3.x) + ef * (p2.x - p1.x)) / (p1.z * (p3.x - p2.x) + p2.z * (p1.x - p3.x) + p3.z * (p2.x - p1.x)) / 2.f;
	float circum_radius = sqrtf(((p1.x - circum_x) * (p1.x - circum_x)) + ((p1.z - circum_y) * (p1.z - circum_y)));

	float dist = sqrtf(((v.x - circum_x) * (v.x - circum_x)) + ((v.z - circum_y) * (v.z - circum_y)));
	return dist <= circum_radius;
}

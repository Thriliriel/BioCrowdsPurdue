#pragma once
#ifndef H_DELAUNAY
#define H_DELAUNAY

#include "triangle.h"

#include <vector>

class Delaunay
{
public:
	const std::vector<Triangle>& triangulate(std::vector<Vector3> &vertices);
	const std::vector<Triangle>& getTriangles() const { return _triangles; };
	const std::vector<Edge>& getEdges() const { return _edges; };
	const std::vector<Vector3>& getVertices() const { return _vertices; };

private:
	std::vector<Triangle> _triangles;
	std::vector<Edge> _edges;
	std::vector<Vector3> _vertices;
};

#endif

#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	bool rayIntersect(Ray& r, float& t)
	{
		float denominator = n.dot(r.dir);

		if (abs(denominator) > 1e-6)           
		{
			t = (d - n.dot(r.o)) / denominator;
			return (t >= 0);                 

		}

		return false;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0)
		{
			return false;
		}

		t = (d - Dot(n, r.o)) / denom;
		if (t < 0)
		{
			return false;
		}

		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;

		if (u < 0 || u > 1.0f)
		{
			return false;
		}

		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;

		if (v < 0 || (u + v) > 1.0f)
		{
			return false;
		}

		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();

		float sqrtR1 = sqrtf(r1);
		float u = 1.0f - sqrtR1;
		float v = r2 * sqrtR1;
		float w = 1.0f - (u + v);

		Vec3 point = (vertices[0].p * w) + (vertices[1].p * u) + (vertices[2].p * v);

		pdf = 1.0f / area;

		return point;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 Tmin = (min - r.o) * r.invDir;
		Vec3 Tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(Tmin, Tmax);
		Vec3 Texit = Max(Tmin, Tmax);
		float tentry = std::max(Tentry.x, std::max(Tentry.y, Tentry.z));
		float texit = std::min(Texit.x, std::min(Texit.y, Texit.z));
		t = std::min(tentry, texit);
		return (tentry <= texit && texit > 0);
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 s = (min - r.o) * r.invDir;
		Vec3 l = (max - r.o) * r.invDir;
		Vec3 s1 = Min(s, l);
		Vec3 l1 = Max(s, l);
		float ts = std::max(s1.x, std::max(s1.y, s1.z));
		float tl = std::min(l1.x, std::min(l1.y, l1.z));
		return (ts <= tl && tl > 0);
	}

	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{

		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset;
	unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, int start, int end)
	{
		if (inputTriangles.empty()) {
			return;
		}

		//find side of triangle with longets dimension
		//use mid split

		//in box
		bounds.reset();
		for (int i = start; i < end; i++) {
			bounds.extend(inputTriangles[i].vertices[0].p);
			bounds.extend(inputTriangles[i].vertices[1].p);
			bounds.extend(inputTriangles[i].vertices[2].p);
		}

		int count = end - start;

		//stop split if low count
		if (count <= MAXNODE_TRIANGLES) {
			offset = start;
			num = count;
			return;
		}


		Vec3 size = bounds.max - bounds.min;
		//start x axis
		int axis = 0;
		//change to y or z
		if (size.y > size.x) axis = 1;
		if (size.z > size.y) axis = 2;

		//sort by center point on axis
		std::sort(inputTriangles.begin() + start, inputTriangles.begin() + end,
			[axis](const Triangle& a, const Triangle& b) {
				if (axis == 0) {
					return a.centre().x < b.centre().x;
				}
				if (axis == 1) {
					return a.centre().y < b.centre().y;
				}
				return a.centre().z < b.centre().z;
			});


		//split list
		int mid = start + count / 2;

		num = 0;
		l = new BVHNode();
		r = new BVHNode();

		//split l r half
		l->build(inputTriangles, start, mid);
		r->build(inputTriangles, mid, end);
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		//check bounds
		//bound box
		float bBox;

		if (!bounds.rayAABB(ray, bBox) or bBox >= intersection.t) {
			//if box not hit return 
			return;
		}

		if (num > 0) {
			for (int i = 0; i < num; i++) {
				//distance, point1,2
				float dis, u, v;

				//if ray intersect
				if (triangles[offset + i].rayIntersect(ray, dis, u, v)) {
					//closest upd
					if (dis < intersection.t) {
						intersection.t = dis;
						intersection.ID = offset + i;

						intersection.alpha = 1.0f - (u + v);
						intersection.beta = u;
						intersection.gamma = v;
					}
				}
			}
		}

		else {
			//check small pts
			if (l != NULL) {
				l->traverse(ray, triangles, intersection);
			}

			if (r != NULL) {
				r->traverse(ray, triangles, intersection);
			}
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		float bBox;
		if (!bounds.rayAABB(ray, bBox) or bBox >= maxT) {
			return true;
		}

		if (num > 0) {
			for (int i = 0; i < num; i++) {
				float dis, u, v;
				if (triangles[offset + i].rayIntersect(ray, dis, u, v)) {
					if (dis < maxT) {
						return false;
					}
				}
			}
		}
		else {
			if (l != NULL && !l->traverseVisible(ray, triangles, maxT)) {
				return false;
			}

			if (r != NULL && !r->traverseVisible(ray, triangles, maxT)) {
				return false;
			}
		}
		return true;
	}
};

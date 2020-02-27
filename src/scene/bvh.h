#pragma once
#include "scene.h"
#include "ray.h"
#include "bbox.h"
#include <iostream>

#define STACK 512
#define BIG 1000000.0

struct stackNode{
	int pos;
	int start;
	int end;
};

struct node{
	int offset;
	int num;
	int start;
	BoundingBox bb;
};

struct travNode{
	int i;
	double mint;
	travNode();
	travNode(int i2, double mint2) : i(i2), mint(mint2){}
};

class Geometry;
class Scene;
class MaterilaSceneObject;
class Trimesh;
class TrimsehFace;

class BVH{
public:
	BVH(const Scene* s){
		this->scene = s;
	}
	~BHV(){
		delete[] tree;
	}
	void build();
	bool getIntersection(ray& r, isect& i) const;
private:
	int nodeNum;
	int leaf;
	std::vector<Geometry> objlist;
	std::vector<BoundingBox> boxes;
	node* tree;
	const Scene* scene;

}
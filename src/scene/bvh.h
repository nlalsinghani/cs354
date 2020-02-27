#pragma once
#include "scene.h"
#include "ray.h"
#include "bbox.h"
#include <iostream>

#define STACK 512

class Trimesh;
class TrimsehFace;
class Geometry;
class Scene;
class MaterilaSceneObject;


struct stackNode{
	int pos;
	int start;
	int end;
};

struct node{
	int offset;
	int num;
	int start;
	BoundingBox box;
};

struct travNode{
	int i;
	double mint;
	travNode(){}
	travNode(int i2, double mint2) : i(i2), mint(mint2){}
};


class BVH{
public:
	BVH(const Scene* s){
		this->scene = s;
	}
	~BVH(){
		delete[] tree;
	}
	void build();
//	bool getIntersection(ray& r, isect& i);
private:
	int nodeNum;
	int leafs;
	std::vector<Geometry*> objects;
	std::vector<BoundingBox*> boxes;
	node* tree;
	const Scene* scene;

};
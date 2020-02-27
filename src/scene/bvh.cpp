#include "bvh.h"
#include "../SceneObjects/trimesh.h"

void BVH::build(){

  for(auto iter = scene -> beginObjects(); iter != scene -> endObjects(); iter++){
    Geometry* geo = iter->get();
      Trimesh* trimesh = (Trimesh*) geo;
      for(auto face = trimesh->beginFace(); face != trimesh->endFace(); face++){
        (*face)->ComputeBoundingBox();
        objects.push_back((Geometry*)(*face));
      
    }
  }
  nodeNum = 0;
  leafs = 0;

  if(objects.size() == 0)
    return;

  stackNode stack[STACK];
  int stackptr = 0;

  stack[stackptr].start = 0;
  stack[stackptr].end = objects.size();
  stack[stackptr].pos = -1;
  stackptr++;

  std::vector<node> nodes;
  nodes.resize(objects.size() * 2);

  while(stackptr > 0) {

    stackptr--;
    stackNode &crtStackNode( stack[stackptr]);
    int start = crtStackNode.start;
    int end = crtStackNode.end;
    int num = end - start;

    node crtNode;
    crtNode.start = start;
    crtNode.num = num;
    crtNode.offset = -1;

    BoundingBox bb = objects[start]->getBoundingBox();
    BoundingBox bc(bb.getCenter());
    for(int p = start + 1; p < end; p++){
      BoundingBox temp = objects[p] -> getBoundingBox();
      bb.merge(temp);
      bc.merge(temp.getCenter());
    }
    crtNode.box = bb;

    if(num <= 1){
      crtNode.offset = 0;
      leafs++;
    }

    nodes[nodeNum] = crtNode;
    nodeNum ++;

    if(crtStackNode.pos != -1){
      nodes[crtStackNode.pos].offset --;
      if( nodes[crtStackNode.pos].offset == -3){
        nodes[crtStackNode.pos].offset = nodeNum - 1 - crtStackNode.pos;
      }
    }

    int idx = bc.maxDim();
    double split = 0.5 * (bc.getMax()[idx] + bc.getMin()[idx]);

    int mid = start;
    for(int i = start; i < end; i++){
      BoundingBox temp = objects[i] -> getBoundingBox();
      if( temp.getCenter()[idx] < split){
        std::swap(objects[i], objects[mid]);
        mid++;
      }
    }

    if(mid == start || mid == end){
      mid = start + (end - start) / 2;
    }
    stack[stackptr].start = mid;
    stack[stackptr].end = end;
    stack[stackptr].pos = nodeNum - 1;
    stackptr++;

    stack[stackptr].start = start;
    stack[stackptr].end = mid;
    stack[stackptr].pos = nodeNum - 1;
    stackptr++;
  }
  
  tree = new node[nodeNum];
  for(int i = 0; i < nodeNum; i++)
    tree[i] = nodes[i];
}

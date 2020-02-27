#include "bvh.h"
#include "../SceneObjects/trimesh.h"

void BVH::build(){

  for(auto iter = scene -> beginObjects(); iter != scene -> endObjects(); iter++){
    Geometry* geo = iter->get();
    if(geo->isTrimesh()){
      Trimesh* trimesh = (Trimesh*) geo;
      for(auto face = trimesh->beginFace(); face != trimesh->endFace(); face++){
        (*face)->ComputeBoundingBox();
        objects.push_back((Geometry*)(*face));
      }
    }else{
      objects.push_back(geo);
    }
  }
  nodeNum = 0;
  leafs = 0;

  if(objects.size() == 0)
    return;

  stackNode stack[STACK_SIZE];
  int stackptr = 0;

  stack[stackptr].start = 0;
  stack[stackptr].end = objects.size();
  stack[stackptr].pos = -1;
  stackptr++;

  std::vector<node> nodes;
  nodes.resize(objects.size() * 2);

  while(stackptr > 0) {
    if(stackptr >= STACK_SIZE - 1){
      std::cout << "Stack Overflow in BVH::build()" << std::endl;
    }
    if(nodeNum >= objects.size() * 2){
      std::cout<< "over stack size" << std::endl;
    }
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

    if (crtNode.offset == 0){
      continue;
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

bool BVH::getIntersection(ray& r, isect& i) const{
  if(nodeNum == 0)
    return false;
  double t = BIG_DOUBLE;
  travNode stack[STACK_SIZE];
  int stackptr = 0;

  stack[stackptr].i = 0;
  stack[stackptr].mint = -BIG_DOUBLE;
  int cnt = 0;
  while(stackptr >= 0 ){
    if(stackptr >= STACK_SIZE - 1){
      std::cout << "Stack Overflow in BVH::getIntersection()" << std::endl;
    }
    int oi = stack[stackptr].i;
    double near = stack[stackptr].mint;
    stackptr--;

    const node &crtNode(tree[oi]);

    if(near > t)
      continue;
    if(crtNode.offset < 0)
      std::cout<<"Negtive offset" <<std::endl;

    if(crtNode.offset == 0){
      cnt++;
      for(int j = 0; j < crtNode.num; j++){
        const Geometry* obj = objects[crtNode.start + j];
        isect temp;
        if(obj->intersect(r, temp) && temp.getT() < t){

          i = temp;
          t = temp.getT();
        }
      }
    }else{
      double t_min_r, t_max_r;
      bool hitRight = tree[oi + crtNode.offset].box.intersect(r, t_min_r, t_max_r);
      double t_min_l, t_max_l;
      bool hitLeft = tree[oi + 1].box.intersect(r, t_min_l, t_max_l);

      if(hitRight && hitLeft){
        if(t_min_r < t_min_l){
          stack[++stackptr] = travNode(oi + 1, t_min_l);
          stack[++stackptr] = travNode(oi + crtNode.offset, t_min_r);
        }else{
          stack[++stackptr] = travNode(oi + crtNode.offset, t_min_r);
          stack[++stackptr] = travNode(oi + 1, t_min_l);
        }
      }
      else if(hitRight)
        stack[++stackptr] = travNode(oi + crtNode.offset, t_min_r);
      else if(hitLeft)
        stack[++stackptr] = travNode(oi + 1, t_min_l);
    }

  }
  return t < BIG_DOUBLE - 0.1;
}
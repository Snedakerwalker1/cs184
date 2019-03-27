#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c, alpha);
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c, alpha);
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox centroid_box, bbox;
  Vector3D centroidaverage = Vector3D(0., 0., 0.);
  for (Primitive *p : prims) {
      BBox bb = p->get_bbox();
      bbox.expand(bb);
      Vector3D c = bb.centroid();
	  centroidaverage += c;
      centroid_box.expand(c);
  }
  BVHNode *node = new BVHNode(bbox);
  if (prims.size() <= max_leaf_size) {
	  node->prims = new vector<Primitive *>(prims);
	  return node;
  }
  else {
	  //split the node and make a tree!!
	  int maxdim = 0;
	  int i = 0;
	  double max_val = 0;
	  while (i < 3) {
		  if (bbox.extent[i] > max_val) {
			  maxdim = i;
			  max_val = bbox.extent[i];
		  }
		  i += 1;
	  }
	  centroidaverage = centroidaverage / ((double) prims.size());
	  double midpoint = centroidaverage[maxdim];
	  //now lets make our lists; 
	  vector<Primitive *> left;
	  vector<Primitive *> right;
	  for (Primitive *p : prims) {
		  if (p->get_bbox().centroid()[maxdim] <= midpoint) {
			  left.push_back(p);
		  }
		  else {
			  right.push_back(p);
		  }
	  }
	  if (left.size() == 0 || right.size() == 0) {
		  int count = 0;
		  for (Primitive *p : prims) {
			  if (count <= prims.size() / 2.0) {
				  left.push_back(p);
			  }
			  else {
				  right.push_back(p);
			  }
			  count += 1;
		  }

	  }
	  node->l = construct_bvh(left, max_leaf_size);
	  node->r = construct_bvh(right, max_leaf_size);
	  return node;
  }  
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.


	double t0 = ray.min_t;
	double t1 = ray.max_t;
	if (node->bb.intersect(ray, t0, t1)) {
		if (node->isLeaf()) {
			for (Primitive *p : *(node->prims)) {
				total_isects++;
				if (p->intersect(ray))
					return true;
			}
			return false;
		}
		else {
			return  intersect(ray, node->l) || intersect(ray, node->r);
		}
	}
	else {
		return false;
	}
}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

	// TODO (Part 2.3):
	// Fill in the intersect function.
	double t0 = ray.min_t;
	double t1 = ray.max_t;
	if (node->bb.intersect(ray, t0, t1)) {
		if (node->isLeaf()) {
			bool hit = false;
			for (Primitive *p : *(node->prims)) {
				total_isects++;
				if (p->intersect(ray, i))
					hit = true;
			}
			return hit;
		}
		else {
			bool lbool = intersect(ray, i, node->l);
			bool rbool = intersect(ray, i, node->r);
			return  lbool || rbool;
		}
	}
	else {
		return false;
	} 
}

}  // namespace StaticScene
}  // namespace CGL

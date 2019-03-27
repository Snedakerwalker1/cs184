#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  //each plane is given as a point on that plane and the normal N coming out of that plane
	
	//first lets check the x planes.
	double tpxc = (this->min.x - r.o.x)/r.d.x;
	double tpxf = (this->max.x - r.o.x)/r.d.x;
	double tpyc = (this->min.y - r.o.y)/r.d.y;
	double tpyf = (this->max.y - r.o.y)/r.d.y;
	double tpzc = (this->min.z - r.o.z)/r.d.z;
	double tpzf = (this->max.z - r.o.z)/r.d.z;
	double minx = std::min(tpxc, tpxf);
	double maxx = std::max(tpxc, tpxf);
	double miny = std::min(tpyc, tpyf);
	double maxy = std::max(tpyc, tpyf);
	double minz = std::min(tpzc, tpzf);
	double maxz = std::max(tpzc, tpzf);
	double tmin = std::max(minx, std::max(miny, std::max(minz, std::max(r.min_t, t0))));
	double tmax = std::min(maxx, std::min(maxy, std::min(maxz, std::min(r.max_t, t1))));
	if ((tmax - tmin) >= 0) {
		t0 = tmin;
		t1 = tmax;
		return true;
	} 
	else {
		return false;
	}

  
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL

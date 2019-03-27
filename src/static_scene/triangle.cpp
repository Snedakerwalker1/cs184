#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  BBox bb(p1);
  bb.expand(p2); 
  bb.expand(p3);
  return bb;

}

bool Triangle::intersect(const Ray& r) const {

  // TODO (Part 1.3):
  // implement ray-triangle intersection

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  Vector3D E1 = p2 - p1;
  Vector3D E2 = p3 - p1;
  Vector3D S = r.o - p1;
  Vector3D S1 = cross(r.d, E2);
  Vector3D S2 = cross(S, E1);
  Vector3D MT = Vector3D(dot(S2, E2), dot(S1, S), dot(S2, r.d));
  MT = MT / (dot(S1, E1));
  double barry3 = 1.0 - MT.y - MT.z;
  if (MT.y <= 1 && MT.y >= 0 && MT.z <= 1 && MT.z >= 0 && barry3 <= 1 && barry3 >= 0 && MT.x <= r.max_t && MT.x >= r.min_t) {
	  r.max_t = MT.x;
      return true;
  }
  else {
	  return false;
  }
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {
  
  // TODO (Part 1.3):
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  Vector3D n1(mesh->normals[v1]), n2(mesh->normals[v2]), n3(mesh->normals[v3]);
  Vector3D E1 = p2 - p1;
  Vector3D E2 = p3 - p1;
  Vector3D S = r.o - p1;
  Vector3D S1 = cross(r.d, E2);
  Vector3D S2 = cross(S, E1);
  Vector3D MT = Vector3D(dot(S2, E2), dot(S1, S), dot(r.d, S2));
  MT = MT/(dot(S1,E1));
  //&& MT.y <= 1 && MT.y >= 0 && MT.z <= 1 && MT.z >= 0
  double barry3 = 1.0 - MT.y - MT.z;
  if (MT.y <= 1 && MT.y >= 0 && MT.z <= 1 && MT.z >= 0 && barry3 >= 0 && barry3 <= 1 && MT.x <= r.max_t && MT.x >= r.min_t) {
	  r.max_t = MT.x;
	  isect->t = MT.x;
	  isect->n = barry3*n1 + MT.y*n2 + MT.z*n3;
	  isect->primitive = this;
	  isect->bsdf = get_bsdf();
	  return true;
  }
  else {
	  //r.max_t = MT.x;
	  return false;
  }  
  return false;
}

void Triangle::draw(const Color& c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CGL

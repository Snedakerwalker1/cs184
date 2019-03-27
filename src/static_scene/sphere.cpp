#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
	double a = dot(r.d, r.d);
	Vector3D dif = r.o - this->o;
	double b = 2 * dot(dif, r.d);
	double c = dot(dif, dif) - (this->r)*(this->r);
	if ((b*b - 4 * a*c) < 0) {
		return false;
	}
	else {
		double t1a = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
		double t2a = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
		if (t2a > r.max_t || t2a < r.min_t) {
			return false;
		}
		t1 = t2a;
		t2 = t1a;
		return true;
	}
}

bool Sphere::intersect(const Ray& r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
	double t1 = 0.0;
	double t2 = 0.0;
	if (test(r, t1, t2)) {
		r.max_t = t1;
		return true;
	}
	else {
		return false;
	}
}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
	double t1 = 0.0;
	double t2 = 0.0;
	if (test(r, t1, t2)) {
		r.max_t = t1;
		i->t = t1;
		Vector3D n = r.o + r.d*t1 - this->o;
		n.normalize();
		i->n = n;
		i->primitive = this;
		i->bsdf = get_bsdf();
		return true;
	}
	else {
		return false;
	}
}

void Sphere::draw(const Color& c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL

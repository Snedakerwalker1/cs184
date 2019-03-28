#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
  else h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: 1.2
  // Using BSDF::reflect(), implement sample_f for a mirror surface
	*pdf= 1;
	reflect(wo,wi);
	return reflectance/(abs_cos_theta(*wi));
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
    return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: 2.2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
	float dh = exp(-(sin_theta(h)*sin_theta(h)) / (cos_theta(h)*cos_theta(h)*alpha*alpha));
	dh = dh / (PI *(alpha*alpha) *(cos_theta(h)*cos_theta(h)*cos_theta(h)*cos_theta(h)));
	return dh;
}

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: 2.3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
	
	float rsr = ((eta.r*eta.r+ k.r*k.r) - 2. * eta.r*cos_theta(wi) + cos_theta(wi)*cos_theta(wi)) / ((eta.r*eta.r+ k.r * k.r) + 2. * eta.r*cos_theta(wi) + cos_theta(wi)*cos_theta(wi));
	float rpr = ((eta.r*eta.r + k.r * k.r)*cos_theta(wi)*cos_theta(wi) - 2. * eta.r*cos_theta(wi) + 1.) / ((eta.r*eta.r + k.r * k.r)*cos_theta(wi)*cos_theta(wi) + 2. * eta.r*cos_theta(wi) + 1.);
	float rsg = ((eta.g*eta.g + k.g*k.g) - 2. * eta.g*cos_theta(wi) + cos_theta(wi)*cos_theta(wi)) / ((eta.g*eta.g + k.g * k.g) + 2. * eta.g*cos_theta(wi) + cos_theta(wi)*cos_theta(wi));
	float rpg = ((eta.g*eta.g + k.g * k.g)*cos_theta(wi)*cos_theta(wi) - 2. * eta.g*cos_theta(wi) + 1.) / ((eta.g*eta.g + k.g * k.g)*cos_theta(wi)*cos_theta(wi) + 2. * eta.g*cos_theta(wi) + 1.);
	float rsb = ((eta.b*eta.b + k.b*k.b) - 2. * eta.b*cos_theta(wi) + cos_theta(wi)*cos_theta(wi)) / ((eta.b*eta.b + k.b * k.b) + 2. * eta.b*cos_theta(wi) + cos_theta(wi)*cos_theta(wi));
	float rpb = ((eta.b*eta.b + k.b * k.b)*cos_theta(wi)*cos_theta(wi) - 2. * eta.b*cos_theta(wi) + 1.) / ((eta.b*eta.b+ k.b * k.b)*cos_theta(wi)*cos_theta(wi) + 2. * eta.b*cos_theta(wi) + 1.);
	float r = (rsr + rpr) / 2.0;
	float g = (rsg + rpg) / 2.0;
	float b = (rsb + rpb) / 2.0;
	return Spectrum(r, g, b);
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
	// TODO: 2.1
	// Implement microfacet model here
	Vector3D h = wo + wi;
	h.normalize();
	Spectrum val = F(wi)*G(wo, wi)*D(h);
	val = val / (4.*(wo.z)*(wi.z));
	return val;
}


Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: 2.4
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.
	*wi = cosineHemisphereSampler.get_sample(pdf); //placeholder
	return MicrofacetBSDF::f(wo, *wi);
	Vector2D r = this->sampler.get_sample();
	float thetaH = atan(sqrt(-alpha*alpha*log(1. - r.x)));
	float phiH = 2. * PI*r.y;
	// may be wrong idk
	Vector3D h = Vector3D(sin(thetaH)*cos(phiH), sin(thetaH)*sin(phiH), cos(thetaH));
	h.normalize();
	*wi = 2.*dot(wo, h)*h - wo;
	if (wo.z > 0 && wi->z > 0) {
		float ptheta = 2. * sin_theta(h)*exp(-sin_theta(h)*sin_theta(h) / (alpha*alpha*cos_theta(h)*cos_theta(h))) / (alpha*alpha*cos_theta(h)*cos_theta(h)*cos_theta(h));
		float pphi = 1./(2.*PI);
		float pwh = ptheta * pphi / (sin_theta(h));
		*pdf = pwh/(4. * dot(*wi, h));
		return MicrofacetBSDF::f(wo, *wi);
	}
	else if (wo.z > 0 && wi->z <= 0) {
		*pdf = 0;
		return Spectrum();
	}
	else {
		*pdf = 0;
		return Spectrum();
	}
}

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 1.4
  // Compute Fresnel coefficient and either reflect or refract based on it.
	if (!refract(wo,wi, this->ior)) {
		reflect(wo, wi);
		*pdf = 1;
		return reflectance / (abs_cos_theta(*wi));
	}
	float schlR = (1 - this->ior)/(1 + this->ior);
	float R = schlR*schlR;
	if (coin_flip(R)) {
		reflect(wo, wi);
		*pdf = R;
		return R * reflectance / (abs_cos_theta(*wi));
	}
	else {
		refract(wo, wi, this->ior);
		*pdf = 1 - R;
		float eta;
		if (wo.z < 0) {
			//wo starts out inside
			eta = ior;
		}
		else {
			eta = 1. / ior;
		}
		return (1. - R)*transmittance / abs_cos_theta(*wi) / (eta*eta) ;
	}
	return Spectrum();
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO: 1.1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
	wi->x = -1*wo.x;
	wi->y = -1 * wo.y;
	wi->z = wo.z;
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO: 1.3
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
	float n;
	int mul;
	if (wo.z < 0) {
		//wo starts out inside
		n = ior;
		mul = 1;
	}
	else {
		n = 1. / ior;
		mul = -1;
	}
	float val = 1 - n * n*(1 - (wo.z*wo.z));
	if (val < 0) {
		return false;
	}
	wi->x = -n * wo.x;
	wi->y = -n * wo.y;
	wi->z = mul * sqrt(val);
	return true;
}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL

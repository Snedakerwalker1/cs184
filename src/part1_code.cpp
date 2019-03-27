//
// TODO: Copy over 3-1 code after turning on BUILD_3-1 flag
//

#include "part1_code.h"
#include <time.h>

using namespace CGL::StaticScene;

using std::min;
using std::max;

namespace CGL {

  Spectrum PathTracer::estimate_direct_lighting_hemisphere(const Ray& r, const Intersection& isect) {
	  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere. 

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
	  Matrix3x3 o2w;
	  make_coord_space(o2w, isect.n);
	  Matrix3x3 w2o = o2w.T();

	  // w_out points towards the source of the ray (e.g.,
	  // toward the camera if this is a primary ray)
	  const Vector3D& hit_p = r.o + r.d * isect.t;
	  const Vector3D& w_out = w2o * (-r.d);

	  // This is the same number of total samples as estimate_direct_lighting_importance (outside of delta lights). 
	  // We keep the same number of samples for clarity of comparison.
	  int num_samples = scene->lights.size() * ns_area_light;
	  Spectrum L_out = Spectrum();

	  // TODO (Part 3.2): 
	  // Write your sampling loop here
	  // COMMENT OUT `normal_shading` IN `est_radiance_global_illumination` BEFORE YOU BEGIN
	  int i = 0;
	  float pdf = 1.0 / (2.0 * PI);
	  while (i < num_samples) {
		  Vector3D sample_local = hemisphereSampler->get_sample();
		  Vector3D sample_world = o2w * sample_local;
		  Vector3D ofset = hit_p + (EPS_D)*sample_world;
		  Ray sample_ray = Ray(ofset, sample_world);
		  Intersection sample_i;
		  bool test = bvh->intersect(sample_ray, &sample_i);
		  if (test) {
			  Spectrum sample_ligt_emited = sample_i.bsdf->get_emission();
			  double costerm = cos_theta(sample_local);
			  Spectrum fterm = isect.bsdf->f(sample_local, w_out);
			  L_out += sample_ligt_emited * fterm*costerm / pdf;
		  }
		  i += 1;
	  }
	  return L_out / ((double)num_samples);
  }

  Spectrum PathTracer::estimate_direct_lighting_importance(const Ray& r, const Intersection& isect) {
	  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in a hemisphere. 

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
	  Matrix3x3 o2w;
	  make_coord_space(o2w, isect.n);
	  Matrix3x3 w2o = o2w.T();

	  // w_out points towards the source of the ray (e.g.,
	  // toward the camera if this is a primary ray)
	  const Vector3D& hit_p = r.o + r.d * isect.t;
	  const Vector3D& w_out = w2o * (-r.d);
	  Spectrum L_out;

	  // TODO (Part 3.2): 
	  // Here is where your code for looping over scene lights goes
	  // COMMENT OUT `normal_shading` IN `est_radiance_global_illumination` BEFORE YOU BEGIN
	  for (SceneLight* l : scene->lights) {
		  if (l->is_delta_light()) {
			  //only need one sample
			  Vector3D wi_world = Vector3D();
			  float dis_to_l = INF_D;
			  float pdf = 1.0 / (2.0*PI);
			  Spectrum incomingR = l->sample_L(hit_p, &wi_world, &dis_to_l, &pdf);
			  Vector3D wi_local = w2o * (wi_world);
			  if (wi_local.z >= 0) {
				  Ray shadow = Ray(hit_p + EPS_D * (wi_world), wi_world);
				  shadow.max_t = dis_to_l;
				  bool test = bvh->intersect(shadow);
				  if (!test) {
					  double costerm = cos_theta(wi_local);
					  Spectrum fterm = isect.bsdf->f(wi_local, w_out);
					  L_out += incomingR * costerm*fterm / (pdf);
				  }
			  }
		  }
		  else {
			  //not a delta so need ns_area_light samples
			  int i = 0;
			  Spectrum lval = Spectrum();
			  while (i < ns_area_light) {
				  Vector3D wi_world = Vector3D();
				  float dis_to_l = INF_D;
				  float pdf = 1.0 / (2.0*PI);
				  Spectrum incomingR = l->sample_L(hit_p, &wi_world, &dis_to_l, &pdf);
				  Vector3D wi_local = w2o * (wi_world);
				  if (wi_local.z >= 0) {
					  Ray shadow = Ray(hit_p + EPS_D * (wi_world), wi_world);
					  shadow.max_t = dis_to_l;
					  bool test = bvh->intersect(shadow);
					  if (!test) {
						  double costerm = cos_theta(wi_local);
						  Spectrum fterm = isect.bsdf->f(wi_local, w_out);
						  lval += incomingR * costerm*fterm / (pdf);
					  }
				  }
				  i += 1;
			  }
			  L_out += lval / ((double)ns_area_light);
		  }
	  }
	  return L_out;
  }

  Spectrum PathTracer::zero_bounce_radiance(const Ray&r, const Intersection& isect) {

	  // TODO (Part 4.2):
	  // Returns the light that results from no bounces of light
	  return isect.bsdf->get_emission();
  }

  Spectrum PathTracer::one_bounce_radiance(const Ray&r, const Intersection& isect) {

	  // TODO (Part 4.2):
	  // Returns either the direct illumination by hemisphere or importance sampling
	  // depending on `direct_hemisphere_sample`
	  // (you implemented these functions in Part 3)
	  if (direct_hemisphere_sample) {
		  return estimate_direct_lighting_hemisphere(r, isect);
	  }
	  else {
		  return estimate_direct_lighting_importance(r, isect);
	  }
  }

  Spectrum PathTracer::at_least_one_bounce_radiance(const Ray&r, const Intersection& isect) {
	  Matrix3x3 o2w;
	  make_coord_space(o2w, isect.n);
	  Matrix3x3 w2o = o2w.T();

	  Vector3D hit_p = r.o + r.d * isect.t;
	  Vector3D w_out = w2o * (-r.d);
	  //check for indirect only 
	  //Spectrum L_out = Spectrum();
	  //if (r.depth != max_ray_depth) {

	  Spectrum L_out;
	  if (!isect.bsdf->is_delta()){
		  L_out += one_bounce_radiance(r, isect);
	  }
	  //}
	  // TODO (Part 4.2): 
	  // Here is where your code for sampling the BSDF,
	  // performing Russian roulette step, and returning a recursively 
	  // traced ray (when applicable) goes

	  // take a sample of the thing
	  //if (max_ray_depth == 0) {
	  //	  return L_out;
	  //}
	  Vector3D w_in = Vector3D();
	  float pdf = 1 / (2 * PI);
	  Spectrum sample = Spectrum();
	  do {
		  sample = isect.bsdf->sample_f(w_out, &w_in, &pdf);
	  } while (pdf == 0);
	  // set russian rulet number to .6
	  double rrn = .6;
	  //terminate if conrrn is false
	  bool conrrn = coin_flip(rrn);
	  Vector3D world = o2w * w_in;
	  Ray bounce = Ray(hit_p + EPS_D * world, world);
	  bounce.depth = r.depth - 1;
	  if (max_ray_depth > 1 && r.depth == max_ray_depth) {
		  Intersection newInt;
		  if (bvh->intersect(bounce, &newInt)) {
			  Spectrum recurse = at_least_one_bounce_radiance(bounce, newInt);
			  if (isect.bsdf->is_delta())
				  recurse += zero_bounce_radiance(bounce,newInt);
			  double costerm = abs_cos_theta(w_in);
			  return L_out + (recurse * costerm * sample / (pdf));
		  }
	  }
	  if (conrrn && (max_ray_depth > 1 || bounce.depth >= 1)) {
		  Intersection newInt;
		  if (bvh->intersect(bounce, &newInt)) {
			  Spectrum recurse = at_least_one_bounce_radiance(bounce, newInt);
			  if (isect.bsdf->is_delta())
				  recurse += zero_bounce_radiance(bounce, newInt);
			  double costerm = abs_cos_theta(w_in);
			  return L_out + (recurse * costerm * sample / (pdf*rrn));
		  }
	  }
	  return L_out;
  }

  Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
	  Intersection isect;
	  Spectrum L_out;

	  // You will extend this in assignment 3-2. 
	  // If no intersection occurs, we simply return black.
	  // This changes if you implement hemispherical lighting for extra credit.

	  if (!bvh->intersect(r, &isect))
		  return L_out;

	  // This line returns a color depending only on the normal vector 
	  // to the surface at the intersection point.
	  // REMOVE IT when you are ready to begin Part 3.

	  //return normal_shading(isect.n);
	  //return estimate_direct_lighting_hemisphere(r, isect);
	  //return estimate_direct_lighting_importance(r,isect);
	  // TODO (Part 3): Return the direct illumination.

	  // TODO (Part 4): Accumulate the "direct" and "indirect" 
	  //Ray outR = Ray(r.o, -r.d);
	  //outR.depth = r.depth;
	  //indirect only change this back 
	  L_out += zero_bounce_radiance(r, isect);
	  if (max_ray_depth >= 1) {
		  L_out += at_least_one_bounce_radiance(r, isect);
	  }
	  // parts of global illumination into L_out rather than just direct
	  return L_out;
  }

  Spectrum PathTracer::raytrace_pixel(size_t x, size_t y, bool useThinLens) {

	  // TODO (Part 1.1):
	  // Make a loop that generates num_samples camera rays and traces them 
	  // through the scene. Return the average Spectrum. 
	  // You should call est_radiance_global_illumination in this function.

	  // TODO (Part 5):
	  // Modify your implementation to include adaptive sampling.
	  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

	  int num_samples = ns_aa;            // total samples to evaluate
	  Vector2D origin = Vector2D(x, y);    // bottom left corner of the pixel
	  Spectrum average = Spectrum();
	  int n = 0;
	  if (num_samples < 1) {
		  return average;
	  }
	  if (num_samples == 1) {
		  double xnorm = (origin.x + .5) / ((double)sampleBuffer.w);
		  double ynorm = (origin.y + .5) / ((double)sampleBuffer.h);
		  Ray r = camera->generate_ray(xnorm, ynorm);
		  r.depth = max_ray_depth;
		  average = est_radiance_global_illumination(r);
	  }
	  else {
		  float s1 = 0.0;
		  float s2 = 0.0;
		  int mult = 1;
		  while (n < num_samples) {
			  Vector2D st = gridSampler->get_sample();
			  double xnorm = (origin.x + st.x) / ((double)sampleBuffer.w);
			  double ynorm = (origin.y + st.y) / ((double)sampleBuffer.h);
			  Ray r = camera->generate_ray(xnorm, ynorm);
			  r.depth = max_ray_depth;
			  Spectrum rspec = est_radiance_global_illumination(r);
			  average += rspec;
			  s1 += rspec.illum();
			  s2 += (rspec.illum()*rspec.illum());
			  n += 1;
			  if (n == mult * samplesPerBatch) {
				  mult += 1;
				  float mean = s1 / ((float)n);
				  float sd_sqrd = (s2 - (s1*s1 / (float)n)) / ((float)(n - 1.0));
				  float convergVar = 1.96*sqrt(sd_sqrd / (float)n);
				  if (convergVar <= maxTolerance * mean) {
					  break;
				  }
			  }
		  }
	  }
	  //edit to be the total number of samples used.
	  sampleCountBuffer[x + y * frameBuffer.w] = n;
	  return average / ((double)n);
  }

  // Diffuse BSDF //

  Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
	  // TODO (Part 3.1): 
  // This function takes in both wo and wi and returns the evaluation of
  // the BSDF for those two directions.
	  return reflectance / PI;
  }

  Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

	  // TODO (Part 3.1): 
	  // This function takes in only wo and provides pointers for wi and pdf,
	  // which should be assigned by this function.
	  // After sampling a value for wi, it returns the evaluation of the BSDF
	  // at (wo, *wi).
	  *wi = sampler.get_sample(pdf);
	  return f(wo, *wi);
  }

  // Camera //
  Ray Camera::generate_ray(double x, double y) const {
	  // TODO (Part 1.2):
  // compute position of the input sensor sample coordinate on the
  // canonical sensor plane one unit away from the pinhole.
  // Note: hFov and vFov are in degrees.
  // 
	  Vector3D botomLeft = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5), -1);
	  Vector3D topRight = Vector3D(tan(radians(hFov)*.5), tan(radians(vFov)*.5), -1);
	  Vector3D d = Vector3D((1 - x)*botomLeft.x + x * topRight.x, (1 - y)*botomLeft.y + y * topRight.y, -1);
	  d = c2w * d;
	  //d = d - pos;
	  d.normalize();
	  Ray ret_ray = Ray(pos, d);
	  ret_ray.min_t = nClip;
	  ret_ray.max_t = fClip;
	  return ret_ray;
  }
}

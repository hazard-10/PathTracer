
#include "shape.h"
#include "../geometry/util.h" 

namespace Shapes {

Vec2 Sphere::uv(Vec3 dir) {
	float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
	if (u < 0.0f) u += 1.0f;
	float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;
	return Vec2{u, v};
}

BBox Sphere::bbox() const {
	BBox box;
	box.enclose(Vec3(-radius));
	box.enclose(Vec3(radius));
	return box;
}

PT::Trace Sphere::hit(Ray ray) const {
	//A3T2 - sphere hit

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    PT::Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
	ret.uv = Vec2{}; 	   // what was the uv coordinates at the intersection? (you may find Sphere::uv to be useful)
    
	// perform ray-sphere intersection
	Vec3 o = ray.point;
	Vec3 d = ray.dir;
	float d_squared = d.norm_squared();
	float o_squared = o.norm_squared();
	float r = radius;

	// calculate discriminant
	float discriminant = 4 * (dot(o, d) * dot(o, d)) - 4 * d_squared * (o_squared - r * r);
	if(discriminant < 0) return ret;
	// check if outside of bounds
	float discriminant_sqrt = std::sqrt(discriminant);

	float t1 = (-2 * dot(o, d) + discriminant_sqrt) / (2 * d_squared);
	float t2 = (-2 * dot(o, d) - discriminant_sqrt) / (2 * d_squared);

	// check if within ray dist bounds
	auto within_bounds = [&](float t) {
		return t >= ray.dist_bounds[0] && t <= ray.dist_bounds[1];
	};

	float t_ret_distance = 0;
	if(within_bounds(t1) && within_bounds(t2)) {
		t_ret_distance = std::min(t1, t2);
	} else if(within_bounds(t1)) {
		t_ret_distance = t1;
	} else if(within_bounds(t2)) {
		t_ret_distance = t2;
	} else {
		return ret;
	}
	

	// return ret
	ret.hit = true;
	ret.distance = t_ret_distance;
	ret.position = ray.point + t_ret_distance * ray.dir; 
	ret.normal = ret.position.unit();
	ret.uv = uv(ret.normal);

	Ray ray_copy = ray;
	ray.dist_bounds.y = t_ret_distance;

	return ret;
}

Vec3 Sphere::sample(RNG &rng, Vec3 from) const {
	die("Sampling sphere area lights is not implemented yet.");
}

float Sphere::pdf(Ray ray, Mat4 pdf_T, Mat4 pdf_iT) const {
	die("Sampling sphere area lights is not implemented yet.");
}

Indexed_Mesh Sphere::to_mesh() const {
	return Util::sphere_mesh(radius, 2);
}

} // namespace Shapes

bool operator!=(const Shapes::Sphere& a, const Shapes::Sphere& b) {
	return a.radius != b.radius;
}

bool operator!=(const Shape& a, const Shape& b) {
	if (a.shape.index() != b.shape.index()) return false;
	return std::visit(
		[&](const auto& shape) {
			return shape != std::get<std::decay_t<decltype(shape)>>(b.shape);
		},
		a.shape);
}

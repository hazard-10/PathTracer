
#include "../test.h"

#include "samplers.h"
#include "tri_mesh.h"
#include "../util/helper.h"

namespace PT {

BBox Triangle::bbox() const {
	//A3T2 / A3T3

	// TODO (PathTracer): Task 2 or 3
    // Compute the bounding box of the triangle.

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::hit.

    BBox box;

	float min_x = std::min(vertex_list[v0].position.x, std::min(vertex_list[v1].position.x, vertex_list[v2].position.x));
	float min_y = std::min(vertex_list[v0].position.y, std::min(vertex_list[v1].position.y, vertex_list[v2].position.y));
	float min_z = std::min(vertex_list[v0].position.z, std::min(vertex_list[v1].position.z, vertex_list[v2].position.z));
	float max_x = std::max(vertex_list[v0].position.x, std::max(vertex_list[v1].position.x, vertex_list[v2].position.x));
	float max_y = std::max(vertex_list[v0].position.y, std::max(vertex_list[v1].position.y, vertex_list[v2].position.y));
	float max_z = std::max(vertex_list[v0].position.z, std::max(vertex_list[v1].position.z, vertex_list[v2].position.z));
	if(min_x == max_x) {
		min_x -= 0.0001f;
		max_x += 0.0001f;
	}
	if(min_y == max_y) {
		min_y -= 0.0001f;
		max_y += 0.0001f;
	}
	if(min_z == max_z) {
		min_z -= 0.0001f;
		max_z += 0.0001f;
	}

    return box;
}

Trace Triangle::hit(const Ray& ray) const {
	//A3T2
	
	// Each vertex contains a postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;

	Vec3 s = ray.point - v_0.position;
	Vec3 e1 = v_1.position - v_0.position;
	Vec3 e2 = v_2.position - v_0.position;
	Vec3 dir = ray.dir;

    // TODO (PathTracer): Task 2
    // Intersect the ray with the triangle defined by the three vertices.

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
                           // (this should be interpolated between the three vertex normals)
	ret.uv = Vec2{};	   // What was the uv associated with the point of intersection?
						   // (this should be interpolated between the three vertex uvs)

	// perform ray-triangle intersection test
	float denom_e1_d_d2 = dot(cross(e1, dir), e2);
	if (denom_e1_d_d2 == 0) { // ray is parallel to triangle
		return ret;
	}
	Vec3 cramer_rule = Vec3(-1*dot(cross(s, e2), dir),
							dot(cross(e1, dir), s),
							-dot(cross(s, e2), e1));
	Vec3 u_v_t = cramer_rule / denom_e1_d_d2; 
	float u = u_v_t.x;
	float v = u_v_t.y;
	float t = u_v_t.z;
	if (u < 0 || v < 0 || u + v > 1 || t < 0) { // intersection is outside triangle
		return ret;
	}
	// check the distance of the intersection
	Vec3 dist = ray.dir * t;
	float distance = dist.norm();
	if (distance > ray.dist_bounds.y || distance < ray.dist_bounds.x) {
		return ret;
	}
	// hit confirmed
	ret.hit = true;
	ret.distance = distance;
	ret.position = ray.point + dist;
	ret.normal = (1 - u - v) * v_0.normal + u * v_1.normal + v * v_2.normal;
	ret.uv = (1 - u - v) * v_0.uv + u * v_1.uv + v * v_2.uv;

    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, uint32_t v0, uint32_t v1, uint32_t v2)
	: v0(v0), v1(v1), v2(v2), vertex_list(verts) {
}

Vec3 Triangle::sample(RNG &rng, Vec3 from) const {
	Tri_Mesh_Vert v_0 = vertex_list[v0];
	Tri_Mesh_Vert v_1 = vertex_list[v1];
	Tri_Mesh_Vert v_2 = vertex_list[v2];
	Samplers::Triangle sampler(v_0.position, v_1.position, v_2.position);
	Vec3 pos = sampler.sample(rng);
	return (pos - from).unit();
}

float Triangle::pdf(Ray wray, const Mat4& T, const Mat4& iT) const {

	Ray tray = wray;
	tray.transform(iT);

	Trace trace = hit(tray);
	if (trace.hit) {
		trace.transform(T, iT.T());
		Vec3 v_0 = T * vertex_list[v0].position;
		Vec3 v_1 = T * vertex_list[v1].position;
		Vec3 v_2 = T * vertex_list[v2].position;
		Samplers::Triangle sampler(v_0, v_1, v_2);
		float a = sampler.pdf(trace.position);
		float g = (trace.position - wray.point).norm_squared() / std::abs(dot(trace.normal, wray.dir));
		return a * g;
	}
	return 0.0f;
}

bool Triangle::operator==(const Triangle& rhs) const {
	if (Test::differs(vertex_list[v0].position, rhs.vertex_list[rhs.v0].position) ||
	    Test::differs(vertex_list[v0].normal, rhs.vertex_list[rhs.v0].normal) ||
	    Test::differs(vertex_list[v0].uv, rhs.vertex_list[rhs.v0].uv) ||
	    Test::differs(vertex_list[v1].position, rhs.vertex_list[rhs.v1].position) ||
	    Test::differs(vertex_list[v1].normal, rhs.vertex_list[rhs.v1].normal) ||
	    Test::differs(vertex_list[v1].uv, rhs.vertex_list[rhs.v1].uv) ||
	    Test::differs(vertex_list[v2].position, rhs.vertex_list[rhs.v2].position) ||
	    Test::differs(vertex_list[v2].normal, rhs.vertex_list[rhs.v2].normal) ||
	    Test::differs(vertex_list[v2].uv, rhs.vertex_list[rhs.v2].uv)) {
		return false;
	}
	return true;
}

Tri_Mesh::Tri_Mesh(const Indexed_Mesh& mesh, bool use_bvh_) : use_bvh(use_bvh_) {
	for (const auto& v : mesh.vertices()) {
		verts.push_back({v.pos, v.norm, v.uv});
	}

	const auto& idxs = mesh.indices();

	std::vector<Triangle> tris;
	for (size_t i = 0; i < idxs.size(); i += 3) {
		tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
	}

	if (use_bvh) {
		triangle_bvh.build(std::move(tris), 4);
	} else {
		triangle_list = List<Triangle>(std::move(tris));
	}
}

Tri_Mesh Tri_Mesh::copy() const {
	Tri_Mesh ret;
	ret.verts = verts;
	ret.triangle_bvh = triangle_bvh.copy();
	ret.triangle_list = triangle_list.copy();
	ret.use_bvh = use_bvh;
	return ret;
}

BBox Tri_Mesh::bbox() const {
	if (use_bvh) return triangle_bvh.bbox();
	return triangle_list.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
	if (use_bvh) return triangle_bvh.hit(ray);
	return triangle_list.hit(ray);
}

size_t Tri_Mesh::n_triangles() const {
	return use_bvh ? triangle_bvh.n_primitives() : triangle_list.n_primitives();
}

uint32_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                             const Mat4& trans) const {
	if (use_bvh) return triangle_bvh.visualize(lines, active, level, trans);
	return 0u;
}

Vec3 Tri_Mesh::sample(RNG &rng, Vec3 from) const {
	if (use_bvh) {
		return triangle_bvh.sample(rng, from);
	}
	return triangle_list.sample(rng, from);
}

float Tri_Mesh::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (use_bvh) {
		return triangle_bvh.pdf(ray, T, iT);
	}
	return triangle_list.pdf(ray, T, iT);
}

} // namespace PT

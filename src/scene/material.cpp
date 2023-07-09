
#include "material.h"
#include "../util/rand.h"
#include <iostream>

namespace Materials {

Vec3 reflect(Vec3 dir) {
	//A3T5 Materials - reflect helper

    // Return direction to incoming light that would be
	// reflected out in direction dir from surface
	// with normal (0,1,0)
	Vec3 ret = -dir;
	ret.y = -ret.y;
	return ret;

    // return Vec3{};
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {
    Vec3 normal = Vec3{0,1,0};
    float cosThetaI = dot(out_dir, normal);
    bool entering = cosThetaI > 0;
    float etaI = entering ? 1.0f : index_of_refraction;
    float etaT = entering ? index_of_refraction : 1.0f;

    // Compute sin^2(theta_t) using Snell's law
    float sin2ThetaI = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
    float sin2ThetaT = etaI * etaI * sin2ThetaI / (etaT * etaT);

    if (sin2ThetaT >= 1) {
        was_internal = true; 
        return Vec3{};
    }

    float cosThetaT = std::sqrt(1 - sin2ThetaT);

    // Vec3 refractedDir;
    // if (entering) { 
    //     refractedDir = etaI / etaT * (-out_dir + normal * cosThetaI) - normal * cosThetaT;
    // } else {
    //     // normal = -normal;
    //     refractedDir = etaI / etaT * (out_dir - normal * cosThetaI) + normal * cosThetaT;
    // }

	Vec3 in_dir(-out_dir.x, 0.0f, -out_dir.z);
    in_dir.normalize();
    in_dir *= std::sqrt(sin2ThetaT);
    in_dir.y = cosThetaT;
    if(0 < out_dir.y) in_dir.y = -in_dir.y;

    return in_dir;
}



float schlick(Vec3 in_dir, float index_of_refraction) {
	//A3T5 Materials - Schlick's approximation helper

	// Implement Schlick's approximation of the Fresnel reflection factor.

	// The surface normal is (0,1,0)

	float cosTheta = abs(dot(in_dir, Vec3{0,1,0}));
	float r0 = (1 - index_of_refraction) / (1 + index_of_refraction);
	r0 = r0 * r0;
	return float(double(r0) + double(1.0f - r0) * (pow(1 - cosTheta, 5)));

	// return 0.0f;
}

Spectrum Lambertian::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	//A3T4: Materials - Lambertian BSDF evaluation

    // Compute the ratio of reflected/incoming radiance when light from in_dir
    // is reflected through out_dir: albedo / PI_F * cos(theta).

	// float cosTheta = std::max(0.0f, dot(in.unit(), Vec3{0,1,0}));
	float cosTheta = std::max(0.0f, in.y);
	//  float cosTheta = std::max(0.0f, dot(in, out));
	Spectrum alebedoIncomToOutgo = albedo.lock()->evaluate(uv);
	return alebedoIncomToOutgo / PI_F * cosTheta;
}

Scatter Lambertian::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T4: Materials - Lambertian BSDF scattering
	//Select a scattered light direction at random from the Lambertian BSDF

	[[maybe_unused]] Samplers::Hemisphere::Cosine sampler; //this will be useful

	Scatter ret;
	//TODO: sample the direction the light was scatter from from a cosine-weighted hemisphere distribution:
	ret.direction = sampler.sample(rng);

	//TODO: compute the attenuation of the light using Lambertian::evaluate():
	ret.attenuation = Lambertian::evaluate(out, ret.direction, uv);

	//indicate that this is not a specular reflection:
	ret.specular = false;
	return ret;
}

float Lambertian::pdf(Vec3 out, Vec3 in) const {
	//A3T4: Materials - Lambertian BSDF probability density function
    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
	[[maybe_unused]] Samplers::Hemisphere::Cosine sampler; //this might be handy!

    return sampler.pdf(in);
}

Spectrum Lambertian::emission(Vec2 uv) const {
	return {};
}

std::weak_ptr<Texture> Lambertian::display() const {
	return albedo;
}

void Lambertian::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(albedo);
}

Spectrum Mirror::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Mirror::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5: mirror

	// Use reflect to compute the new direction
	// Don't forget that this is a discrete material!
	// Similar to albedo, reflectance represents the ratio of incoming light to reflected light

    Scatter ret;
    ret.direction = reflect(out);
    ret.attenuation = reflectance.lock()->evaluate(uv);
	ret.specular = true;
    return ret;
}

float Mirror::pdf(Vec3 out, Vec3 in) const {
	return 0.0f;
}

Spectrum Mirror::emission(Vec2 uv) const {
	return {};
}

std::weak_ptr<Texture> Mirror::display() const {
	return reflectance;
}

void Mirror::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(reflectance);
}

Spectrum Refract::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Refract::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5 - refract

	// Use refract to determine the new direction - what happens in the total internal reflection case?
    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
	// Don't forget that this is a discrete material!
	// For attenuation, be sure to take a look at the Specular Transimission section of the PBRT textbook for a derivation
 
    Scatter ret;
	bool was_internal = false; 
    ret.direction = refract(out, ior, was_internal).unit(); 
	if(was_internal) { 
		ret.direction = reflect(out).unit();
		ret.attenuation = Spectrum{1.0f};
	}else{
		float cosAngleOutDir = dot(out, Vec3{0,1,0});
		bool entering = cosAngleOutDir > 0;
		float ior_ratio = entering ? 1.0f / ior : ior; 
		float sqr_ior_ratio = ior_ratio * ior_ratio;
		Spectrum local_transmittance = transmittance.lock()->evaluate(uv) * sqr_ior_ratio; 
		ret.attenuation = local_transmittance;
	}
	
	// ret.direction = reflect(out);
	// ret.attenuation = transmittance.lock()->evaluate(uv);
	ret.specular = true;
    return ret;
}

float Refract::pdf(Vec3 out, Vec3 in) const {
	return 0.0f;
}

Spectrum Refract::emission(Vec2 uv) const {
	return {};
}

bool Refract::is_emissive() const {
	return false;
}

bool Refract::is_specular() const {
	return true;
}

bool Refract::is_sided() const {
	return true;
}

std::weak_ptr<Texture> Refract::display() const {
	return transmittance;
}

void Refract::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(transmittance);
}

Spectrum Glass::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Glass::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5 - glass

    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?
    // When debugging Glass, it may be useful to compare to a pure-refraction BSDF
	// For attenuation, be sure to take a look at the Specular Transimission section of the PBRT textbook for a derivation

	// bool was_internal = false;
	// Vec3 refract_indir = refract(out, ior, was_internal);
	// float fresnel = 0.0f;
	// if(was_internal) {
	// 	fresnel = 1.0f;
	// } else {
	// 	// fresnel = schlick(out, ior);
	// 	fresnel = 0.0f;
	// }

   	Scatter ret;
	bool was_internal = false; 
    ret.direction = refract(out, ior, was_internal).unit(); 
	float fresnel = schlick(out, ior);
	// std::cout << "fresnel: " << fresnel << std::endl;
	if(was_internal || rng.coin_flip(fresnel)) { 
		ret.direction = reflect(out).unit();
		ret.attenuation = reflectance.lock()->evaluate(uv);
	}else{
		float cosAngleOutDir = dot(out, Vec3{0,1,0});
		bool entering = cosAngleOutDir > 0;
		float ior_ratio = entering ? 1.0f / ior : ior; 
		float sqr_ior_ratio = ior_ratio * ior_ratio;
		Spectrum local_transmittance = transmittance.lock()->evaluate(uv) * sqr_ior_ratio; 
		ret.attenuation = local_transmittance;
	}
	
	// ret.direction = reflect(out);
	// ret.attenuation = transmittance.lock()->evaluate(uv);
	ret.specular = true;
    return ret;
}

float Glass::pdf(Vec3 out, Vec3 in) const {
	return 0.0f;
}

Spectrum Glass::emission(Vec2 uv) const {
	return {};
}

bool Glass::is_emissive() const {
	return false;
}

bool Glass::is_specular() const {
	return true;
}

bool Glass::is_sided() const {
	return true;
}

std::weak_ptr<Texture> Glass::display() const {
	return transmittance;
}

void Glass::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(reflectance);
	f(transmittance);
}

Spectrum Emissive::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Emissive::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	Scatter ret;
	ret.specular = true;
	ret.direction = {};
	ret.attenuation = {};
	return ret;
}

float Emissive::pdf(Vec3 out, Vec3 in) const {
	return 0.0f;
}

Spectrum Emissive::emission(Vec2 uv) const {
	return emissive.lock()->evaluate(uv);
}

bool Emissive::is_emissive() const {
	return true;
}

bool Emissive::is_specular() const {
	return true;
}

bool Emissive::is_sided() const {
	return false;
}

std::weak_ptr<Texture> Emissive::display() const {
	return emissive;
}

void Emissive::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(emissive);
}

} // namespace Materials

bool operator!=(const Materials::Lambertian& a, const Materials::Lambertian& b) {
	return a.albedo.lock() != b.albedo.lock();
}

bool operator!=(const Materials::Mirror& a, const Materials::Mirror& b) {
	return a.reflectance.lock() != b.reflectance.lock();
}

bool operator!=(const Materials::Refract& a, const Materials::Refract& b) {
	return a.transmittance.lock() != b.transmittance.lock() || a.ior != b.ior;
}

bool operator!=(const Materials::Glass& a, const Materials::Glass& b) {
	return a.reflectance.lock() != b.reflectance.lock() ||
	       a.transmittance.lock() != b.transmittance.lock() || a.ior != b.ior;
}

bool operator!=(const Materials::Emissive& a, const Materials::Emissive& b) {
	return a.emissive.lock() != b.emissive.lock();
}

bool operator!=(const Material& a, const Material& b) {
	if (a.material.index() != b.material.index()) return false;
	return std::visit(
		[&](const auto& material) {
			return material != std::get<std::decay_t<decltype(material)>>(b.material);
		},
		a.material);
}

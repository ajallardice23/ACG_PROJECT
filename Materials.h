#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here

		//clamp costheta
		cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));

		//check if in or out material
		bool in = cosTheta > 0.0f;

		float n1;
		float n2;
		//ior
		if (in == true) {
			n1 = iorExt;
			n2 = iorInt;
		}

		else {
			n1 = iorInt;
			n2 = iorExt;
		}

		//positive
		cosTheta = std::abs(cosTheta);

		//sn
		//sin theta incident / transmit
		float sinThetaInc = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
		float sinThetaTrans = (n1 / n2) * sinThetaInc;
		

		if (sinThetaTrans >= 1.0f) {
			return 1.0f;
		}

		float cosThetaTrans = std::sqrt(std::max(0.0f, 1.0f - sinThetaTrans * sinThetaTrans));


		//Fresnel
		float Perp = ((n1 * cosTheta) - (n2 * cosThetaTrans)) / ((n1 * cosTheta) + (n2 * cosThetaTrans));
		float Parallel = ((n2 * cosTheta) - (n1 * cosThetaTrans)) / ((n2 * cosTheta) + (n1 * cosThetaTrans));

		//Sqr
		return (Perp * Perp + Parallel * Parallel) / 2.0f;
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here
		//nts - slide 40 reprot

		//clamp 0 1
		cosTheta = std::max(0.0f, std::min(1.0f, cosTheta));

		//costheta sq
		float cosThetaSquared = cosTheta * cosTheta;

		//colours rgb
		Colour ctsColour(cosThetaSquared, cosThetaSquared, cosThetaSquared);
		Colour colourO(1.0f, 1.0f, 1.0f);

		Colour nkSquared = (ior * ior) + (k * k);

		//2costheta
		Colour nCos = ior * (2.0f * cosTheta);

		//top para equation
		Colour parallelTop = (nkSquared - nCos + ctsColour);
		//bottom para equation
		Colour parallelBottom = (nkSquared + nCos + ctsColour);
		//result
		Colour parallelProduct = (parallelTop / parallelBottom);

		//top perp equation
		Colour perpTop = (nkSquared - nCos + ctsColour);
		//bottom perp equation
		Colour perpBottom = (nkSquared - nCos + ctsColour);
		//result
		Colour perpProduct = (perpTop / perpBottom);

		return Colour(parallelProduct + perpProduct / 2);
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		//nts - 88 report

		float cosTheta = wi.z;

		//if less 0 just return 0
		if (cosTheta <= 0.0f) {
			return 0.0f;
		}


		float cosThetaSquared = cosTheta * cosTheta;
		float alphaSquared = alpha * alpha;
		//sinsq+cossq = 1 (-cos from 1 for sin)
		float sinThetaSquared = std::max(0.0f, 1.0f - cosThetaSquared);
		// tan is sin over cos
		float tanThetaSquared = (sinThetaSquared / cosThetaSquared);


		//result
		return ((std::sqrt(1.0f + alphaSquared * tanThetaSquared) - 1.0f) / 2);
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		//nts - 83 report
		float lambdai = lambdaGGX(wi, alpha);
		float lambdao = lambdaGGX(wo, alpha);

		return (1.0f / (1 + (lambdai + lambdao)));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		//nts - 80report

		float cosTheta = h.z;

		if (cosTheta <= 0) {
			return 0;
		}
		float alphaSquared = alpha * alpha;
		float cosThetaSquared = cosTheta * cosTheta;

		const float pi = 3.142f;
		//bottom equation ()
		float bottomi = (cosThetaSquared * (alphaSquared - 1.0f) + 1.0f);
		//result
		return (alphaSquared / (pi * bottomi * bottomi));
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		/*Vec3 wi = Vec3(0, 1, 0);
		pdf = 1.0f;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/

		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wiLocal.z / M_PI;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		//return 1.0f;
		//local
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		//if less 0 0
		if (wiLocal.z <= 0.0f) {
			return 0.0f;
		}


		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/

		//locals
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);

		pdf = std::abs(wiLocal.z);

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
		return shadingData.frame.toWorld(wiLocal);

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
		//note slide 94

		//local
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		//if less 0 return 0
		if (woLocal.z <= 0.0f) {
			pdf = 0.0f;
			return Vec3(0, 0, 0);
		}

		//2 rand numbers
		float rand = sampler->next();
		float rand2 = sampler->next();

		float alphaSquared = (alpha * alpha);
		//thetaM equation
		float thetaM = (std::sqrt(1.0f - rand) / (rand * (alphaSquared - 1.0f) + 1.0f));
		//phiM 2 pi rand2
		float phiM = (2.0f * 3.142 * rand2);

		//convert to vec

		Vec3 product(
			std::sin(thetaM) * std::cos(phiM),
			std::sin(thetaM) * std::sin(phiM),
			std::cos(thetaM)
		);

		
		float woDotH = woLocal.dot(product);
		//less 0 return 0
		if (woDotH <= 0.0f) {
			pdf = 0.0f;
			return Vec3(0, 0, 0);
		}

		Vec3 wiLocal = (product * (2.0f * woDotH)) - woLocal;

		float D = ShadingHelper::Dggx(product, alpha);
		float pdf_h = D * product.z;
		pdf = pdf_h / (4.0f * woDotH);

		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal);

		//evaluate colour
		reflectedColour = evaluate(shadingData, wiWorld);

		return wiWorld;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{

		//nts - slide 100 report
		// Replace this with Conductor evaluation code
		//locals for cook-torance brdf
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		//if under 0 return 0
		if (woLocal.z <= 0.0f or wiLocal.z <= 0.0f) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Vec3 halfVec = woLocal + wiLocal;
		halfVec.normalize();
		float cosThetaI = wiLocal.z;
		float cosThetaO = woLocal.z;

		float cosThetaD = std::max(0.0f, wiLocal.dot(halfVec));

		//DGGX, GGGX, FRES
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		float D = ShadingHelper::Dggx(halfVec, alpha);
		Colour F = ShadingHelper::fresnelConductor(cosThetaD, eta, k);

		//cooktorance
		Colour ct = (F * G * D) / (4.0f * cosThetaO * cosThetaI);

		Colour albedoColour = albedo->sample(shadingData.tu, shadingData.tv);
		return ct * albedoColour;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		//nts 92
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		//if under 0 return 0
		if (woLocal.z <= 0.0f or wiLocal.z <= 0.0f) {
			return 0.0f;
		}


		Vec3 halfVec = woLocal + wiLocal;
		halfVec.normalize();
		
		float woDHalf = woLocal.dot(halfVec);
		if (woDHalf <= 0) {
			return 0.0f;
		}

		float D = ShadingHelper::Dggx(halfVec, alpha);
		float pdfHalf = D * halfVec.z;

		return (pdfHalf / (4 * woDHalf));
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		/* Replace this with Glass sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/

		//local
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		//in/out

		bool in = woLocal.z > 0.0f;
		float n1;
		float n2;
		//ior
		if (in == true) {
			n1 = extIOR;
			n2 = intIOR;
		}

		else {
			n1 = intIOR;
			n2 = extIOR;
		}

		Vec3 nLocal;
		//if ray in
		if (in == true) {
			nLocal = Vec3(0.0f, 0.0f, 1.0f);
		}
		//ray out
		else {
			nLocal = Vec3(0.0f, 0.0f, -1.0f);
		}

		float cosTheta = std::abs(woLocal.z);

		float Fres = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		//sn
		float sinThetaInc = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
		float sinThetaTrans = (n1 / n2) * sinThetaInc;
		bool isR;
		if (sinThetaTrans >= 1.0f) {
			isR = true;
		}
		else {
			isR = false;
		}
		Vec3 wiLocal;


		if (isR == true or sampler->next() < Fres) {
			//reflect
			wiLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		}
		else {
			//refract
			//cos angle py
			float cosThetaTrans = std::sqrt(std::max(0.0f, 1.0f - sinThetaTrans * sinThetaTrans));
			//sn
			float eta = n1 / n2;
			wiLocal = (woLocal * -eta) + (nLocal * (eta * cosTheta - cosThetaTrans));
		}

		pdf = std::abs(wiLocal.z);
		//pdf = 1.0f;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
		return shadingData.frame.toWorld(wiLocal);

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		return Colour(0.0f, 0.0f, 0.0f);

		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/

		//local
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		//in/out
		bool in = shadingData.wo.dot(shadingData.gNormal) > 0.0f;

		float n1;
		float n2;

		//ior
		if (in == true) {
			n1 = extIOR;
			n2 = intIOR;
		}
		else {
			n1 = intIOR;
			n2 = extIOR;
		}

		Vec3 nLocal;
		//if ray in
		if (in == true) {
			nLocal = Vec3(0.0f, 0.0f, 1.0f);
		}
		//ray out
		else {
			nLocal = Vec3(0.0f, 0.0f, -1.0f);
		}


		float cosTheta = std::abs(woLocal.z);
		float Fres = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);

		//sn
		float eta = n1 / n2;
		float sinThetaInc = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
		float sinThetaTrans = eta * sinThetaInc;

		bool isR;
		if (sinThetaTrans >= 1.0f) {
			isR = true;
		}
		else {
			isR = false;
		}

		Vec3 wiLocal;

		//russian roulette
		if (isR == true || sampler->next() < Fres) {
			//reflect
			wiLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		}
		else {
			//refract
			float cosThetaTrans = std::sqrt(std::max(0.0f, 1.0f - sinThetaTrans * sinThetaTrans));
			wiLocal = (woLocal * -eta) + (nLocal * (eta * cosTheta - cosThetaTrans));
		}

		pdf = std::abs(wiLocal.z);
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);
		return (0.0f);
	}
	bool isPureSpecular()
	{
		return true;
		//return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};
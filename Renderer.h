#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

//octree for irradiance cache
#include "Octree.h"
//mutex stops crashing with multithreading (took a while to figure out)
#include <mutex>
//MT
#include <atomic>
#include <vector>



class RayTracer {
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	//octree
	Octree* irradianceCache;
	std::mutex cacheMutex;

	int tileSize = 32;
	int tile_x = 0, tile_y = 0;


	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas) {
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;

		//irradiance cache / octree- create box
		irradianceCache = new Octree(Vec3(0, 0, 0), 2000.0f);

		GetSystemInfo(&sysInfo);
		samplers = new MTRandom[sysInfo.dwNumberOfProcessors];
		clear();
	}

	void clear() {
		film->clear();
	}


	Colour computeDirect(ShadingData shadingData, Sampler* sampler) {
		if (shadingData.bsdf->isPureSpecular()) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float pmf;
		Light* lit = scene->sampleLight(sampler, pmf);

		//if (!lit->isArea()) { return Colour(0.0f, 0.0f, 0.0f); } //if (!light) return Colour(0.0f, 0.0f, 0.0f);

		float pdf;
		Colour emitted;
		Vec3 pos = lit->sample(shadingData, sampler, emitted, pdf);


		Vec3 wi;
		float pdfAngle;


		if (lit->isArea() == true) {
			wi = pos - shadingData.x;
			float distanceSq = wi.lengthSq();
			float dist = sqrtf(distanceSq);
			wi = wi / dist;


			//shadow
			if (!scene->visible(shadingData.x, pos)) {
				return Colour(0.0f, 0.0f, 0.0f);
			}

			float cosThetaLit = std::max(0.0f, Dot(lit->normal(shadingData, wi), -wi));
			//less 0 ret 0
			if (cosThetaLit <= 0.0f) {
				return Colour(0.0f, 0.0f, 0.0f);
			}


			pdfAngle = (pdf * distanceSq) / cosThetaLit;

		}

		else {
			wi = pos;
			//shoot away
			if (!scene->visible(shadingData.x + wi * EPSILON, shadingData.x + wi * 10000.0f)) {
				return Colour(0.0f, 0.0f, 0.0f);
			}
			pdfAngle = pdf;
		}

		pdfAngle = pdfAngle * pmf;
		if (pdfAngle <= 0.0f) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Colour f = shadingData.bsdf->evaluate(shadingData, wi);
		float cosThetaSur = std::max(0.0f, Dot(shadingData.sNormal, wi));
		float bsdfPdf = shadingData.bsdf->PDF(shadingData, wi);

		float wTop = pdfAngle * pdfAngle;
		float wBottom = wTop + (bsdfPdf * bsdfPdf);
		float weight = wTop / wBottom;

		return (f * emitted * cosThetaSur * weight) / pdfAngle;
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, float previous = 0.0f, bool prevS = true) {
		//stop bounce infin 
		//nts THIS CAUSES ISSUES LOW SOME SCENES
		if (depth > 8) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		IntersectionData intersection = scene->traverse(r);
		if (intersection.t >= FLT_MAX) {
			//hit nothing 
			Colour nt = scene->background->evaluate(r.dir);

			if (depth == 0 or prevS == true) {
				return nt;
			}

			float pmf = 1.0f / scene->lights.size();
			float lightPdf = scene->background->PDF(ShadingData(), r.dir) * pmf;

			float wTop = previous * previous;
			float wBottom = wTop + (lightPdf * lightPdf);
			float weight = wTop / wBottom;

			return nt * weight;
		}

		ShadingData sd = scene->calculateShadingData(intersection, r);
		//light source
		if (sd.bsdf->isLight()) {
			if (depth <= 0) {
				return sd.bsdf->emit(sd, sd.wo);
			}

			float pmf = 1.0f / scene->lights.size();
			float area = scene->triangles[intersection.ID].area;
			float pdfArea = 1.0f / area;
			float distSq = intersection.t * intersection.t;
			float cosThetaLit = std::max(0.0f, Dot(sd.gNormal, sd.wo));

			if (cosThetaLit <= 0.0f) {
				return Colour(0.0f, 0.0f, 0.0f);
			}

			//angle
			float lightPdf = pmf * (pdfArea * distSq) / cosThetaLit;

			float wTop = previous * previous;
			float wBottom = wTop + (lightPdf * lightPdf);
			float weight = wTop / wBottom;

			return sd.bsdf->emit(sd, sd.wo) * weight;
		}

		Colour directLighting = computeDirect(sd, sampler);

		//irradiance cache
		//caching if indirect
		if (depth == 0) {
			//val store near pos
			std::vector<IrradianceCache> values;
			//lock mutex when add pts (stop two threads writing at once)
			cacheMutex.lock();
			irradianceCache->findPt(sd.x, values);
			cacheMutex.unlock();


			//colours close to vals
			Colour totalI(0, 0, 0);
			//weights for avg
			float weight = 0.0f;

			//loop through octree val
			for (int i = 0; i < values.size(); i++) {
				float distance = (sd.x - values[i].pos).length();
				float dotProduct = Dot(sd.sNormal, values[i].normal);
				//if distance between points close
				if (distance < 0.8f && dotProduct > 0.9f) {
					float w = 1.0f / distance;
					totalI = totalI + (values[i].irradiance * w);
					weight += w;
				}
			}

			//if the weight > cache hit return
			if (weight > 0.5f) {
				return directLighting + (totalI / weight);
			}

		}

		Colour indirectLighting;
		//probdensity
		float pdf;


		//incoming light5
		Vec3 wi = sd.bsdf->sample(sd, sampler, indirectLighting, pdf);

		if (pdf <= 0.0f) {
			return directLighting;
		}

		//bounceray
		Ray br(sd.x + (wi * EPSILON), wi);
		float cosTheta = fabsf(Dot(sd.sNormal, wi));


		Colour inLight = pathTrace(br, pathThroughput, depth + 1, sampler);
		//nts DONT TOUCH
		Colour finalInLight = inLight * indirectLighting * (cosTheta / pdf);



		if (depth <= 0) {
			IrradianceCache val;
			val.pos = sd.x;
			val.normal = sd.sNormal;
			val.irradiance = finalInLight;
			cacheMutex.lock();
			irradianceCache->newPt(val);
			cacheMutex.unlock();
		}
		return directLighting + finalInLight;

	}

	Colour direct(Ray& r, Sampler* sampler) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}

	Colour albedo(Ray& r) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}

	Colour viewNormals(Ray& r) {
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX) {
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}


	void renderMT() {
		film->incrementSPP();

		//tile count
		int gridW = (film->width + tileSize - 1) / tileSize;
		int gridH = (film->height + tileSize - 1) / tileSize;
		int totalTiles = gridW * gridH;

	
		std::atomic<int> tileCounter{ 0 };

		//cores
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		int cores = sysInfo.dwNumberOfProcessors;

		//thrjeads
		std::vector<std::thread> threads;
		for (int i = 0; i < cores; ++i) {

			//start thread
			threads.push_back(std::thread([this, i, gridW, totalTiles, &tileCounter]() {

				while (true) {
					int tileID = tileCounter.fetch_add(1);

					//break if no tiles
					if (tileID >= totalTiles) break;

					//calc tile pos
					int gridX = tileID % gridW;
					int gridY = tileID / gridW;
					int xStart = gridX * tileSize;
					int yStart = gridY * tileSize;
					int xEnd = std::min(xStart + tileSize, (int)film->width);
					int yEnd = std::min(yStart + tileSize, (int)film->height);

					
					for (int y = yStart; y < yEnd; ++y) {
						for (int x = xStart; x < xEnd; ++x) {
							float px = x + 0.5f;
							float py = y + 0.5f;
							Ray ray = scene->camera.generateRay(px, py);

							Colour paththroughput(1.0f, 1.0f, 1.0f);

							Colour col = pathTrace(ray, paththroughput, 0, &samplers[i]);

							film->splat(px, py, col);
							unsigned char r, g, b;
							film->tonemap(x, y, r, g, b, 15.0f);

							canvas->draw(x, y, r, g, b);
						}
					}
				}
				}));
		}

		for (auto& t : threads) {
			t.join();
		}
	}


	void render() {
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				//sample point and generate ray
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				//direct
				//Colour col = direct(ray, &samplers[0]);
				//PART PATHTRACE
				Colour paththroughput(1.0f, 1.0f, 1.0f);
				Colour col = pathTrace(ray, paththroughput, 0, &samplers[0]);

				film->splat(px, py, col);
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b, 15.0f);

				//draw final image
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	int getSPP() {
		return film->SPP;
	}

	void saveHDR(std::string filename) {
		film->save(filename);
	}

	void savePNG(std::string filename) {
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};
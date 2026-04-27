#pragma once
//nts 211-220
#include "Core.h"

//From irradiance cache formula

//xi = pos
//n normal
//rad radius

struct IrradianceCache {
    Vec3 pos;
    Vec3 normal;
    Colour irradiance;
};

class Octree {
public:
    //midbox
    Vec3 center;
    //dis from mid
    float halfWidth;
    //true when no children (start end)
    bool end = true;
    //stores
    std::vector<IrradianceCache> caches;
    //split 8 pts - tb/lr/bf
    Octree* points[8];

    Octree(Vec3 c, float hw) {
        center = c;
        halfWidth = hw;
        end = true;
        for (int i = 0; i < 8; i++) points[i] = nullptr;
    }


    void newPt(IrradianceCache point) {
        //once reach end pt
        if (end == true) {
            caches.push_back(point);

            //once reaches 8 (oct) splice
            //nts HALFWIDTH STOPS CRASHES (check if box is still big enough)
            if (caches.size() > 8 && halfWidth > 0.2f) {
                end = false;

                //next step (half again)
                float step = halfWidth * 0.5;

                //box create from steps each dirx                
                points[0] = new Octree(center + Vec3(-step, -step, -step), step);
                points[1] = new Octree(center + Vec3(-step, -step, step), step);
                points[2] = new Octree(center + Vec3(-step, step, -step), step);
                points[3] = new Octree(center + Vec3(-step, step, step), step);
                points[4] = new Octree(center + Vec3(step, -step, -step), step);
                points[5] = new Octree(center + Vec3(step, step, -step), step);
                points[6] = new Octree(center + Vec3(step, step, step), step);
                points[7] = new Octree(center + Vec3(-step, step, step), step);

                //shift pts
                for (int i = 0; i < caches.size(); i++) {
                    newPt(caches[i]);
                }
                caches.clear();
            }

        }

        else {         
            //binary type logic by adding points depending where point. example top right would be +4+2 = 6 which lines with box creation
            int num = 0;

            //if right
            if (point.pos.x > center.x) {
                num += 4;
            }
            //if top 
            if (point.pos.y > center.y) {
                num += 2;
            }
            //if front
            if (point.pos.z > center.z) {
                num += 1;
            }
            points[num]->newPt(point);
        }
    }

    void findPt(Vec3 pointPos, std::vector<IrradianceCache>& found) {
        if (end == true) {
            for (int i = 0; i < caches.size(); i++) {
                found.push_back(caches[i]);
            }
        }
        else {
            int num = 0;
            //if right
            if (pointPos.x > center.x) {
                num += 4;
            }
            //if top 
            if (pointPos.y > center.y) {
                num += 2;
            }
            //if front
            if (pointPos.z > center.z) {
                num += 1;
            }

            points[num]->findPt(pointPos, found);
        }
    }
};
//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    
    // ? use maxDepth or only Russia Roulette?
    if (depth > this->maxDepth)
        return Vector3f(0.0);

    Intersection intersection = Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    //dark background
    Vector3f hitColor(0.0);
    // Vector3f hitColor = this->backgroundColor;
    // float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;

    static uint32_t testCount = 0;

    if (intersection.happened)
    {
        testCount++;

        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal;       // normal
        Object *hitObject = intersection.obj;
        Vector2f st; // st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
        
        Vector3f wo = normalize(-ray.direction);

        // * L_dir
        Vector3f L_dir(0.0);
        Intersection L_dir_inter;
        float pdf_light;
        
        sampleLight(L_dir_inter, pdf_light);

        auto x =            L_dir_inter.coords;
        auto NN =           L_dir_inter.normal;
        auto ws =           normalize(x - hitPoint);     // ? wi for dir-light; should negative or not?
        auto distance_sqr = dotProduct(x - hitPoint, x - hitPoint);
        auto emit =         L_dir_inter.emit;    

        // ! this n*EPSILON is nessesary(avoid bounding itself!!)
        Vector3f hitPoint_surface_bias = hitPoint + N*EPSILON *(dotProduct(N, ws)>=0.0f ? 1:-1);
        // Ray light_ray(hitPoint, ws);
        Ray light_ray(hitPoint_surface_bias, ws);               // slight diff in dir but its ok

        Intersection sourceBound = Scene::intersect(light_ray);
        if (sourceBound.happened)
        // if (1)
        {
            float bias_sqr = dotProduct(sourceBound.coords - x, sourceBound.coords - x);
            if (bias_sqr <= EPSILON)     // should be set to 0.0f, or EPSILON consider about calculation precision
            // if (sourceBound.obj == L_dir_inter.obj)          // can be a better choice if sampleLight sets L_dir_inter.obj
            {
                L_dir += (
                    emit                                    // L(x, ws)
                    * m->eval(ws, wo, N)                    // f(x, ws, wo)
                    * dotProduct(ws, N)                     // cos for x
                    * (dotProduct(-ws, NN) / distance_sqr)     // dw = dA * (cos'/(x-p)^2), ws is neg for light Source!
                    / pdf_light                             // monte carlo sampling for dA->A
                );
            }

            // if (0)
            // {
            //     printf("\n");
            //     printf("hitPoint:\t%f %f %f\n", hitPoint.x, hitPoint.y, hitPoint.z);
            //     printf("x:\t%f %f %f\n", x.x, x.y, x.z);
            //     printf("ws:\t%f %f %f\n", ws.x, ws.y, ws.z);
            //     printf("wo:\t%f %f %f\n", wo.x, wo.y, wo.z);
            //     printf("N:\t%f %f %f\n", N.x, N.y, N.z);
            //     printf("NN:\t%f %f %f\n", NN.x, NN.y, NN.z);
            //     printf("emit:\t%f %f %f\n", emit.x, emit.y, emit.z);
            //     printf("eval:\t%f %f %f\n", m->eval(ws, wo, N).x, m->eval(ws, wo, N).y ,m->eval(ws, wo, N).z);
            //     printf("cos:\t%f\n", dotProduct(ws, N));
            //     printf("dw:\t%f\n", (dotProduct(ws, NN) / distance_sqr));
            //     printf("pdf_light:\t%f\n", pdf_light);
            //     printf("L_dir:\t%f %f %f\n", L_dir.x, L_dir.y, L_dir.z);
            // } 
        }

        // * L_indir
        Vector3f L_indir(0.0);
        float probability = get_random_float();
        if (probability <= this->RussianRoulette)
        {
            auto wi = m->sample(wo, N);
            
            Ray trace_ray(hitPoint, wi);
            Intersection objectBound = Scene::intersect(trace_ray);
            if (objectBound.happened && objectBound.obj->hasEmit() == false)
            {
                L_indir += (
                    castRay(trace_ray, depth+1)             // shade(q, wi) --> L(x, ws)
                    * m->eval(wi, wo, N)                    // f(y, wi. wo) , y is objectBound point coords(q)
                    * dotProduct(wi, N)                     // cos for y
                    / m->pdf(wi, wo, N)                     // dw->w
                    / this->RussianRoulette                 // RussianRoulette sampling, preventing dimention explode
                );
            }
        }

        hitColor = L_dir + L_indir;

        // consider self radiance
        hitColor += m->getEmission();

    }
    
    // return
    return hitColor;
}
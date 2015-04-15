#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec.h"
#include "modelerdraw.h"
#include <FL/gl.h>

static int idGenerator = 0;
class Particle
{
public:
    Particle(Vec3f _pos, double _mass, Vec3f _color, float _lifespan, Vec3f _vel, int _type)
    {
    	id = idGenerator;
        idGenerator++;
    	pos = _pos;
        prevPos = _pos;
    	vel = _vel;
    	mass =_mass;
        radius = 0.02*mass;
        // radius = 0.5*mass;
    	color = _color;
    	lifespan = _lifespan;
        type = _type;
    	age = 0;
    }
    int id;
    int type;
    Vec3f pos;
    Vec3f prevPos;
    Vec3f vel;
    Vec3f color;
    double mass;
    double radius;
    double lifespan;
    double age;
    void draw()
    {
        setDiffuseColor(color[0], color[1], color[2]);
        setAmbientColor(color[0], color[1], color[2]);
        glPushMatrix();
            glTranslatef(pos[0], pos[1], pos[2]);
            drawSphere(radius);
        glPopMatrix();
    }

    bool isDead()
    {
    	return age > lifespan;
    }

    void incrementAge()
    {
    	age++;
    }
};

#endif
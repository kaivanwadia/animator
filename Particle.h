#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec.h"
#include <FL/gl.h>

static int idGenerator = 0;
class Particle
{
public:
    Particle(Vec3f _pos, double _mass, double _size, Vec4f _color, float _lifespan, Vec3f _vel)
    {
    	id = idGenerator;
        idGenerator++;
    	pos = _pos;
    	vel = _vel;
    	mass =_mass;
    	size = _size;
    	color = _color;
    	lifespan = _lifespan;
    	age = 0;
    	forces = Vec3f(0,0,0);
    }
    int id;
    Vec3f pos;
    Vec3f vel;
    Vec4f color;
    double mass;
    double size;
    float lifespan;
    float age;
    Vec3f forces;
    void draw()
    {
    	glPointSize(size);
    	glBegin(GL_POINTS);
	    	glColor4f(color[0], color[1], color[2], color[3]);
	    	glVertex3f(pos[0], pos[1], pos[2]);
    	glEnd();
    }

    bool isDead()
    {
    	return age > lifespan;
    }

    void incrementAge()
    {
    	age++;
    }

    void clearForce() {
	    forces[0] = 0.0;
	    forces[1] = 0.0;
	    forces[2] = 0.0;
	}
};

#endif
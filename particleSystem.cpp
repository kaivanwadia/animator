#pragma warning(disable : 4786)

#include "particleSystem.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>

#include <FL/gl.h>

using namespace std;

static float prevT;

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	totalNoOfParticles = 0;
	maxPartPerFrame = 6;
	gravity = -9.8;
	dragCoeff = 1;
	gravityForce = true;
	dragForce = true;
	maxVelocity = 2;
}




/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	// TODO
}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
	// These values are used by the UI ...
	// negative bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;
}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
	// These values are used by the UI
	simulate = false;
	dirty = true;
}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
	// These values are used by the UI
	simulate = false;
	dirty = true;
}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
	if(t == prevT) return;	//no time has passed
    
    int numberErased = 0;

	if (simulate) {
		//EMIT new particles
		emitParticles();

		std::set<int> particlesToDelete;
		for(int i=0; i<(int)particles.size(); i++)
	    {
	        particles[i].incrementAge();
	        if (particles[i].isDead())
	        {
	        	particlesToDelete.insert(i);
	        	numberErased++;
	        	continue;
	        }
	    }

	    vector<Particle> newparticles;
	    if(!particlesToDelete.empty())
	    {
	    	for(int i=0; i<(int)particles.size(); i++)
	        {
	            if(particlesToDelete.count(i) == 0)
	            {
	                newparticles.push_back(particles[i]);
	            }
	        }
	        particles = newparticles;
	    }
		for(auto partItr = particles.begin(); partItr != particles.end(); ++partItr) {
			//Clear forces
			partItr->clearForce();

			//ADD FORCES:
			if (gravityForce)
			{
				// cout << "Gravity\n";
				partItr->forces[0] += 0.0;
				partItr->forces[1] += partItr->mass*gravity;
				partItr->forces[2] += 0.0;
			}
			if (dragForce)
			{
				// cout << "Drag\n";
				partItr->forces[0] += partItr->vel[0] * dragCoeff;
				partItr->forces[1] += partItr->vel[1] * dragCoeff;
				partItr->forces[2] += partItr->vel[2] * dragCoeff;
			}
		}
		//SOLVER
		numericalIntegration(t-prevT);
	}
	
	// Debugging info
	if( t - prevT > .04 )
		printf("(!!) Dropped Frame %lf (!!)\n", t-prevT);
	prevT = t;
	//cout << "Erased: " << numberErased << endl;
	//fcout << "Particles: " << particles.size() << endl;
}

void ParticleSystem::numericalIntegration(float timeStep)
{
	for(auto partItr = particles.begin(); partItr != particles.end(); ++partItr) {
		partItr->pos[0] = partItr->pos[0] + (timeStep * partItr->vel[0]);
		partItr->pos[1] = partItr->pos[1] + (timeStep * partItr->vel[1]);
		partItr->pos[2] = partItr->pos[2] + (timeStep * partItr->vel[2]);
		// p->setPosition( p->getPosition() + (dt * p->getVelocity()) );
		partItr->vel[0] = partItr->vel[0] + (timeStep * (partItr->forces[0]/partItr->mass));
		partItr->vel[1] = partItr->vel[1] + (timeStep * (partItr->forces[1]/partItr->mass));
		partItr->vel[2] = partItr->vel[2] + (timeStep * (partItr->forces[2]/partItr->mass));
		// p->setVelocity( p->getVelocity() + (dt * (p->getForce()/p->getMass()) ) );
	}
}

/** Emitting Particles **/
void ParticleSystem::emitParticles()
{
	int noOfPart = rand() % maxPartPerFrame + 1;
	for (int i = 0; i < noOfPart; i++)
	{
		float x = rand()/float(RAND_MAX) - 0.5;
		float y = rand()/float(RAND_MAX);
		float z = rand()/float(RAND_MAX) - 0.5;
		Vec3f pos = Vec3f(x, y, z);
		cout << "Pos : " << pos[0] << "\t" << pos[1] << "\t" << pos[2] << "\n";
		pos = pos + emitPosition;
		cout << "Emit Pos : " << emitPosition[0] << "\t" << emitPosition[1] << "\t" << emitPosition[2] << "\n";
		Vec3f vel = getRandomVelocity(maxVelocity*2, -maxVelocity);
		// Vec3f vel = Vec3f(7, 7, 0.0);
		Vec4f color = Vec4f(1.0, 0.0, 0.0, 1.0);
	    double mass = rand() % 8 + 1;
	    double size = rand() % 10 + 2;
	    float lifespan = rand() % 100 + 0;
	    Particle p = Particle(pos, mass, size, color, lifespan, vel);
	    particles.push_back(p);
	}
	totalNoOfParticles += noOfPart;
}

Vec3f ParticleSystem::getRandomVelocity(int max, int min)
{
	Vec3f vel;
	vel[0] = rand() % max + min;
	vel[1] = abs(rand() % max + min);
	vel[2] = rand() % max + min;
	return vel;
}


/** Render particles */
void ParticleSystem::drawParticles(float t)
{
	// TODO
	// cout << "In drawParticles of ParticleSystem \n";
    for(auto partItr = particles.begin(); partItr != particles.end(); ++partItr) {
        partItr->draw();
    }
}

void ParticleSystem::setEmitterPosition(Vec3f _emitPos)
{
	emitPosition[0] = _emitPos[0];
	emitPosition[1] = _emitPos[1];
	emitPosition[2] = _emitPos[2];
	// emitPosition[0] = -5.0;
	// emitPosition[1] = 3.0;
	// emitPosition[2] = 0.0;
}

/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{
	// TODO
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	// TODO
}
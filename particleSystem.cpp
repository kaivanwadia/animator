#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "animatoruiwindows.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>
#include <string>

#include <FL/gl.h>

using namespace std;

static float prevT;

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	totalNoOfParticles = 0;
	maxPartPerFrame = 2;
	gravity = -ModelerUIWindows::m_nGravity;
	dragCoeff = -ModelerUIWindows::m_nDragCoeff;
	floorStiff = ModelerUIWindows::m_nFlStiff;
	floorDrag = ModelerUIWindows::m_nFlDrag;
	maxVelocityChimney = 4; //5
	maxVelocityClaw = 3;
	timeStep = 0;
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

	gravity = -ModelerUIWindows::m_nGravity;
	dragCoeff = -ModelerUIWindows::m_nDragCoeff;
	floorStiff = ModelerUIWindows::m_nFlStiff;
	floorDrag = ModelerUIWindows::m_nFlDrag;
	timeStep = t-prevT;
    
    int numberErased = 0;

	if (simulate) {

		// Kill dead particles
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

	    //EMIT new particles
		emitParticles();
		vector<float> q, qprev, vel;
		// Build configuration
		buildConfiguration(q, qprev, vel);
		// printVector(vel, "Start Vel");
		numericalIntegration(q, qprev, vel);

		unbuildConfiguration(q, vel);
	}
	
	// Debugging info
	if( t - prevT > .04 )
		printf("(!!) Dropped Frame %lf (!!)\n", t-prevT);
	prevT = t;
	//cout << "Erased: " << numberErased << endl;
	//fcout << "Particles: " << particles.size() << endl;
}

void ParticleSystem::printVector(vector<float> &v, string name) const
{
	cout << name << " : \n";
	for (int i = 0; i < v.size(); i++)
	{
		cout << i << " : " << v[i] << "\n";
	}
	cout << "\n";
}

void ParticleSystem::numericalIntegration(vector<float> &q, vector<float> &qprev, vector<float> &vel)
{
    std::vector<float> forces;

    std::vector<float> oldq = q;
    for (int i = 0; i < q.size(); i++)
    {
    	q[i] += timeStep*vel[i];
    }
    computeForceAndHessian(q, oldq, vel, forces);
    for (int i = 0; i < q.size(); i++)
    {
    	int index = i/3;
    	float tempVel = timeStep * forces[i]/particles[index].mass;
    	vel[i] = vel[i] + tempVel;
    }
}

void ParticleSystem::computeForceAndHessian(vector<float> &q, vector<float> &qprev, vector<float> &vel, vector<float> &forces)
{
    forces.clear();
    forces.resize(q.size(), 0);
    processGravityForce(forces);
    processDragForce(vel, forces);
    processFloorForce(q, qprev, forces);

    // if(params_.activeForces & SimParameters::F_SPRINGS)
    //     processSpringForce(q, F, Hcoeffs);
    // if(params_.activeForces & SimParameters::F_DAMPING)
    //     processDampingForce(q, qprev, F, Hcoeffs);
    // if(params_.activeForces & SimParameters::F_FLOOR)
    //     processFloorForce(q, qprev, F, Hcoeffs);
    // if(params_.activeForces & SimParameters::F_ELASTIC)
    //     processElasticBendingForce(q, F);
    // if(params_.constraint == SimParameters::CH_PENALTY_FORCE)
    //     processPenaltyForce(q, F);

    // H.setFromTriplets(Hcoeffs.begin(), Hcoeffs.end());

}

void ParticleSystem::processGravityForce(vector<float> &forces)
{
    int nparticles = (int)particles.size();
    for(int i=0; i<nparticles; i++)
    {
        forces[(3*i)+1] += gravity*particles[i].mass;
    }
}

void ParticleSystem::processDragForce(vector<float> &vel, vector<float> &forces)
{
    int nparticles = (int)particles.size();
    for(int i=0; i<nparticles; i++)
    {
        forces[(3*i)] = forces[(3*i)] + dragCoeff*vel[(3*i)];
        forces[(3*i) + 1] = forces[(3*i) + 1] + dragCoeff*vel[(3*i) + 1];
        forces[(3*i) + 2] = forces[(3*i) + 2] + dragCoeff*vel[(3*i) + 2];
    }
}

void ParticleSystem::processFloorForce(vector<float> &q, vector<float> &qprev, vector<float> &forces)
{
    int nparticles = particles.size();
    for(int i=0; i<nparticles; i++)
    {
        if(q[3*i+1] < 0.05)
        {
            double vel = (q[3*i+1]-qprev[3*i+1])/timeStep;
            if (vel > 0)
            {
            	continue;
            }
            double dist = 0.05 - q[3*i+1];
            float stiffForce = floorStiff*dist;
            // float dragForce = floorDrag*dist*vel;
            float dragForce = stiffForce * -0.7;
            forces[3*i+1] = forces[3*i + 1] + stiffForce + dragForce;
        }
    }
}

/** Emitting Particles **/
void ParticleSystem::emitParticles()
{
	int noOfPart = rand() % maxPartPerFrame + 1;
	// noOfPart = 1;
	// if (particles.size() >= 1)
	// {
	// 	return;
	// }
	for (int i = 0; i < noOfPart; i++)
	{
		float x = rand()/float(RAND_MAX) - 0.5;
		float y = rand()/float(RAND_MAX);
		float z = rand()/float(RAND_MAX) - 0.5;
		Vec3f pos = Vec3f(x, y, z);
		// cout << "Pos : " << pos[0] << "\t" << pos[1] << "\t" << pos[2] << "\n";
		pos = pos + emitPosition;
		// cout << "Emit Pos : " << emitPosition[0] << "\t" << emitPosition[1] << "\t" << emitPosition[2] << "\n";
		Vec3f vel = getRandomVelocity(maxVelocityChimney*2, -maxVelocityChimney);
		// Vec3f vel = Vec3f(7, 7, 0.0);
		Vec4f color = Vec4f(1.0, 0.0, 0.0, 1.0);
	    double mass = rand() % 5 + 2;
	    // double mass = 1;
	    float lifespan = rand() % 200 + 1;
	    // float lifespan = 1000;
	    Particle p = Particle(pos, mass, color, lifespan, vel);
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

void ParticleSystem::buildConfiguration(vector<float> &q, vector<float> &qprev, vector<float> &vel)
{
	int ndofs = 3*particles.size();
	q.clear();
	qprev.clear();
	vel.clear();
    q.resize(ndofs);
    qprev.resize(ndofs);
    vel.resize(ndofs);

    // cout << "In buildConfiguration : \n";
    for(int i=0; i<particles.size(); i++)
    {
        q[3*i] = particles[i].pos[0];
        q[(3*i) + 1] = particles[i].pos[1];
        q[(3*i) + 2] = particles[i].pos[2];

        qprev[3*i] = particles[i].prevPos[0];
        qprev[(3*i) + 1] = particles[i].prevPos[1];
        qprev[(3*i) + 2] = particles[i].prevPos[2];

        // cout << 3*i << "v : " << particles[i].vel[0] << "\n";
        // cout << (3*i) + 1<< "v : " << particles[i].vel[1] << "\n";
        // cout << (3*i) + 2<< "v : " << particles[i].vel[2] << "\n";
        vel[3*i] = particles[i].vel[0];
        vel[(3*i) + 1] = particles[i].vel[1];
        vel[(3*i) + 2] = particles[i].vel[2];
        // cout << 3*i << "vel : " << vel[3*i] << "\n";
        // cout << (3*i) + 1<< "vel : " << vel[(3*i) + 1] << "\n";
        // cout << (3*i) + 2<< "vel : " << vel[(3*i) + 2] << "\n";
    }
}

void ParticleSystem::unbuildConfiguration(vector<float> &q, vector<float> &vel)
{
    int ndofs = q.size();
    assert(ndofs == int(3*particles.size()));
    for(int i=0; i<ndofs/3; i++)
    {
        particles[i].prevPos[0] = particles[i].pos[0];
        particles[i].prevPos[1] = particles[i].pos[1];
        particles[i].prevPos[2] = particles[i].pos[2];
        particles[i].pos[0] = q[3*i];
        particles[i].pos[1] = q[(3*i) + 1];
        particles[i].pos[2] = q[(3*i) + 2];
        particles[i].vel[0] = vel[3*i];
        particles[i].vel[1] = vel[(3*i) + 1];
        particles[i].vel[2] = vel[(3*i) + 2];
    }
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
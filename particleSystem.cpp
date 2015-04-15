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
static int timeStepcount = 0;
ParticleSystem::ParticleSystem() 
{
	totalNoOfParticles = 0;
	maxPartPerFrame = 2;
	gravity = -ModelerUIWindows::m_nGravity;
	dragCoeff = -ModelerUIWindows::m_nDragCoeff;
	floorStiff = ModelerUIWindows::m_nFlStiff;
	floorDrag = ModelerUIWindows::m_nFlDrag;
	penaltyStiffness = ModelerUIWindows::m_nPenaltyStiffness;
	maxVelocityChimney = ModelerUIWindows::m_nMaxChimVel;
	maxVelocityClaw = ModelerUIWindows::m_nMaxClawVel;
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
	particles.clear();
	bakeParticles(t);
	simulate = false;
	dirty = true;
}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
	// These values are used by the UI
	cout << "In resetSimulation\n";
	particles.clear();
	clearBaked();
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
	penaltyStiffness = ModelerUIWindows::m_nPenaltyStiffness;
	maxVelocityChimney = ModelerUIWindows::m_nMaxChimVel;
	maxVelocityClaw = ModelerUIWindows::m_nMaxClawVel;
	timeStep = t-prevT;
    
    int numberErased = 0;

	if (simulate) {

		// Kill dead particles
		timeStepcount++;
		bakeParticles(t);
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
    if (ModelerUIWindows::m_bCollisions)
    {
    	processCollisionForce(q, vel, forces);
    }

    // if(params_.activeForces & SimParameters::F_SPRINGS)
    //     processSpringForce(q, F, Hcoeffs);
    // if(params_.activeForces & SimParameters::F_DAMPING)
    //     processDampingForce(q, qprev, F, Hcoeffs);
}

void ParticleSystem::processCollisionForce(vector<float> &q, vector<float> &vel, vector<float> &forces)
{
	// cout << "Entering Collision Force : " << particles.size() << "\n";
	for (int p1id = 0; p1id < particles.size(); p1id++)
	{
		Vec3f p1Pos = Vec3f(q[3*p1id + 0], q[3*p1id + 1], q[3*p1id + 2]);
		for (int p2id = 0; p2id < particles.size(); p2id++)
		{
			Vec3f p2Pos = Vec3f(q[3*p2id + 0], q[3*p2id + 1], q[3*p2id + 2]);
			if (p1id == p2id) // Same particle
			{
				continue;
			}
			if (particles[p1id].type == particles[p2id].type) // Same type of particle
			{
				continue;
			}
			float minDist = particles[p1id].radius + particles[p2id].radius;
			minDist = minDist*minDist;
			float currentDistSq = distSquared(p1Pos, p2Pos);
			if (currentDistSq < minDist) // Collision has occured. Apply a force
			{
				Vec3f forceP1 = penaltyStiffness * sqrt(currentDistSq) * 2 * (p1Pos - p2Pos); // This force should be negative
				forces[p1id*3 + 0] = forces[p1id*3 + 0] + forceP1[0];
				forces[p1id*3 + 1] = forces[p1id*3 + 1] + forceP1[1];
				forces[p1id*3 + 2] = forces[p1id*3 + 2] + forceP1[2];

				forces[p2id*3 + 0] = forces[p2id*3 + 0] - forceP1[0];
				forces[p2id*3 + 1] = forces[p2id*3 + 1] - forceP1[1];
				forces[p2id*3 + 2] = forces[p2id*3 + 2] - forceP1[2];
			}
		}
	}
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
        if(q[3*i+1] < 0.05 + particles[i].radius)
        {
            double vel = (q[3*i+1]-qprev[3*i+1])/timeStep;
            if (vel > 0)
            {
            	continue;
            }
            double dist = (0.05 + particles[i].radius) - q[3*i+1];
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
	int noOfPart = rand() % ModelerUIWindows::m_nPartPerFrame + 1;
	// noOfPart = 1;
	int totPartEmitted = 0;
	if (particles.size() >= ModelerUIWindows::m_nMaxParticles)
	{
		return;
	}
	for (int emitterNo = 0; emitterNo < emitPositions.size(); emitterNo++)
	{
		if (emitterNo == 1 && !ModelerUIWindows::m_bClawEmit)
		{
			continue;
		}
		if (emitterNo == 0 && !ModelerUIWindows::m_bChimEmit)
		{
			continue;
		}
		for (int i = 0; i < noOfPart; i++)
		{
			// float x = -3;
			// float y = 2.0;
			// float z = -2;
			// Vec3f pos = Vec3f(x, y, z);
			// Vec3f vel = Vec3f(4, 0.0, -0);
			// Vec3f color = Vec3f(0, 1, 0);
			// double mass = 1;
			// float lifespan = 500;
			// Particle p = Particle(pos, mass, color, lifespan, vel, 0);
			// particles.push_back(p);

			// float x2 = 3;
			// float y2 = 2.0;
			// float z2 = -2;
			// Vec3f pos2 = Vec3f(x2, y2, z2);
			// Vec3f vel2 = Vec3f(-4, 0.0, 0);
			// Vec3f color2 = Vec3f(1, 0, 0);
			// double mass2 = 1;
			// float lifespan2 = 500;
			// Particle p2 = Particle(pos2, mass2, color2, lifespan2, vel2, 1);
			// particles.push_back(p2);

			if (emitterNo == 0)
			{
				emitChimneyParticle();
			}
			else if (emitterNo == 1)
			{
				emitClawParticle();
			}
			totPartEmitted++;
		}
	}
	totalNoOfParticles += totPartEmitted;
}

void ParticleSystem::emitChimneyParticle()
{
	// float x = rand()/float(RAND_MAX) - 0.5;
	// float y = rand()/float(RAND_MAX);
	// float z = rand()/float(RAND_MAX) - 0.5;
	// Vec3f pos = Vec3f(x, y, z);
	// pos = pos + emitPositions[1];
	Vec3f pos = emitPositions[0];
	Vec3f vel;
	int max = maxVelocityChimney*2;
	int min = -maxVelocityChimney;
	if (ModelerUIWindows::m_bCollisions)
	{
		Vec3f direction = emitPositions[1] - emitPositions[0];
		direction.normalize();
		direction[1] = 1;
		vel[0] = rand() % max + 1;
		vel[1] = rand() % max + 1;
		vel[2] = rand() % max + 1;
		vel = prod(vel, direction);
	}
	else
	{
		vel[0] = rand() % max + min;
		vel[1] = abs(rand() % max + min);
		vel[2] = rand() % max + min;
	}
	Vec3f color = Vec3f(ModelerUIWindows::m_nRed, ModelerUIWindows::m_nGreen, ModelerUIWindows::m_nBlue);
	double mass = rand() % 5 + 2;
	float lifespan = rand() % ModelerUIWindows::m_nLifespan + 1;
	int type = 0;
	Particle p = Particle(pos, mass, color, lifespan, vel, type);
	particles.push_back(p);
}

void ParticleSystem::emitClawParticle()
{
	// float x = rand()/float(RAND_MAX) - 0.5;
	// float y = rand()/float(RAND_MAX);
	// float z = rand()/float(RAND_MAX) - 0.5;
	// Vec3f pos = Vec3f(x, y, z);
	// pos = pos + emitPositions[1];
	Vec3f pos = emitPositions[1];
	Vec3f vel;
	int max = maxVelocityChimney*2;
	int min = -maxVelocityChimney;
	if (ModelerUIWindows::m_bCollisions)
	{
		Vec3f direction = emitPositions[0] - emitPositions[1];
		direction.normalize();
		direction[1] = 1;
		vel[0] = rand() % max + 1;
		vel[1] = rand() % max + 1;
		vel[2] = rand() % max + 1;
		vel = prod(vel, direction);
	}
	else
	{
		Vec3f direction = emitPositions[1];
		direction.direction();
		vel[0] = rand() % max + 1;
		vel[1] = 0; //rand() % max + 1;
		vel[2] = rand() % max + 1;
		vel = prod(vel, direction);
	}
	Vec3f color = Vec3f(ModelerUIWindows::m_nRed, ModelerUIWindows::m_nGreen, ModelerUIWindows::m_nBlue);
	color[0] = 1.0 - color[0];
	color[1] = 1.0 - color[1];
	color[2] = 1.0 - color[2];
	double mass = rand() % 5 + 2;
	float lifespan = rand() % ModelerUIWindows::m_nLifespan + 1;
	int type = 1;
	Particle p = Particle(pos, mass, color, lifespan, vel, type);
	particles.push_back(p);
}

/** Render particles */
void ParticleSystem::drawParticles(float t)
{
	// cout << "In drawParticles() \n";
	if (!simulate)
	{
		// Then draw baked if baked exists for time t
		if (bakedParticleMap.empty())
		{
			return;
		}
		else if (bake_start_time > t || bake_end_time < t)
		{
			return;
		}
		else if (t >= bake_start_time && t <= bake_end_time)
		{
			vector<Particle> bakedParticles = (*(--bakedParticleMap.end())).second;
	        for(auto itr = bakedParticleMap.begin(); itr != bakedParticleMap.end(); ++itr)
	        {
	        	float mapTime = (*itr).first;
	            if(t == mapTime)
	            {
	                bakedParticles = (*itr).second;
	                break;
	            }
	            else if(t<mapTime)
	            {
	            	bakedParticles = (*(--itr)).second;
	                break;
	            }
	        }
	        for(auto partItr = bakedParticles.begin(); partItr != bakedParticles.end(); ++partItr) {
		        partItr->draw();
		    }
		}
		else
		{
			cout << " Should never come here \n";
		}
	}
	else
	{
		for(auto partItr = particles.begin(); partItr != particles.end(); ++partItr) {
	        partItr->draw();
	    }
	}
}

void ParticleSystem::setEmitterPosition(Vec3f _emitPos, int index)
{
	if (index >= emitPositions.size())
	{
		addEmitterPosition(_emitPos);
	}
	else
	{
		Vec3f &emitPosition = emitPositions[index];
		emitPosition[0] = _emitPos[0];
		emitPosition[1] = _emitPos[1];
		emitPosition[2] = _emitPos[2];
	}
}

void ParticleSystem::addEmitterPosition(Vec3f _emitPos)
{
	Vec3f emitPosition;
	emitPosition[0] = _emitPos[0];
	emitPosition[1] = _emitPos[1];
	emitPosition[2] = _emitPos[2];
	emitPositions.push_back(emitPosition);
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

    for(int i=0; i<particles.size(); i++)
    {
        q[3*i] = particles[i].pos[0];
        q[(3*i) + 1] = particles[i].pos[1];
        q[(3*i) + 2] = particles[i].pos[2];

        qprev[3*i] = particles[i].prevPos[0];
        qprev[(3*i) + 1] = particles[i].prevPos[1];
        qprev[(3*i) + 2] = particles[i].prevPos[2];

        vel[3*i] = particles[i].vel[0];
        vel[(3*i) + 1] = particles[i].vel[1];
        vel[(3*i) + 2] = particles[i].vel[2];
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

float ParticleSystem::distSquared(Vec3f p1Pos, Vec3f p2Pos)
{
	float xSq = (p1Pos[0] - p2Pos[0])*(p1Pos[0] - p2Pos[0]);
	float ySq = (p1Pos[1] - p2Pos[1])*(p1Pos[1] - p2Pos[1]);
	float zSq = (p1Pos[2] - p2Pos[2])*(p1Pos[2] - p2Pos[2]);
	return xSq + ySq + zSq;
}

/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{
	// TODO
	// cout << "In bakeParticles() \n";
	if (bakedParticleMap.size() == 0)
	{
		bake_start_time = t;
	}
	bake_end_time = t;
	bakedParticleMap[t] = particles;
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	// TODO
	cout << "In clearBaked() \n";
	bakedParticleMap.clear();
}
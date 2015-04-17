#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "animatoruiwindows.h"
#include "modelerdraw.h"

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
	maxVelocityTank1 = ModelerUIWindows::m_nMaxTank1Vel;
	maxVelocityTank2 = ModelerUIWindows::m_nMaxTank2Vel;
	springMaxStrain = ModelerUIWindows::m_nMaxSpringStrain;
	springDampingStiffness = ModelerUIWindows::m_nDampStiffness;
	emitPositions.resize(4, Vec3f(0,0,0));
	emitDirections.resize(4, Vec3f(0,0,0));
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
	springs.clear();
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
	springs.clear();
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
	maxVelocityTank1 = ModelerUIWindows::m_nMaxTank1Vel;
	maxVelocityTank2 = ModelerUIWindows::m_nMaxTank2Vel;
	springMaxStrain = ModelerUIWindows::m_nMaxSpringStrain;
	springDampingStiffness = ModelerUIWindows::m_nDampStiffness;
	timeStep = t-prevT;

	if (simulate) {

		// Kill dead particles
		timeStepcount++;
		bakeParticles(t);

		// Killing particles
		deleteParticles();
		pruneOverstrainedSprings();

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

void ParticleSystem::deleteParticles()
{
	int numberErased = 0;
	set<int> particlesToDelete;
	set<int> springsToDelete;
	vector<int> remainingParticleMap;
    vector<int> remainingSpringMap;
    vector<Particle> newparticles;
    vector<Spring> newsprings;
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
    if (!particlesToDelete.empty())
    {
    	for(int i=0; i<(int)springs.size(); i++)
        {
            if(particlesToDelete.count(springs[i].p1Id) || particlesToDelete.count(springs[i].p2Id))
            {
                springsToDelete.insert(i);
            }
        }
        for(int i=0; i<(int)particles.size(); i++)
        {
            if(particlesToDelete.count(i) == 0)
            {
                remainingParticleMap.push_back(newparticles.size());
                newparticles.push_back(particles[i]);
            }
            else
                remainingParticleMap.push_back(-1);
        }
    }
    if(!springsToDelete.empty())
    {
        for(int i=0; i<(int)springs.size(); i++)
        {
            if(springsToDelete.count(i) == 0)
            {
                remainingSpringMap.push_back(newsprings.size());
                newsprings.push_back(springs[i]);
            }
            else
            {
                remainingSpringMap.push_back(-1);
            }
        }
    }
    if(!springsToDelete.empty() || !particlesToDelete.empty())
    {
        if(!springsToDelete.empty())
        {
            springs = newsprings;
        }
        if(!particlesToDelete.empty())
        {
            particles = newparticles;
            for(auto it = springs.begin(); it != springs.end(); ++it)
            {
                it->p1Id = remainingParticleMap[it->p1Id];
                it->p2Id = remainingParticleMap[it->p2Id];
            }
        }
    }
}

void ParticleSystem::pruneOverstrainedSprings()
{
    set<int> springsToDelete;

    vector<Spring> newsprings;
    vector<int> remainingSpringMap;
    for(int i=0; i<springs.size(); i++)
    {
    	Vec3f p1Pos = particles[springs[i].p1Id].pos;
    	Vec3f p2Pos = particles[springs[i].p2Id].pos;
    	double dist = (p2Pos - p1Pos).length();
    	double strain = (dist - springs[i].restLength)/springs[i].restLength;
    	if (strain > springMaxStrain)
    	{
    		springsToDelete.insert(i);
    	}
    }
    if (!springsToDelete.empty())
    {
    	for(int i=0; i<springs.size(); i++)
    	{
    		if (springsToDelete.count(i) == 0)
    		{
    			newsprings.push_back(springs[i]);
    		}
    	}
    	springs = newsprings;
    }
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
    if (ModelerUIWindows::m_bSprings)
    {
    	processSpringForce(q, forces);
    	processDampingForce(q, qprev, forces);
    }
}

void ParticleSystem::processDampingForce(vector<float> &q, vector<float> &qprev, vector<float> &forces)
{
	for (int i = 0; i < springs.size(); i++)
	{
		int p1Index = springs[i].p1Id;
		int p2Index = springs[i].p2Id;
		Vec3f p1Pos = Vec3f(q[3*p1Index], q[3*p1Index + 1], q[3*p1Index + 2]);
		Vec3f p2Pos = Vec3f(q[3*p2Index], q[3*p2Index + 1], q[3*p2Index + 2]);
		Vec3f p1PrevPos = Vec3f(qprev[3*p1Index], qprev[3*p1Index + 1], qprev[3*p1Index + 2]);
		Vec3f p2PrevPos = Vec3f(qprev[3*p2Index], qprev[3*p2Index + 1], qprev[3*p2Index + 2]);
		Vec3f relVel = (p2Pos - p2PrevPos)/timeStep - (p1Pos - p1PrevPos)/timeStep;
		Vec3f force = springDampingStiffness * relVel;
		forces[3*p1Index + 0] = forces[3*p1Index + 0] + force[0];
		forces[3*p1Index + 1] = forces[3*p1Index + 1] + force[1];
		forces[3*p1Index + 2] = forces[3*p1Index + 2] + force[2];

		forces[3*p2Index + 0] = forces[3*p2Index + 0] - force[0];
		forces[3*p2Index + 1] = forces[3*p2Index + 1] - force[1];
		forces[3*p2Index + 2] = forces[3*p2Index + 2] - force[2];

	}
}

void ParticleSystem::processSpringForce(vector<float> &q, vector<float> &forces)
{
	for (int i = 0; i < springs.size(); i++)
	{
		int p1Index = springs[i].p1Id;
		int p2Index = springs[i].p2Id;
		Vec3f p1Pos = Vec3f(q[3*p1Index], q[3*p1Index + 1], q[3*p1Index + 2]);
		Vec3f p2Pos = Vec3f(q[3*p2Index], q[3*p2Index + 1], q[3*p2Index + 2]);
		double dist = (p2Pos-p1Pos).length();
		Vec3f force = springs[i].stiffness * (dist - springs[i].restLength)/dist * (p2Pos - p1Pos);
		forces[3*p1Index + 0] = forces[3*p1Index + 0] + force[0];
		forces[3*p1Index + 1] = forces[3*p1Index + 1] + force[1];
		forces[3*p1Index + 2] = forces[3*p1Index + 2] + force[2];

		forces[3*p2Index + 0] = forces[3*p2Index + 0] - force[0];
		forces[3*p2Index + 1] = forces[3*p2Index + 1] - force[1];
		forces[3*p2Index + 2] = forces[3*p2Index + 2] - force[2];
	}
}

void ParticleSystem::processCollisionForce(vector<float> &q, vector<float> &vel, vector<float> &forces)
{
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
        if(q[3*i+1] < 0.1 + particles[i].radius)
        {
            double vel = (q[3*i+1]-qprev[3*i+1])/timeStep;
            if (vel > 0)
            {
            	continue;
            }
            double dist = (0.1 + particles[i].radius) - q[3*i+1];
            float stiffForce = floorStiff*dist;
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
	int x = 3+5;
	for (int emitterNo = 0; emitterNo < emitPositions.size(); emitterNo++)
	{
		if (emitterNo == 0 && !ModelerUIWindows::m_bt1TurEmit)
		{
			continue;
		}
		if (emitterNo == 1 && !ModelerUIWindows::m_bt1GunEmit)
		{
			continue;
		}
		if (emitterNo == 2 && !ModelerUIWindows::m_bt2TurEmit)
		{
			continue;
		}
		if (emitterNo == 3 && !ModelerUIWindows::m_bt2GunEmit)
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
			if (ModelerUIWindows::m_bSprings && (emitterNo == 1 || emitterNo == 3))
			{
				Vec3f pos = emitPositions[emitterNo];
				Vec3f velDir = emitDirections[emitterNo];
				double mass = rand() % 5 + 2;
				float lifespan = rand() % ModelerUIWindows::m_nLifespan + 1;

				Vec3f color = Vec3f(ModelerUIWindows::m_nRed, ModelerUIWindows::m_nGreen, ModelerUIWindows::m_nBlue);			
				Vec3f vel;
				int type;
				if (emitterNo == 0 || emitterNo == 1)
				{
					vel[0] = rand() % maxVelocityTank1 + 1;
					vel[1] = rand() % maxVelocityTank1 + 1;
					vel[2] = rand() % maxVelocityTank1 + 1;
					type = 0;
				}
				else if (emitterNo == 2 || emitterNo == 3)
				{
					color[0] = 1.0 - color[0];
					color[1] = 1.0 - color[1];
					color[2] = 1.0 - color[2];
					vel[0] = rand() % maxVelocityTank2 + 1;
					vel[1] = rand() % maxVelocityTank2 + 1;
					vel[2] = rand() % maxVelocityTank2 + 1;
					type = 1;
				}
				vel = prod(vel, velDir);
				Particle p1 = Particle(pos, mass, color, lifespan, vel, type);
				particles.push_back(p1);
				int p1ID = particles.size()-1;
				Vec3f newPos = pos;
				double rLen = ModelerUIWindows::m_nSpringRestLen;
				newPos[0] = newPos[0] + ((double(rand())/RAND_MAX) * rLen * 2 - rLen);
				newPos[1] = newPos[1] + ((double(rand())/RAND_MAX) * rLen * 2 - rLen);
				newPos[2] = newPos[2] + ((double(rand())/RAND_MAX) * rLen * 2 - rLen);
				mass = rand() % 5 + 2;
				lifespan = rand() % ModelerUIWindows::m_nLifespan + 1;
				Particle p2 = Particle(newPos, mass, color, lifespan, vel, type);
				particles.push_back(p2);
				int p2ID = particles.size()-1;

				Spring spring = Spring(p1ID, p2ID, rLen, ModelerUIWindows::m_nSpringStiffness);
				springs.push_back(spring);
				totPartEmitted++;
				i++;
				continue;
			}

			Vec3f pos = emitPositions[emitterNo];
			Vec3f velDir = emitDirections[emitterNo];
			double mass = rand() % 5 + 2;
			float lifespan = rand() % ModelerUIWindows::m_nLifespan + 1;

			Vec3f color = Vec3f(ModelerUIWindows::m_nRed, ModelerUIWindows::m_nGreen, ModelerUIWindows::m_nBlue);			
			Vec3f vel;
			int type;
			if (emitterNo == 0 || emitterNo == 1)
			{
				vel[0] = rand() % maxVelocityTank1 + 1;
				vel[1] = rand() % maxVelocityTank1 + 1;
				vel[2] = rand() % maxVelocityTank1 + 1;
				type = 0;
			}
			else if (emitterNo == 2 || emitterNo == 3)
			{
				color[0] = 1.0 - color[0];
				color[1] = 1.0 - color[1];
				color[2] = 1.0 - color[2];
				vel[0] = rand() % maxVelocityTank2 + 1;
				vel[1] = rand() % maxVelocityTank2 + 1;
				vel[2] = rand() % maxVelocityTank2 + 1;
				type = 1;
			}
			vel = prod(vel, velDir);
			Particle p = Particle(pos, mass, color, lifespan, vel, type);
			particles.push_back(p);
			totPartEmitted++;
		}
	}
	totalNoOfParticles += totPartEmitted;
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
			vector<Spring> bakedSprings = (*(--bakedSpringMap.end())).second;
			auto sitr = bakedSpringMap.begin();
	        for(auto itr = bakedParticleMap.begin(); itr != bakedParticleMap.end(); ++itr)
	        {
	        	float mapTime = (*itr).first;
	            if(t == mapTime)
	            {
	                bakedParticles = (*itr).second;
	                bakedSprings = (*sitr).second;
	                break;
	            }
	            else if(t<mapTime)
	            {
	            	bakedParticles = (*(--itr)).second;
	            	bakedSprings = (*(--sitr)).second;
	                break;
	            }
	            ++sitr;
	        }
	        for(auto partItr = bakedParticles.begin(); partItr != bakedParticles.end(); ++partItr) {
		        partItr->draw();
		    }
		    setAmbientColor(1,1,1);
			setDiffuseColor(1,1,1);
		    glLineWidth(0.5);
			for (int i = 0; i < bakedSprings.size(); i++)
			{
				Vec3f pos1 = bakedParticles[bakedSprings[i].p1Id].pos;
				Vec3f pos2 = bakedParticles[bakedSprings[i].p2Id].pos;
				glColor4f(1, 1, 1, 1);
				glBegin(GL_LINES);
					glVertex3f(pos1[0], pos1[1], pos1[2]);
					glVertex3f(pos2[0], pos2[1], pos2[2]);
				glEnd();
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
	    setAmbientColor(1,1,1);
		setDiffuseColor(1,1,1);
	    glLineWidth(0.5);
		for (int i = 0; i < springs.size(); i++)
		{
			Vec3f pos1 = particles[springs[i].p1Id].pos;
			Vec3f pos2 = particles[springs[i].p2Id].pos;
			glBegin(GL_LINES);
				glVertex3f(pos1[0], pos1[1], pos1[2]);
				glVertex3f(pos2[0], pos2[1], pos2[2]);
			glEnd();
		}
	}
}

void ParticleSystem::setEmitterPosition(Vec3f _emitPos, int index)
{
	Vec3f &emitPosition = emitPositions[index];
	emitPosition[0] = _emitPos[0];
	emitPosition[1] = _emitPos[1];
	emitPosition[2] = _emitPos[2];
	return;
}

void ParticleSystem::setEmitterDirection(Vec3f _emitDir, int index)
{
	Vec3f &emitDirection = emitDirections[index];
	emitDirection[0] = _emitDir[0];
	emitDirection[1] = _emitDir[1];
	emitDirection[2] = _emitDir[2];
	return;
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
	bakedSpringMap[t] = springs;
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	// TODO
	cout << "In clearBaked() \n";
	bakedParticleMap.clear();
	bakedSpringMap.clear();
}
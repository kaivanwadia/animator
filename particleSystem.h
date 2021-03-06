/***********************
 * ParticleSystem class
 ***********************/

/**
 * The particle system class simply "manages" a collection of particles.
 * Its primary responsibility is to run the simulation, evolving particles
 * over time according to the applied forces using Euler's method.
 * This header file contains the functions that you are required to implement.
 * (i.e. the rest of the code relies on this interface)
 * In addition, there are a few suggested state variables included.
 * You should add to this class (and probably create new classes to model
 * particles and forces) to build your system.
 */

#ifndef __PARTICLE_SYSTEM_H__
#define __PARTICLE_SYSTEM_H__

#include "vec.h"
#include "Particle.h"
#include "mat.h"
#include <map>
#include <string>

struct Spring {
public:
    Spring(int p1, int p2, double restLength, double stiffness): p1Id(p1), p2Id(p2), restLength(restLength), stiffness(stiffness)
    {
    }
    int p1Id;
    int p2Id;
    double restLength;
    double stiffness;
};

class ParticleSystem {

public:

	/** Constructor **/
	ParticleSystem();

	/** Destructor **/
	virtual ~ParticleSystem();

	/** Simulation fxns **/
	// This fxn should render all particles in the system,
	// at current time t.
	virtual void drawParticles(float t);

	// This fxn should save the configuration of all particles
	// at current time t.
	virtual void bakeParticles(float t);

	// This function should compute forces acting on all particles
	// and update their state (pos and vel) appropriately.
	virtual void computeForcesAndUpdateParticles(float t);

	// This function should reset the system to its initial state.
	// When you need to reset your simulation, PLEASE USE THIS FXN.
	// It sets some state variables that the UI requires to properly
	// update the display.  Ditto for the following two functions.
	virtual void resetSimulation(float t);

	// This function should start the simulation
	virtual void startSimulation(float t);

	// This function should stop the simulation
	virtual void stopSimulation(float t);

	// This function should clear out your data structure
	// of baked particles (without leaking memory).
	virtual void clearBaked();	


	// These accessor fxns are implemented for you
	float getBakeStartTime() { return bake_start_time; }
	float getBakeEndTime() { return bake_end_time; }
	float getBakeFps() { return bake_fps; }
	bool isSimulate() { return simulate; }
	bool isDirty() { return dirty; }
	void setDirty(bool d) { dirty = d; }

	void deleteParticles();
	void pruneOverstrainedSprings();
	void setEmitParticles();
	void setEmitterPosition(Vec3f _emitPos, int index);
	void setEmitterDirection(Vec3f _emitDir, int index);
	void emitParticles();

	void buildConfiguration(std::vector<float> &q, std::vector<float> &qprev, std::vector<float> &vel);
	void unbuildConfiguration(std::vector<float> &q, std::vector<float> &vel);
	void numericalIntegration(std::vector<float> &q, std::vector<float> &qprev, std::vector<float> &vel);

	void computeForceAndHessian(std::vector<float> &q, std::vector<float> &qprev, std::vector<float> &vel, std::vector<float> &forces);
	void processGravityForce(std::vector<float> &forces);
	void processDragForce(std::vector<float> &vel, std::vector<float> &forces);
	void processFloorForce(std::vector<float> &q, std::vector<float> &qprev, std::vector<float> &forces);
	void processCollisionForce(std::vector<float> &q, std::vector<float> &vel, std::vector<float> &forces);
	void processSpringForce(std::vector<float> &q, std::vector<float> &forces);
	void processDampingForce(std::vector<float> &q, std::vector<float> &qprev, std::vector<float> &forces);

	void printVector(std::vector<float> &v, std::string name) const;
	float distSquared(Vec3f p1Pos, Vec3f p2Pos);


protected:
	

	/** Some baking-related state **/
	float bake_fps;						// frame rate at which simulation was baked
	float bake_start_time;				// time at which baking started 
										// These 2 variables are used by the UI for
										// updating the grey indicator 
	float bake_end_time;				// time at which baking ended

	/** General state variables **/
	bool simulate;						// flag for simulation mode
	bool dirty;							// flag for updating ui (don't worry about this)
	
	// Particle Stuff
	std::vector<Particle> particles;
	std::vector<Spring> springs;
	std::map<float, std::vector<Particle>> bakedParticleMap;
	std::map<float, std::vector<Spring>> bakedSpringMap;

	std::vector<Vec3f> emitPositions;
	std::vector<Vec3f> emitDirections;
	int maxPartPerFrame;
	int totalNoOfParticles;

	int maxVelocityTank1;
	int maxVelocityTank2;
	float timeStep;
	float gravity;
	float dragCoeff;
	float floorStiff;
	float floorDrag;
	float penaltyStiffness;
	float springMaxStrain;
	float springDampingStiffness;
};


#endif	// __PARTICLE_SYSTEM_H__

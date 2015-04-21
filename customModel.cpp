// The sample robotarm model.  You should build a file
// very similar to this for when you make your model.
#pragma warning (disable : 4305)
#pragma warning (disable : 4244)
#pragma warning(disable : 4786)
#pragma warning (disable : 4312)


#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "particleSystem.h"



#include "mat.h"
#include <FL/gl.h>
#include <cstdlib>
#include <iostream>

using namespace std;

#define M_DEFAULT 2.0f
#define M_OFFSET 3.0f
#define P_OFFSET 0.3f
#define MAX_VEL 200
#define MIN_STEP 0.1





// This is a list of the controls for the RobotArm
// We'll use these constants to access the values 
// of the controls from the user interface.
enum RobotArmControls
{ 
    BASE_ROTATION=0, LOWER_TILT, UPPER_TILT, CLAW_ROTATION,
        BASE_LENGTH, LOWER_LENGTH, UPPER_LENGTH, PARTICLE_COUNT, NUMCONTROLS, 
};

enum TANK_CONTROLS
{
	T1_ROTATION = 0, T1_SCALE, T1_X, T1_Z, T1_TOP_ROTATION, T1_TOPGUN_LENGTH, 
	T1_TOPGUN_ROT, T1_TURRET_LENGTH, T1_TURRET_ZROT, T2_ROTATION, T2_SCALE, T2_X, 
	T2_Z, T2_TOP_ROTATION, T2_TOPGUN_LENGTH, T2_TOPGUN_ROT, T2_TURRET_LENGTH, 
	T2_TURRET_ZROT, NUM_CONTROLS
};

void ground(float h);
void drawCube(float len, float wid, float hei);

void tankChasis(float len, float wid, float hei);
void drawBumper(float len, float wid, float hei);
void drawWheels(float cLen, float cWid, float cHei);
void drawTurretBase(float len, float wid, float hei);
void drawDoor(float len, float wid, float hei);
void drawTopGun(float gunLength, float gunRotation, int tankNo);
void drawTurret(float turretZRot, float turretLength, int tankNo);

void base(float h);
void rotation_base(float h);
void lower_arm(float h);
void upper_arm(float h);
void claw(float h);
void y_box(float h);
Mat4f glGetMatrix(GLenum pname);
Vec3f getWorldPoint(Mat4f matCamXforms);

ParticleSystem* ps;
Mat4f matCamInverse;

// To make a RobotArm, we inherit off of ModelerView
class RobotArm : public ModelerView 
{
public:
    RobotArm(int x, int y, int w, int h, char *label) 
        : ModelerView(x,y,w,h,label) {}
    virtual void draw();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createRobotArm(int x, int y, int w, int h, char *label)
{ 
    return new RobotArm(x,y,w,h,label); 
}

// We'll be getting the instance of the application a lot; 
// might as well have it as a macro.
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))


// Utility function.  Use glGetMatrix(GL_MODELVIEW_MATRIX) to retrieve
//  the current ModelView matrix.
Mat4f glGetMatrix(GLenum pname)
{
    GLfloat m[16];
    glGetFloatv(pname, m);
    Mat4f matCam(m[0],  m[1],  m[2],  m[3],
                            m[4],  m[5],  m[6],  m[7],
                            m[8],  m[9],  m[10], m[11],
                            m[12], m[13], m[14], m[15] );

    // because the matrix GL returns is column major...
    return matCam.transpose();
}





// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out RobotArm
void RobotArm::draw()
{
	/* pick up the slider values */

	float t1Theta = VAL( T1_ROTATION );
	float t1Scale = VAL( T1_SCALE );
	float t1X = VAL(T1_X);
	float t1Z = VAL(T1_Z);
	float t1TopRotation = VAL( T1_TOP_ROTATION );
	float t1TopGunLength = VAL( T1_TOPGUN_LENGTH );
	float t1TopGunRotation = VAL( T1_TOPGUN_ROT );
	float t1TurretZRot = VAL ( T1_TURRET_ZROT );
	float t1TurretLength = VAL ( T1_TURRET_LENGTH );

	float t2Theta = VAL( T2_ROTATION );
	float t2Scale = VAL( T2_SCALE );
	float t2X = VAL(T2_X);
	float t2Z = VAL(T2_Z);
	float t2TopRotation = VAL( T2_TOP_ROTATION );
	float t2TopGunLength = VAL( T2_TOPGUN_LENGTH );
	float t2TopGunRotation = VAL( T2_TOPGUN_ROT );
	float t2TurretZRot = VAL ( T2_TURRET_ZROT );
	float t2TurretLength = VAL ( T2_TURRET_LENGTH );

    // This call takes care of a lot of the nasty projection 
    // matrix stuff
    ModelerView::draw();

    // Save the camera transform that was applied by
    // ModelerView::draw() above.
    // While we're at it, save an inverted copy of this matrix.  We'll
    // need it later.
    Mat4f matCam = glGetMatrix( GL_MODELVIEW_MATRIX );
    matCamInverse = matCam.inverse();



	static GLfloat lmodel_ambient[] = {0.4,0.4,0.4,1.0};

	// define the model

	ground(-0.2);
	
	
	glPushMatrix(); // Tank 1
		glScalef(t1Scale, t1Scale, t1Scale); // Scale the whole tank
		glTranslatef(t1X, 0.2, t1Z); //Translate the whole tank
		glRotatef(t1Theta, 0.0, 1.0, 0.0); // Rotate the whole tank
		glPushMatrix();
			tankChasis(2.5, 1.5, 1);
			glPushMatrix(); // Drawing front of tank
				glTranslatef(-(2.5)/2, (1.5)/3, -(1)/1.33);
				drawBumper(2.5, 1, 1.5);
			glPopMatrix();
			glPushMatrix(); // Drawing back of tank
				glTranslatef((2.5)/2, (1.5)/3, -(1)/1.33);
				drawBumper(2.5, 1, 1.5);
			glPopMatrix();
			glPushMatrix(); // Draw Right wheels
				drawWheels(2.5, 1, 1.5);
			glPopMatrix();
		glPopMatrix();

		glTranslatef(0.2, 1, 0.0); // Move to the top of the Chasis
		glRotatef(t1TopRotation, 0.0, 1.0, 0.0); // Rotation for top of tank
		glPushMatrix();
			drawTurretBase(2.5, 1, 1.5); // Draw the turret base cylinder
			glPushMatrix(); // Draw opening of the tank
				glTranslatef(0.0, 0.3, 0.0); // Move to top of turret base
				drawDoor(2.5, 1, 1.5);
				drawTopGun(t1TopGunLength, t1TopGunRotation, 0);
			glPopMatrix();
			glPushMatrix();
				glTranslatef(-0.6, 0.15, 0.0);
				glRotatef(-90.0, 0.0, 1.0, 0.0);
				drawTurret(t1TurretZRot, t1TurretLength, 0);
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();

	glPushMatrix(); // Tank 2
		glScalef(t2Scale, t2Scale, t2Scale); // Scale the whole tank
		glTranslatef(t2X, 0.2, t2Z); //Translate the whole tank
		glRotatef(t2Theta, 0.0, 1.0, 0.0); // Rotate the whole tank
		glPushMatrix();
			tankChasis(2.5, 1.5, 1);
			glPushMatrix(); // Drawing front of tank
				glTranslatef(-(2.5)/2, (1.5)/3, -(1)/1.33);
				drawBumper(2.5, 1, 1.5);
			glPopMatrix();
			glPushMatrix(); // Drawing back of tank
				glTranslatef((2.5)/2, (1.5)/3, -(1)/1.33);
				drawBumper(2.5, 1, 1.5);
			glPopMatrix();
			glPushMatrix(); // Draw Right wheels
				drawWheels(2.5, 1, 1.5);
			glPopMatrix();
		glPopMatrix();

		glTranslatef(0.2, 1, 0.0); // Move to the top of the Chasis
		glRotatef(t2TopRotation, 0.0, 1.0, 0.0); // Rotation for top of tank
		glPushMatrix();
			drawTurretBase(2.5, 1, 1.5); // Draw the turret base cylinder
			glPushMatrix(); // Draw opening of the tank
				glTranslatef(0.0, 0.3, 0.0); // Move to top of turret base
				drawDoor(2.5, 1, 1.5);
				drawTopGun(t2TopGunLength, t2TopGunRotation, 1);
			glPopMatrix();
			glPushMatrix();
				glTranslatef(-0.6, 0.15, 0.0);
				glRotatef(-90.0, 0.0, 1.0, 0.0);
				drawTurret(t2TurretZRot, t2TurretLength, 1);
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();

	//*** DON'T FORGET TO PUT THIS IN YOUR OWN CODE **/
	endDraw();
}

void drawTurret(float turretZRot, float turretLength, int tankNo)
{
	int emitNo = 2;
	if (tankNo == 0)
	{
		emitNo = 0;
	}
	setDiffuseColor(0, 0, 1);
	setDiffuseColor(0, 0, 1);
	glPushMatrix();
		glRotatef(turretZRot, 1.0, 0.0, 0.0);
		Mat4f mvMatrix = glGetMatrix(GL_MODELVIEW_MATRIX);
		Vec4f startPos = matCamInverse * mvMatrix * Vec4f(0.0, 0.0, 0.0, 1.0);
		drawCylinder(turretLength*0.8, 0.14, 0.14);
		glPushMatrix();
			setDiffuseColor(0, 0, 0.75);
			setDiffuseColor(0, 0, 0.75);
			glTranslatef(0, 0, turretLength*0.8);
			drawCylinder(turretLength, 0.08, 0.08);
			glPushMatrix();
				glTranslatef(0, 0, turretLength);
				mvMatrix = glGetMatrix(GL_MODELVIEW_MATRIX);
				Vec4f posT = matCamInverse * mvMatrix * Vec4f(0.0, 0.0, 0.15, 1.0);
				if (ps != NULL)
				{
					Vec3f pos = Vec3f(posT[0], posT[1], posT[2]);
					ps->setEmitterPosition(pos, emitNo);
					Vec3f velDir = (pos - startPos);
					velDir.normalize();
					ps->setEmitterDirection(velDir, emitNo);
				}
				drawCylinder(0.1, 0.12, 0.12);
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
}

void drawTopGun(float gunLength, float gunRotation, int tankNo)
{
	int emitNo = 3;
	if (tankNo == 0)
	{
		emitNo = 1;
	}
	setDiffuseColor(1,1,0);
	setAmbientColor(1,1,0);
	glPushMatrix();
		glTranslatef(-0.2, 0.0, 0.35); // Move to draw the base of the top gun
		glRotatef(gunRotation, 0.0, 1.0, 0.0);
		Mat4f mvMatrix = glGetMatrix(GL_MODELVIEW_MATRIX);
		Vec4f startPos = matCamInverse * mvMatrix * Vec4f(0.0, 0.0, 0.0, 1.0);
		drawCube(0.30, 0.30, 0.1); // Draw the base of the top gun
		glPushMatrix();
			setDiffuseColor(0,1,1);
			setAmbientColor(0,1,1);
			glTranslatef(0.0, 0.05, 0.0);
			glRotatef(-90.0, 0.0, 1.0, 0.0);
			mvMatrix = glGetMatrix(GL_MODELVIEW_MATRIX);
			Vec4f posT = matCamInverse * mvMatrix * Vec4f(0.0, 0.0, gunLength + 0.05, 1.0);
			if (ps != NULL)
			{
				Vec3f pos = Vec3f(posT[0], posT[1], posT[2]);
				ps->setEmitterPosition(pos, emitNo);
				Vec3f velDir = (pos - startPos);
				velDir.normalize();
				ps->setEmitterDirection(velDir, emitNo);
			}
			drawCylinder(gunLength, 0.05, 0.05);
		glPopMatrix();
	glPopMatrix();
}

void drawDoor(float len, float wid, float hei)
{
	setDiffuseColor(0,0,1);
	setAmbientColor(0,0,1);
	glPushMatrix();
		glTranslatef(0.22, -0.08, -0.22);
		drawSphere(0.2);
	glPopMatrix();
}

void drawTurretBase(float len, float wid, float hei)
{
	setDiffuseColor(0, 1, 0);
	setAmbientColor(0, 1, 0);
	glPushMatrix();
		glRotatef(-90.0, 1.0, 0.0, 0.0);
		drawCylinder(0.3, 0.65, 0.65);
	glPopMatrix();
}

void drawWheels(float cLen, float cWid, float cHei)
{
	double radius  = cLen/(5 * 2.5);
	// Right wheels
	glPushMatrix(); // Rear wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(cLen/2.5, 0.0, -cWid/1.15);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, -0.1);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	glPushMatrix(); // Rear - 1 wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(cLen/5, 0.0, -cWid/1.15);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, -0.1);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	glPushMatrix(); // Center wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(0.0, 0.0, -cWid/1.15);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, -0.1);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	glPushMatrix(); // Front - 1 wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(-cLen/5, 0.0, -cWid/1.15);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, -0.1);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	glPushMatrix(); // Front wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(-cLen/2.5, 0.0, -cWid/1.15);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, -0.1);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	// Left wheels
	glPushMatrix(); // Rear wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(cLen/2.5, 0.0, cWid/1.55);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, 0.25);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	glPushMatrix(); // Rear - 1 wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(cLen/5, 0.0, cWid/1.55);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, 0.25);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	glPushMatrix(); // Center wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(0.0, 0.0, cWid/1.55);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, 0.25);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	glPushMatrix(); // Front - 1 wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(-cLen/5, 0.0, cWid/1.55);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, 0.25);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
	glPushMatrix(); // Front wheel
		setDiffuseColor( 1, 1, 1);
		setAmbientColor( 1, 1, 1);
		glTranslatef(-cLen/2.5, 0.0, cWid/1.55);
		drawCylinder(cWid/4, radius, radius);
		glPushMatrix();
			setDiffuseColor( 1, 1, 0);
			setAmbientColor( 1, 1, 0);
			glTranslatef(0, 0, 0.25);
			drawCylinder(cWid/9, radius/2.5, radius/2.5);
		glPopMatrix();
	glPopMatrix();
}

void drawBumper(float len, float wid, float hei)
{
	setDiffuseColor(1, 0, 0);
	setAmbientColor(1, 0, 0);
	glPushMatrix();
		drawCylinder(wid*1.5, hei/3, hei/3);
	glPopMatrix();
}

void tankChasis(float len, float wid, float hei)
{
	setDiffuseColor( 1, 0, 0 );
	setAmbientColor( 1, 0, 0 );
	glPushMatrix();
		drawCube(len, wid ,hei);
	glPopMatrix();
}

void ground(float h) 
{
	glDisable(GL_LIGHTING);
	glColor3f(0.65,0.45,0.2);
	glPushMatrix();
		glScalef(30,0,30);
		drawCube(0.5, 0.5, h);
	glPopMatrix();
	glEnable(GL_LIGHTING);
}

void drawCube(float len, float wid, float hei)
{
	glBegin( GL_QUADS );

	glNormal3d( 1.0 ,0.0, 0.0);			// +x side
	glVertex3d( len/2,0.0, wid/2);
	glVertex3d( len/2,0.0,-wid/2);
	glVertex3d( len/2,hei,-wid/2);
	glVertex3d( len/2,hei, wid/2);

	glNormal3d( 0.0 ,0.0, -1.0);		// -z side
	glVertex3d( len/2,0.0,-wid/2);
	glVertex3d(-len/2,0.0,-wid/2);
	glVertex3d(-len/2,hei,-wid/2);
	glVertex3d( len/2,hei,-wid/2);

	glNormal3d(-1.0, 0.0, 0.0);			// -x side
	glVertex3d(-len/2,0.0,-wid/2);
	glVertex3d(-len/2,0.0, wid/2);
	glVertex3d(-len/2,hei, wid/2);
	glVertex3d(-len/2,hei,-wid/2);

	glNormal3d( 0.0, 0.0, 1.0);			// +z side
	glVertex3d(-len/2,0.0, wid/2);
	glVertex3d( len/2,0.0, wid/2);
	glVertex3d( len/2,hei, wid/2);
	glVertex3d(-len/2,hei, wid/2);

	glNormal3d( 0.0, 1.0, 0.0);			// top (+y)
	glVertex3d( len/2,hei, wid/2);
	glVertex3d( len/2,hei,-wid/2);
	glVertex3d(-len/2,hei,-wid/2);
	glVertex3d(-len/2,hei, wid/2);

	glNormal3d( 0.0,-1.0, 0.0);			// bottom (-y)
	glVertex3d( len/2,0.0, wid/2);
	glVertex3d(-len/2,0.0, wid/2);
	glVertex3d(-len/2,0.0,-wid/2);
	glVertex3d( len/2,0.0,-wid/2);

	glEnd();
}

int main()
{
    // ModelerControl controls[NUMCONTROLS ];
    ModelerControl controls[NUM_CONTROLS];

	controls[T1_ROTATION] = ModelerControl("Tank1 rotation (t1Theta)", -180.0, 180.0, 0.1, 0.0 );
	controls[T1_SCALE] = ModelerControl("Tank1 scale (t1Scale)", 0.1, 5.0, 0.1, 0.8 );
	controls[T1_X] = ModelerControl("Tank1 X (t1X)", -5.6, 5.6, 0.1, 3.0 );
	controls[T1_Z] = ModelerControl("Tank1 Z (t1Z)", -5.6, 5.6, 0.1, 0.0 );
	controls[T1_TOP_ROTATION] = ModelerControl("Tank1 Top rotation (t1TopRotation)", -180.0, 180.0, 0.1, 0.0 );
	controls[T1_TOPGUN_LENGTH] = ModelerControl("Tank1 Top Gun Length (t1TopGunLength)", 0.5, 2.0, 0.1, 0.75 );
	controls[T1_TOPGUN_ROT] = ModelerControl("Tank1 Top Gun rotation (t1TopGunRotation)", -90.0, 90.0, 0.1, 0.0 );
	controls[T1_TURRET_ZROT] = ModelerControl("Tank1 Turret ZRot (t1TurretZRot)", -20.0, 2.0, 0.1, 0.0 );
	controls[T1_TURRET_LENGTH] = ModelerControl("Tank1 Turret Length (t1TurretLength)", 0.5, 2.0, 0.1, 1 );

	controls[T2_ROTATION] = ModelerControl("Tank2 rotation (t2Theta)", -180.0, 180.0, 0.1, 180.0 );
	controls[T2_SCALE] = ModelerControl("Tank2 scale (t2Scale)", 0.1, 5.0, 0.1, 0.8 );
	controls[T2_X] = ModelerControl("Tank2 X (t2X)", -5.6, 5.6, 0.1, -3.0 );
	controls[T2_Z] = ModelerControl("Tank2 Z (t2Z)", -5.6, 5.6, 0.1, 0.0 );
	controls[T2_TOP_ROTATION] = ModelerControl("Tank2 Top rotation (t2TopRotation)", -180.0, 180.0, 0.1, 0.0 );
	controls[T2_TOPGUN_LENGTH] = ModelerControl("Tank2 Top Gun Length (t2TopGunLength)", 0.5, 2.0, 0.1, 0.75 );
	controls[T2_TOPGUN_ROT] = ModelerControl("Tank2 Top Gun rotation (t2TopGunRotation)", -90.0, 90.0, 0.1, 0.0 );
	controls[T2_TURRET_ZROT] = ModelerControl("Tank2 Turret ZRot (t2TurretZRot)", -20.0, 2.0, 0.1, 0.0 );
	controls[T2_TURRET_LENGTH] = ModelerControl("Tank2 Turret Length (t2TurretLength)", 0.5, 2.0, 0.1, 1 );
    

	// You should create a ParticleSystem object ps here and then
	// call ModelerApplication::Instance()->SetParticleSystem(ps)
	// to hook it up to the animator interface.
	// ParticleSystem* ps = new ParticleSystem();
	ps = new ParticleSystem();
	ModelerApplication::Instance()->SetParticleSystem(ps);

    ModelerApplication::Instance()->Init(&createRobotArm, controls, NUM_CONTROLS);
    return ModelerApplication::Instance()->Run();
}

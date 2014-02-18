#ifndef __FLOCK__
#define __FLOCK__
/*
   Irena Rindos 	CPSC 617	Assignment 3    

   Flock class- controls the number of boids and tells them to adjust themselves based on other boids
*/


#include <cstdlib>
#include <iostream>
#include <cmath>
#include "Matrix.h"
#include "boid.h"

#define NUMBOIDS 100

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

const float PGENCOLOR[]={.8, .35, 0, 1};
const float BL[] = {0, 0, 0, 1};


class Flock {
public:
   Flock();

   void init(Vector3d c);
   void draw(double Time);
   void reset();

   void F(Vector3d x[], double t, Vector3d fill[]);
   void EulerNumInt(Vector3d x[], Vector3d A[], double deltaT, Vector3d fill[]);
   void K2NumInt(Vector3d x[], Vector3d A[], double deltaT, Vector3d fill[]);
   Vector3d getCenter();

   void simulate(double airViscosity,  double &TimeStep, double &Time, double CoeffofRestitution, 
		int &NTimeSteps, int &TimeStepsPerDisplay);

private:
   Vector3d coords;
   Vector3d stateArray[NUMBOIDS*2];
   Boid boidArray[NUMBOIDS];
};
#endif

#ifndef __BOID__
#define __BOID__
/*
   Irena Rindos 	CPSC 617	Assignment 3   

   Boid class- Boid will check "rules" to adjust its motion in respect to other boids
*/


#include <cstdlib>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "Matrix.h"

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#define MASSMEDIAN 20
#define EPS		0.1

//Mess with these values to adjust the weightng
#define KA 	30//60
#define KC	6//5
#define KV    .25//.35//	.025

#define OUTERFOV  15.0
#define INNERFOV  10.0

struct limitSphere{
   Vector3d center;
   double radius;
};

const float WHITE[] = {1, 1, 1, 1};

class Boid {
public:
   Boid();

   double gauss(double mean, double std, int seed);

   void draw(double Time);

   void init(Vector3d c);

   Vector3d getCoords(){return coords;};
   Vector3d getVel(){return velocity;}
   void setCoords(Vector3d c){coords = c;}
   void setOldCoords(Vector3d c){oldCoords = c;}
   void setVelocity(Vector3d c){velocity = c;}

   Vector3d accel(Boid bArray[], double t, int self, int numBoids);

private:
   Vector3d coords, oldCoords;
   Vector3d velocity;
   float PCOLOR[4];
   int mass;
   static int limitSphereRadius;
   static Vector3d limitSphereCenter;
};
#endif

#include "flock.h"


Flock::Flock(){
   coords[0]= 0;
   coords[1]= 0;
   coords[2]= 0;
}

void Flock::init(Vector3d c){
  coords = c;
  reset();
}

void Flock::draw(double Time){
   //Call draw for the particles
   int i;
   for(i=0; i< NUMBOIDS; i++){
	boidArray[i].draw(Time);
   }
}

void Flock::reset(){
//Reset to new start positions, based on flock start pos
  int i;
  for(i=0; i< NUMBOIDS; i++){
	boidArray[i].init(coords);
	stateArray[i] = boidArray[i].getCoords();
	stateArray[NUMBOIDS +i] = boidArray[i].getVel();
   }
}
   
void Flock::F(Vector3d x[], double t, Vector3d fill[]){
  //Copy velocities to new locations
  int i;
  for(i=0; i< NUMBOIDS; i++){
	fill[i] = stateArray[i+NUMBOIDS];//boidArray[i].getVel();
   }
//Have the boids calc their acceleration based on other boids
  for(i=NUMBOIDS; i< (NUMBOIDS*2); i++){
     fill[i] = boidArray[i-NUMBOIDS].accel(boidArray, t, (i-NUMBOIDS), NUMBOIDS);
  }
}

void Flock::EulerNumInt(Vector3d x[], Vector3d A[], double deltaT, Vector3d fill[]){
  F(x, deltaT, A);
//Below is the Euler method; use a better numerical integration method
   int i;
   for(i=0; i < (NUMBOIDS*2); i++){
      fill[i] = x[i] + (deltaT * A[i]);
   }
}

void Flock::K2NumInt(Vector3d x[], Vector3d A[], double deltaT, Vector3d fill[]){
   Vector3d K[NUMBOIDS*2];
   F(x, deltaT, K); //K is now a full Euler step
   //above is equivalent to deltaT*f(x)
  
   //Next step in K2 is x = x+deltaT *f(x+.5k) 
   int i;
   for(i=0; i < (NUMBOIDS*2); i++){
      K[i]= x[i]+ (0.5*K[i]);
   }
   F(K, deltaT, A);
   for(i=0; i < (NUMBOIDS*2); i++){
      fill[i] = x[i] + (deltaT * A[i]);
   }
}

//Call on active particles to simulate their movement
void Flock::simulate(double airViscosity, double &TimeStep, double &Time, 
	double CoeffofRestitution, int &NTimeSteps, int &TimeStepsPerDisplay){

   Vector3d xNew[NUMBOIDS*2];
   Vector3d A[NUMBOIDS*2];

//do display, etc while the output state is fixed
//possibly extra stuff here
   if(NTimeSteps % TimeStepsPerDisplay == 0){
     draw(Time);
     glutPostRedisplay();
   }

//   EulerNumInt(stateArray, A, TimeStep, xNew);
   K2NumInt(stateArray, A, TimeStep, xNew);

   int i;
   for(i=0; i<(NUMBOIDS*2); i++){
      if(i < NUMBOIDS){
	boidArray[i].setCoords(xNew[i]); //Update boid coords
        boidArray[i].setOldCoords(stateArray[i]); //Update boid coords
      }
      else boidArray[i-NUMBOIDS].setVelocity(xNew[i]); //Update velocity 

      stateArray[i]= xNew[i];//Update state array      
    }

 // advance the timestep 
  Time += TimeStep; 
  NTimeSteps++;
}

Vector3d Flock::getCenter(){
   Vector3d center(0,0,0);

   int i;
   for(int i=0; i<NUMBOIDS; i++) center = center+ boidArray[i].getCoords();

   center = center *(1.0/(double)NUMBOIDS);

   return center;
}

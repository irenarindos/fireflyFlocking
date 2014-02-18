#include "boid.h"

Boid::Boid(){
   coords[0]= 0;
   coords[1]= 0;
   coords[2]= 0;

   oldCoords[0]= 0;
   oldCoords[1]= 0;
   oldCoords[2]= 0;

   PCOLOR[0] = .4;
//some of these will be values outside of hte 0-1 range...
   PCOLOR[1] = .1+ drand48();
   PCOLOR[2] = .1 + drand48();
   PCOLOR[3] = 1; 
}

int Boid::limitSphereRadius=20;
Vector3d center(0,0,0);
Vector3d Boid::limitSphereCenter= center;


double Boid::gauss(double mean, double std, int seed)
{
  const int itblmax = 20;	// length - 1 of table describing F inverse
  const double didu = 40.0;	// delta t)*kdijable position/delta ind. variable

  // interpolation table for F inverse
  static double tbl[] =
    {0.00000E+00, 6.27500E-02, 1.25641E-01, 1.89000E-01,
       2.53333E-01, 3.18684E-01, 3.85405E-01, 4.53889E-01,
       5.24412E-01, 5.97647E-01, 6.74375E-01, 7.55333E-01,
       8.41482E-01, 9.34615E-01, 1.03652E+00, 1.15048E+00,
       1.28167E+00, 1.43933E+00, 1.64500E+00, 1.96000E+00,
       3.87000E+00};
  
  static int first_time = 1;

  double u;
  double di;
  int index, minus;
  double delta, gaussian_random_value;

  if (first_time){
#ifdef WIN32
    srand((unsigned)seed);
#else
    srand48(seed);
#endif
    first_time = 0;
  }

  //
  // compute uniform random number between 0.0 - 0.5, and a sign with 
  // probability 1/2 of being either + or -
  //
#ifdef WIN32
  int rn = rand();
  u = double(rn) / double(RAND_MAX);
#else
  u = drand48();
#endif
  if (u >= 0.5){
    minus = 0;
    u = u - 0.5;
  }
  else
    minus = 1;

  //
  // interpolate gaussian random number using table
  //
  index = (int)(di = (didu * u));
  if(index == itblmax)
    delta = tbl[index];
  else{
    di -= index;
    delta =  tbl[index] + (tbl[index + 1] - tbl[index]) * di;
  }
  gaussian_random_value = mean + std * (minus? -delta : delta);

  return gaussian_random_value;
}

//Also employ motionblur
void Boid::draw(double Time){

   double opac = sin(Time)/4.0 + gauss(.1, 0.15, 0.1);
   if(opac < 0) opac = opac * -1;
   if(opac > .4) opac = .4;


   glColor4f(PCOLOR[0],PCOLOR[1],PCOLOR[2],1.0f); 
   glPointSize(2.0);
   glBegin(GL_POINTS);
      glVertex3f(coords[0], coords[1], coords[2]);
   glEnd();
   glColor4f(PCOLOR[0],PCOLOR[1],PCOLOR[2], opac);
   glPointSize(8.0);
   glBegin(GL_POINTS);
      glVertex3f(coords[0], coords[1], coords[2]);
   glEnd(); 

}

void Boid::init(Vector3d c){   
//Set initial coords to offset from flock starting position
//Mess with these gauss values
   coords[0]= c[0]+ gauss(2.0, 5.0, 2.5);
   coords[1]= c[1]+ gauss(2.0, 5.0, 2.5);
   coords[2]= c[2]+ gauss(2.0, 5.0, 2.5);

   double phi, theta; //will be in radians...

//Generated between0 and 1; want phi between -pi/2 and pi/2
//Want theta between -pi and pi
//Might want to change this for the flock...

   phi = (drand48() * PI) -(PI/2.0);
   theta = (drand48() * 2*PI) -PI;

   //Using these angles will give a unit direction vector
   velocity[0] = cos(phi)*cos(theta);
   velocity[1] = sin(phi);
   velocity[2] = -cos(phi)*sin(theta);

   //Generate speed using gaussian distribution
   double speed = gauss(0.35, 0.25, .45);//have no ideas what these values will dooooo, should be globals eventually

   //Then multiply it times the velocity
   velocity = speed*velocity;

   mass = MASSMEDIAN + (drand48()*10);
}

Vector3d Boid::accel(Boid bArray[], double t, int self, int numBoids){//Self index to prevent self comparison
  Vector3d acceleration;  

  //Init to 0
  Vector3d aA(0,0,0);//Avoidance
  Vector3d aC(0,0,0);//Centering
  Vector3d aV(0,0,0);//Velocity matching

//Use this to calculate the forces exerted on the boid by other boids
  int i;
  for(i=0; i<= numBoids; i++){
     if(i == self) continue;
    
     Vector3d xij = bArray[i].getCoords()- coords; //Vec between
     double dij = xij.norm(); //Distance betweent the two  

     Vector3d kdij(1.0, 1.0, 1.0);
     if(dij > OUTERFOV) continue;//outside of Field of View
     //Between inner and outer FOV, linear drop off
     if(dij > INNERFOV) kdij = kdij * ((dij- INNERFOV)/OUTERFOV);

     Vector3d uij = xij.normalize(); //Unit vec
    
//kdij is a scalar constant that drops off depending on how far away a bird is from the
//"field of view" of this bird

      aA = (aA +( uij/dij)); //*kdij;   
      aC = (aC +(dij * uij)); //*kdij; 
      aV = (aV+ ( bArray[i].getVel() - velocity)); //*kdij;
  }

//Then multiply by the scalar constants for that force over the mass of the boid
  aA = (-KA/(double)mass) * aA;
  aC = (KC/(double)mass) * aC; 
  aV = (KV/(double)mass) * aV; 

  //Sum acceleration forces; 
  acceleration = aA + aC + aV;

//Make sure the boid avoids the sphere limits
  Vector3d toSphere = limitSphereCenter- coords; //From boid to center
  double dToSphere = toSphere.norm(); //Distance to center


  //Then check to see if it's inside the sphere; use that as a weight?
  double weight = dToSphere/(double)limitSphereRadius ;
  toSphere = toSphere*weight;
  

  toSphere = toSphere*(1/(double)mass);

  Vector3d cool = toSphere*gauss(0.75, 0.25, 2.75);

  acceleration = acceleration +cool;//toSphere;



  //Make their movement a bit more interesting
  Vector3d excitement(1.0, 1.0, 1.0);
  return acceleration+ (excitement *gauss(0.75, 0.25, 2.75));
}


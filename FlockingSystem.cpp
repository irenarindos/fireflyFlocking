/*
   Irena Rindos 	CPSC 617	Assignment 3    

   Flocking System 

   Handles the camera, etc, and calls the flock class, which holds an array of boids (gnats)

   Uses Dr. House's camera class as a base; also utilizes Dr. House's gauss function.

*/

#include "Camera.h"
#include "flock.h"

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif


int FrameNumber = 0;

static double airViscosity =10;
static double TimeStep =.1;
static double DispTime = 1;
static int TimeStepsPerDisplay = 1;
static double CoeffofRestitution =0.75;

static int NSteps = 0;
static int NTimeSteps = -1;
static double Time = 0;

//const float WHITE[] = {1, 1, 1, 1};
const float BLACK[] = {0, 0, 0, 1};
const float GREEN[]={0.05, .15, 0.05, 1};
const float BLUE[]={0, 0, 1, 1};

int WIDTH = 900;
int HEIGHT = 750;

static int TimerDelay= 30;

int persp_win;

Camera *camera;
Flock flock;

void TimerCallback(int value);

void init() {
  // set up camera
  // parameters are eye point, aim point, up vector
  camera = new Camera(Vector3d(15, 50, 100), Vector3d(0, 0, 0), 
		      Vector3d(0, 1, 0));

  glClearColor(0.05, 0.05, .2, 1.0);
  glDepthRange(0.0, 1.0);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glShadeModel(GL_SMOOTH);

  glEnable (GL_BLEND); 
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  Vector3d location(0.0,0.0,-10.0);
  flock.init(location);
}

void drawGround(){
   glColor4f(0.05, .15, 0.05, 1); 
   glBegin(GL_POLYGON);
      glVertex3f(100.0, -55.0, 100.0);
      glVertex3f(100.0, -55.0, -100.0);
      glVertex3f(-100.0, -55.0, -100.0);
      glVertex3f(-100.0, -55.0, 100.0);
   glEnd();

}


void PerspDisplay() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
  // draw the camera created in perspective
  camera->PerspectiveDisplay(WIDTH, HEIGHT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

//Draw the flock
  flock.draw(Time);
 // drawGround();
  glutSwapBuffers();
}

void mouseEventHandler(int button, int state, int x, int y) {
  // let the camera handle some specific mouse events (similar to maya)
  camera->HandleMouseEvent(button, state, x, y);
}

void motionEventHandler(int x, int y) {
  // let the camera handle some mouse motions if the camera is to be moved
  camera->HandleMouseMotion(x, y);
  glutPostRedisplay();
}

void keyboardEventHandler(unsigned char key, int x, int y) {
  switch (key) {
  case 'r': case 'R':
    // reset the camera to its initial position
    camera->Reset();
    break;
  case 'f': case 'F':
    camera->SetCenterOfFocus(Vector3d(0, 0, 0));
    break;
  case 'q': case 'Q':	// q or esc - quit
  case 27:		// esc
    exit(0);
  }

  glutPostRedisplay();
}

void Simulate(){
   //make the flock simulate...
   flock.simulate(airViscosity, TimeStep, Time, CoeffofRestitution, NTimeSteps, TimeStepsPerDisplay);


   glutIdleFunc(NULL);
   glutTimerFunc(TimerDelay, TimerCallback, 0);
}

/*  Run a single time step in the simulation*/
void TimerCallback(int){
  Simulate();
}

int main(int argc, char *argv[]) {

  // set up opengl window
  glutInit(&argc, argv);
  // create the graphics window, giving width, height, and title text
  // and establish double buffering, RGBA color, depth testing
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

  glutInitWindowSize(WIDTH, HEIGHT);
  glutInitWindowPosition(50, 50);
  persp_win = glutCreateWindow("Flocking System: Fireflies");

  // initialize the camera and such
  init();

  // set up opengl callback functions
  glutDisplayFunc(PerspDisplay);
  glutMouseFunc(mouseEventHandler);
  glutMotionFunc(motionEventHandler);
  glutKeyboardFunc(keyboardEventHandler);
  glutIdleFunc(Simulate);

  glutMainLoop();
  return(0);
}


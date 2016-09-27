#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <Windows.h>
#include <gl/GL.h>
#include <glut.h>

#include "glm.h"
#include "mtxlib.h"
#include "trackball.h"

using namespace std;

_GLMmodel *mesh;
int WindWidth, WindHeight;

int last_x , last_y;
int selectedFeature = -1;
vector<int> featureList;
_GLMmodel *originMesh;
vector<float> w;

/*implement*/
#define STDDIFF 0.3
double radiusBasis(vector3 cPt, vector3 Pt)
{
	float r = (Pt - cPt).length();
	return exp(-r*r / (2 * STDDIFF*STDDIFF));
}

/*calculate weight of control point */
void weightCalculate()
{
	
}

void localDeformation()
{
	vector3 fPt(mesh->vertices[3 * selectedFeature + 0], mesh->vertices[3 * selectedFeature + 1], mesh->vertices[3 * selectedFeature + 2]);
	vector3 orif(originMesh->vertices[3 * selectedFeature + 0], originMesh->vertices[3 * selectedFeature + 1], originMesh->vertices[3 * selectedFeature + 2]);
	vector3 vecf(fPt.x - orif.x, fPt.y - orif.y, fPt.z - orif.z);
	vector3 d(0.0, 0.0, 0.0);
	for (int i = 0; i < mesh->numvertices; i++)
	{
		if (i == selectedFeature) continue;
		vector3 point(originMesh->vertices[3 * i + 0], originMesh->vertices[3 * i + 1], originMesh->vertices[3 * i + 2]);
		double sai = radiusBasis(orif, point);
		d = sai*vecf;
		mesh->vertices[3 * i + 0] = originMesh->vertices[3 * i + 0] + d.x;
		mesh->vertices[3 * i + 1] = originMesh->vertices[3 * i + 1] + d.y;
		mesh->vertices[3 * i + 2] = originMesh->vertices[3 * i + 2] + d.z;
	}
}

void Reshape(int width, int height)
{
  int base = min(width , height);

  tbReshape(width, height);
  glViewport(0 , 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0,(GLdouble)width / (GLdouble)height , 1.0, 128.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -3.5);

  WindWidth = width;
  WindHeight = height;
}

void Display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
  tbMatrix();
  
  // render solid model
  glEnable(GL_LIGHTING);
  glColor3f(1.0 , 1.0 , 1.0f);
  glPolygonMode(GL_FRONT_AND_BACK , GL_FILL);
  glmDraw(mesh , GLM_SMOOTH);

  // render wire model
  glPolygonOffset(1.0 , 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glLineWidth(1.0f);
  glColor3f(0.6 , 0.0 , 0.8);
  glPolygonMode(GL_FRONT_AND_BACK , GL_LINE);
  glmDraw(mesh , GLM_SMOOTH);

  // render features
  glPointSize(10.0);
  glColor3f(1.0 , 0.0 , 0.0);
  glDisable(GL_LIGHTING);
  glBegin(GL_POINTS);
	for (int i = 0 ; i < featureList.size() ; i++)
	{
		int idx = featureList[i];

		glVertex3fv((float *)&mesh->vertices[3 * idx]);
	}
  glEnd();
  
  glPopMatrix();

  glFlush();  
  glutSwapBuffers();
}

vector3 Unprojection(vector2 _2Dpos)
{
	float Depth;
	int viewport[4];
	double ModelViewMatrix[16];				//Model_view matrix
	double ProjectionMatrix[16];			//Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	glReadPixels((int)_2Dpos.x , viewport[3] - (int)_2Dpos.y , 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

	double X = _2Dpos.x;
	double Y = _2Dpos.y;
	double wpos[3] = {0.0 , 0.0 , 0.0};

	gluUnProject(X , ((double)viewport[3] - Y) , (double)Depth , ModelViewMatrix , ProjectionMatrix , viewport, &wpos[0] , &wpos[1] , &wpos[2]);

	return vector3(wpos[0] , wpos[1] , wpos[2]);
}

void mouse(int button, int state, int x, int y)
{
  tbMouse(button, state, x, y);

  // add feature
  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
  {
	  int minIdx = 0;
	  float minDis = 9999999.0f;

	  vector3 pos = Unprojection(vector2((float)x , (float)y));

	  /*find the nearest vertex as feature vertex*/
	  for (int i = 0 ; i < mesh->numvertices ; i++)
	  {
		  vector3 pt(mesh->vertices[3 * i + 0] , mesh->vertices[3 * i + 1] , mesh->vertices[3 * i + 2]);
		  float dis = (pos - pt).length();
		  
		  if (minDis > dis)
		  {
			  minDis = dis;
			  minIdx = i;
		  }
	  }
	  
	  /*record new feature*/
	  featureList.push_back(minIdx);
	  w.push_back(1.0);
	  weightCalculate();
  }

  // manipulate feature
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
  {
	  int minIdx = 0;
	  float minDis = 9999999.0f;

	  vector3 pos = Unprojection(vector2((float)x , (float)y));

	  for (int i = 0 ; i < featureList.size() ; i++)
	  {
		  int idx = featureList[i];
		  vector3 pt(mesh->vertices[3 * idx + 0] , mesh->vertices[3 * idx + 1] , mesh->vertices[3 * idx + 2]);
		  float dis = (pos - pt).length();

		  if (minDis > dis)
		  {
			  minDis = dis;
			  minIdx = featureList[i];
		  }
	  }

	  selectedFeature = minIdx;
  }

  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
	  selectedFeature = -1;

  last_x = x;
  last_y = y;
}

void motion(int x, int y)
{
  tbMotion(x, y);

  if (selectedFeature != -1)
  {
	  matrix44 m;
	  vector4 vec = vector4((float)(x - last_x) / 100.0f , (float)(y - last_y) / 100.0f , 0.0 , 1.0);
	  
	  gettbMatrix((float *)&m);
	  vec = m * vec;

	  /*new position of feature vertex*/
	  mesh->vertices[3 * selectedFeature + 0] += vec.x;
	  mesh->vertices[3 * selectedFeature + 1] -= vec.y;
	  mesh->vertices[3 * selectedFeature + 2] += vec.z;
	  
	  /*new position of other points*/
	  localDeformation();
	  
  }

  last_x = x;
  last_y = y;
}

void timf(int value)
{
  glutPostRedisplay();
  glutTimerFunc(1, timf, 0);
}

int main(int argc, char *argv[])
{
  WindWidth = 400;
  WindHeight = 400;
	
  GLfloat light_ambient[] = {0.0, 0.0, 0.0, 1.0};
  GLfloat light_diffuse[] = {0.8, 0.8, 0.8, 1.0};
  GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_position[] = {0.0, 0.0, 1.0, 0.0};

  glutInit(&argc, argv);
  glutInitWindowSize(WindWidth, WindHeight);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow("Trackball Example");

  glutReshapeFunc(Reshape);
  glutDisplayFunc(Display);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glClearColor(0, 0, 0, 0);

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  glEnable(GL_LIGHT0);
  glDepthFunc(GL_LESS);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  tbInit(GLUT_LEFT_BUTTON);
  tbAnimate(GL_TRUE);

  glutTimerFunc(40, timf, 0); // Set up timer for 40ms, about 25 fps

  // load 3D model
  mesh = glmReadOBJ("../data/head.obj");
  originMesh = glmReadOBJ("../data/head.obj"); //implement
  cout << originMesh << " " << mesh << endl;

  glmUnitize(mesh);
  glmFacetNormals(mesh);
  glmVertexNormals(mesh , 90.0);

  glmUnitize(originMesh);
  glmFacetNormals(originMesh);
  glmVertexNormals(originMesh, 90.0);

  glutMainLoop();

  return 0;

}


/*
Author: Nithin Shrivatsav Srikanth 
Description: 
This is a code to carry out the movement of UAV's around a sphere in a football field. There are four forces acting on the uav. 
These are gravitational force pulling it down, modified hooke's law force holding it on a sphere, radial force into the center of the sphere,
tangential force to move it around the sphere. 
*/
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mpi.h"
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <GL/glut.h>
#include <chrono>
#include <thread>
#include <vector>
#include "ECE_Bitmap.h"

// Send location and velocity vector in each direction
const int numElements = 6; // x, y, z, vx, vy, vz

const int rcvSize = 16 * 6; // (Main task + 15 UAVs) * numElements

// mass of the UAV
float mass = 1;

double rcvbuffer[rcvSize];

double sendBuffer[numElements];

// sphere coordinates and target positions for each UAV
float circleX = 0.0; float circleY = 0.0; float circleZ = 50.0;

// parameters to adapt the color
int colorChangeTime = 1;
double colorValue = 255;
double colorValueNormalized = 1.0;
int flag = 0;

// acceleration due to gravity
float g = -10;

// texture object
GLuint texture[1];
BMP inBitmap;

//----------------------------------------------------------------------
// Reshape callback
//
// Window size has been set/changed to w by h pixels. Set the camera
// perspective to 45 degree vertical field of view, a window aspect
// ratio of w/h, a near clipping plane at depth 1, and a far clipping
// plane at depth 100. The viewport is the entire window.
//
//----------------------------------------------------------------------
void changeSize(int w, int h)
{
    float ratio = ((float)w) / ((float)h); // window aspect ratio
    glMatrixMode(GL_PROJECTION); // projection matrix is active
    glLoadIdentity(); // reset the projection
    gluPerspective(60.0, ratio, 0.1, 600.0); // perspective transformation
    glMatrixMode(GL_MODELVIEW); // return to modelview mode
    glViewport(0, 0, w, h); // set viewport (drawing area) to entire window
}

//----------------------------------------------------------------------
// Initialize World
//
// Function to position the UAV's at the initial coordinates. They are sepeated by 25 yards horizontally and cover the entire field.
// These values are used to initialize the rcvbuffer of the processing to start the UAV's at these initial locations.
//
//----------------------------------------------------------------------
void initializeWorld(void)
{
    rcvbuffer[0] = 0; rcvbuffer[1] = 0; rcvbuffer[2] = 0; rcvbuffer[3] = 0; rcvbuffer[4] = 0; rcvbuffer[5] = 0;
    int agent = 1;
    for (int i = -2; i < 3; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            rcvbuffer[agent*6] = i*21.86;
            rcvbuffer[agent*6+1] = j*21.26;
            rcvbuffer[agent*6+2] = 0.0;
            rcvbuffer[agent*6+3] = 0.0;
            rcvbuffer[agent*6+4] = 0.0;
            rcvbuffer[agent*6+5] = 0.0;
            ++agent;
        }
    }
}

//----------------------------------------------------------------------
// Draw UAV
//
// Draw a Torus for UAV and scale it by 1.0 in all directions. Then loop
// through to construct all the UAVs.
//
//----------------------------------------------------------------------
void drawSingleUAV(void)
{
    glColor3f(0.0, 0.0, colorValueNormalized);
    glPushMatrix();
        // glScalef(1.0, 1.0, 1.0);
        glutSolidTorus(0.25, 0.5, 20, 20);
    glPopMatrix();
}

void drawUAVs(void)
{
   for(int i=1; i<=15; ++i)
    {
        glPushMatrix();
            glTranslatef(rcvbuffer[i*6], rcvbuffer[i*6+1], rcvbuffer[i*6+2]);
            drawSingleUAV();
        glPopMatrix();
    }            
}

//----------------------------------------------------------------------
// Texture
//
// Load the texture into the memory.
// 
//----------------------------------------------------------------------
void myinit(void)
{

    glClearColor(0.5, 0.5, 0.5, 0.0);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    inBitmap.read("AmFBfield.bmp");

    // Create Textures
    glGenTextures(1, texture);
    
    // Setup first texture
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //scale linearly when image smalled than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0,
        GL_BGR_EXT, GL_UNSIGNED_BYTE, &inBitmap.data[0]);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

    glEnable(GL_TEXTURE_2D);
}

//----------------------------------------------------------------------
// Draw the entire scene
//
// We first update the camera location based on its distance from the
// origin and its direction.
//----------------------------------------------------------------------
void renderScene()
{
    int i,j; 

    // Clear color and depth buffers
    glClearColor(1.0, 1.0, 1.0, 1.0); 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Reset transformations
    glLoadIdentity();


    gluLookAt(0, 60, 100, 
              0, 0, 0,
              0.0, 0.0, 1.0); // eye locations ==> -10, -10, 70, the center is the center of the field will be center_x, center_y, center_z+z

    glMatrixMode(GL_MODELVIEW);

    glBindTexture(GL_TEXTURE_2D, texture[0]);
    // glColor3f(0.0, 0.7, 0.0);
    glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex3f(-54.85, -24.4, 0.0);
        glTexCoord2f(0, 1);
        glVertex3f(-54.85, 24.4, 0.0);
        glTexCoord2f(1, 1);
        glVertex3f(54.85, 24.4, 0.0);
        glTexCoord2f(1, 0);
        glVertex3f(54.85, -24.4, 0.0);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);

    glPushMatrix();
        glColor3f(1.0, 0.0, 0.0);
        glTranslatef(0, 0, 50.0);
        glutWireSphere(10, 20, 20);
    glPopMatrix();

    drawUAVs();

    glutSwapBuffers(); 

    // MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
    
    // MPI_Finalize();
    // std::this_thread::sleep_for(std::chrono::millisecond(2000));
    // return 0;
}

//----------------------------------------------------------------------
// Timer function called whenever the timer fires. Timer function to be 
// called every 100ms to change the color value and refresh the scene 
// for every 100ms and gather data from all UAVs.  
//----------------------------------------------------------------------
void timerFunction(int id)
{
    glutPostRedisplay();
    if(flag == 0)
    {
        --colorValue;
    }
    else if(flag == 1)
    {
        ++colorValue;
    }

    if(colorValue == 255)
    {
        flag = 0;
    }
    else if(colorValue == 128)
    {
        flag = 1;
    }
    colorValueNormalized = colorValue/255.0;

    MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
    glutTimerFunc(100, timerFunction, 0);
}

//----------------------------------------------------------------------
// mainOpenGL  - standard GLUT initializations and callbacks
//----------------------------------------------------------------------
void mainOpenGL(int argc, char**argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);

    glutCreateWindow(argv[0]);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    myinit();
    glutReshapeFunc(changeSize);
    glutDisplayFunc(renderScene);
    glutTimerFunc(100, timerFunction, 0);
    glutMainLoop();
}

//----------------------------------------------------------------------
// function to calculate the uav location
//----------------------------------------------------------------------
float calcualteUAVsLocation(float x0, float vx0, float ax)
{
    return (x0 + vx0*0.1 + 0.5*ax*0.1*0.1);
}

//----------------------------------------------------------------------
// function to calculate the uav velocity
//----------------------------------------------------------------------
float calculateVelocity(float vx0, float ax)
{
    return (vx0 + ax*(0.1));
}

//----------------------------------------------------------------------
// function to calculate the force due to modified hooke's law to maintain
// uav on the surface of a sphere
//----------------------------------------------------------------------
float generateForce(float D)
{
    float force = -5*(10.0-D);
    return force;
}

//----------------------------------------------------------------------
// main
//----------------------------------------------------------------------
int main(int argc, char**argv)
{ 
    int numTasks, rank;

    int rc = MPI_Init(&argc, &argv);

    if (rc != MPI_SUCCESS) 
    {
        printf("Error starting MPI program. Terminating.\n");
        MPI_Abort(MPI_COMM_WORLD, rc);
    }

    MPI_Comm_size(MPI_COMM_WORLD, &numTasks);

    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    int gsize = 0;

    MPI_Comm_size(MPI_COMM_WORLD, &gsize);

    initializeWorld();

    // use rank 0 to call opengl functions
    if (rank == 0) 
    {
        mainOpenGL(argc, argv);
    }

    // the rest of the ranks are dedicated to the UAVs
    else
    {
        // Sleep for 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));
 
        // 100msec is 10 frames/sec 
        // this is 60s * 10frames/sec 
        // the main for loop which does all the processing for 60s      
        for (int ii = 0; ii < 600 ; ++ii)
        {
            // gather current data
            float x0 = rcvbuffer[rank*6]; float y0 = rcvbuffer[rank*6+1]; float z0 = rcvbuffer[rank*6+2]; 
            float vx0 = rcvbuffer[rank*6+3]; float vy0 = rcvbuffer[rank*6+4]; float vz0 = rcvbuffer[rank*6+5];

            //elastic collision 
            int innerLoopClose = 1;
            int outerLoopClose = 1;

            // this loop tries to find the collision between pairs of UAVs
            // a round robin vlocity transfer is performed
            while(outerLoopClose)
            {
                // moving in the right direction of round robin velocity replacement
                if(innerLoopClose)
                {
                    // move from the next higher rank uav till end
                    for(int uav=rank+1; uav<=15; ++uav)
                    {
                        float tempDist = sqrt(pow(rcvbuffer[uav*6]-x0,2) + pow(rcvbuffer[uav*6+1]-y0,2) + pow(rcvbuffer[uav*6+2]-z0,2));
                        if(tempDist<=1.01)
                        {
                            x0 = rcvbuffer[uav*6];
                            y0 = rcvbuffer[uav*6+1];
                            z0 = rcvbuffer[uav*6+2];
                            outerLoopClose = 0;
                            break;
                        }
                    }
                    innerLoopClose = 0;
                }
                
                // if no uav of higer rank can be used for velocity replacement, then we need to move to uav's of smaller ranks as it's cylindrical or round robin
                else
                {
                    // move from the lowest ranked uav to current uav
                    for(int uav=1; uav<rank; ++uav)
                    {
                        float tempDist = sqrt(pow(rcvbuffer[uav*6]-x0,2) + pow(rcvbuffer[uav*6+1]-y0,2) + pow(rcvbuffer[uav*6+2]-z0,2));
                        if(tempDist<=1.01)
                        {
                            x0 = rcvbuffer[uav*6];
                            y0 = rcvbuffer[uav*6+1];
                            z0 = rcvbuffer[uav*6+2];
                            outerLoopClose = 0;
                            break;
                        }
                    }
                }
                outerLoopClose = 0;
            }
            
            
            // calculate the direction vector to the center of the sphere
            float fx = circleX-x0;
            float fy = circleY-y0;
            float fz = circleZ-z0;

            // calculate the distance to the center of the sphere
            float D = sqrt(fx*fx + fy*fy + fz*fz);

            // generate the modified hooke's law force
            float Fs = generateForce(D);

            // calculate the unit vectors in the radial direction
            float unitVectorFx = fx/D;
            float unitVectorFy = fy/D;
            float unitVectorFz = fz/D;
            
            // calculate the tangential direction to the sphere
            // the tangential direction is calculated by taking a basis vector that's not perpendiculur to the radial vector
            // and the cross product of these two vectors returns the tangential direction
            float basis1X = 1.0; float basis1Y = 0.0; float basis1Z = 0.0;
            float basis2X = 0.0; float basis2Y = 1.0; float basis2Z = 0.0;
            float basis3X = 0.0; float basis3Y = 0.0; float basis3Z = 1.0;

            float basisX; float basisY; float basisZ;


            if(rank==1 || rank==4 || rank==7 || rank==10 || rank==13)
            {
                basisX = 1.0;
                basisY = 0.0;
                basisZ = 1.0;
            }
            else if(rank==2 || rank==5 || rank==8 || rank==11 || rank==14)
            {
                basisX = 1.0;
                basisY = 1.0;
                basisZ = 0.0;
            }
            else if(rank==3 || rank==6 || rank==9 || rank==12 || rank==15)
            {
                basisX = 0.0;
                basisY = 1.0;
                basisZ = 1.0;
            }

            // cross-product
            float fxt1 = -fy*basis1Z + fz*basis1Y;
            float fyt1 = -fz*basis1X + fx*basis1Z;
            float fzt1 = -fx*basis1Y + fy*basis1X;

            float fxt = -fy*basisZ + fz*basisY;
            float fyt = -fz*basisX + fx*basisZ;
            float fzt = -fx*basisY + fy*basisX;

            float fxt3 = -fy*basis3Z + fz*basis3Y;
            float fyt3 = -fz*basis3X + fx*basis3Z;
            float fzt3 = -fx*basis3Y + fy*basis3X;

            // magnitude of the tangential vector
            float D2 = sqrt(fxt*fxt + fyt*fyt + fzt*fzt);
            float Ft = 5.0;

            // calculate the tangential direction
            float unitVectorFxTangential = fxt/D2;
            float unitVectorFyTangential = fyt/D2;
            float unitVectorFzTangential = fzt/D2;

            // hooke's law force
            float ax = Fs*unitVectorFx; 
            float ay = Fs*unitVectorFy; 
            float az = Fs*unitVectorFz;
 
            // tangential force
            float axt = Ft*unitVectorFxTangential; 
            float ayt = Ft*unitVectorFyTangential; 
            float azt = Ft*unitVectorFzTangential;

            // radial force
            float axx = 40.0*unitVectorFx; 
            float ayy = 40.0*unitVectorFy; 
            float azz = 40.0*unitVectorFz;

            // total force
            float forceX = ax+axt+axx;
            float forceY = ay+ayt+ayy;
            float forceZ = az+azt+azz;
            float totalForceMag = sqrt(pow(forceX,2) + pow(forceY,2) + pow(forceZ,2));
            
            // resultant unit vector
            forceX /= totalForceMag;
            forceY /= totalForceMag;
            forceZ /= totalForceMag;

            // thresholding force 
            if(totalForceMag>=20)
            {
                totalForceMag = 20;
            }

            // resultant acceleration due to the forces
            float accX = totalForceMag*forceX/mass;
            float accY = totalForceMag*forceY/mass;
            float accZ = totalForceMag*forceZ/mass;

            // apply gravitational accelration
            if(z0>=0.001)
            {
                accZ = accZ - g;
            }

            float x1, y1, z1, vx1, vy1, vz1;

            // If the UAVs are far away from the sphere then pull them closer else apply the resultant of the three forces
            if(D>=13)
            {

                float accMag = sqrt(pow(ax,2) + pow(ay,2) + pow(az,2));
                ax /= accMag;
                ay /= accMag;
                az /= accMag;

                if(accMag>=20)
                {
                    accMag = 20;
                }

                float vx1Temp, vy1Temp, vz1Temp;
                x1 = calcualteUAVsLocation(x0, vx0, accMag*ax/mass); vx1Temp = calculateVelocity(vx0, accMag*ax/mass);
                y1 = calcualteUAVsLocation(y0, vy0, accMag*ay/mass); vy1Temp = calculateVelocity(vy0, accMag*ay/mass);
                z1 = calcualteUAVsLocation(z0, vz0, accMag*az/mass); vz1Temp = calculateVelocity(vz0, accMag*az/mass);

                float velMag = sqrt(pow(vx1Temp,2) + pow(vy1Temp,2) + pow(vz1Temp,2));

                // unit vectors for velocity
                vx1 = vx1Temp/velMag;
                vy1 = vy1Temp/velMag;
                vz1 = vz1Temp/velMag;   

                velMag = velMag>=2?2:velMag;

                vx1 *= velMag;
                vy1 *= velMag;
                vz1 *= velMag;             
            }
            else
            {
                float vx1Temp, vy1Temp, vz1Temp;
                x1 = calcualteUAVsLocation(x0, vx0, accX); vx1Temp = calculateVelocity(vx0, accX);
                y1 = calcualteUAVsLocation(y0, vy0, accY); vy1Temp = calculateVelocity(vy0, accY);
                z1 = calcualteUAVsLocation(z0, vz0, accZ); vz1Temp = calculateVelocity(vz0, accZ);

                float velMag = sqrt(pow(vx1Temp,2) + pow(vy1Temp,2) + pow(vz1Temp,2));

                // unit vectors for velocity
                vx1 = vx1Temp/velMag;
                vy1 = vy1Temp/velMag;
                vz1 = vz1Temp/velMag;   

                velMag = velMag>=10?10:velMag;
                velMag = velMag<=2?2:velMag;

                vx1 *= velMag;
                vy1 *= velMag;
                vz1 *= velMag;
            }

            sendBuffer[0] = x1; sendBuffer[1] = y1; sendBuffer[2] = z1;
            sendBuffer[3] = vx1; sendBuffer[4] = vy1; sendBuffer[5] = vz1;
            

            MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
        }
    }
    MPI_Finalize();
    return 0;
}

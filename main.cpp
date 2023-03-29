/////////////////////////////////////////////////////////////////////////////
// OpenHaptics QuickHaptics - TeethCavityPick example
// SensAble Technologies, Woburn, MA
// November 11, 2008
// Programmer: Venkat Gourishankar
//////////////////////////////////////////////////////////////////////////////
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>
#include <QHHeadersGLUT.h>  //Include all necessary headers

#include <cstdio>
///
#include <cmath>
#include <eigen3/Eigen/Geometry>

#include "cameras.h"
///
#include "OptiTrack.h"
///
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <map>
#include <memory>
// #include <stack>
#include <thread>  // multithreading (using libs=-lpthread)
#include <vector>

#include "Model.h"
#include "Object.h"
#include "coordinateTransform.h"
#include "omp.h"  // parallel for loop (using libs=-lgomp and CXXFLAGS=-fopenmp)

Eigen::Vector3d hcurr_temp;
Eigen::Vector3d ocurr_temp;

std::vector<Eigen::Vector3d> optitrack_ptsA;
std::vector<Eigen::Vector3d> haptic_ptsB;
bool matrix_flag = false;

std::shared_ptr<coordinateTransform> t;  // = std::make_shared<coordinateTransform>();
std::shared_ptr<OptiTrack> opti = std::make_shared<OptiTrack>();

Text* notCallibrate = nullptr;
Text* calibInstruction = nullptr;

class DataTransportClass  // This class carried data into the ServoLoop thread
{
   public:
    TriMesh* c1;
    TriMesh* c2;
};
double chargeRadius = 10.0;  // This variable defines the radius around the charge when the inverse square law changes to a spring force law.
double multiplierFactor = 40.0;
hduMatrix WorldToDevice;  // This matrix contains the World Space to DeviceSpace Transformation
hduVector3Dd forceVec;    // This variable contains the force vector.

// Callback functions
void glut_main(int, char**);
void button1DownCallback(unsigned int ShapeID);
void button1UpCallback(unsigned int ShapeID);
void touchCallback(unsigned int ShapeID);

void graphicsCallback(void);
void glutMenuFunction(int);

void HLCALLBACK computeForceCB(HDdouble force[3], HLcache* cache, void* userdata);                    // Servo loop callback
void HLCALLBACK startEffectCB(HLcache* cache, void* userdata);                                        // Servo Loop callback
void HLCALLBACK stopEffectCB(HLcache* cache, void* userdata);                                         // Servo Loop callback
hduVector3Dd forceField(hduVector3Dd Pos1, hduVector3Dd Pos2, HDdouble Multiplier, HLdouble Radius);  // This function computer the force beween the Model and the particle based on the positions

int main(int argc, char** argv) {
    std::thread t1(&OptiTrack::run, opti, argc, argv);
    std::thread t2(glut_main, argc, argv);

    t1.join();
    t2.join();
    return 0;
}

void glut_main(int argc, char** argv) {
    QHGLUT* DisplayObject = new QHGLUT(argc, argv);  // create a display window

    DeviceSpace* deviceSpace = new DeviceSpace;  // Find a Phantom device named "Default PHANToM"
    DisplayObject->tell(deviceSpace);            // tell Quickhaptics that Omni exists
    DisplayObject->setBackgroundColor(0.0, 0.0, 0.6);

    DisplayObject->setHapticWorkspace(hduVector3Dd(-40, -40.0, -17.0), hduVector3Dd(95, 45.0, 17.0));

    DataTransportClass dataObject;  // Initialize an Object to transport data into the servoloop callback

    // Load cube1 model
    dataObject.c1 = new TriMesh("Models/t21.obj");
    dataObject.c1->setName("cube1");
    dataObject.c1->setShapeColor(1.0, 0.5, 0.65);
    dataObject.c1->setRotation(hduVector3Dd(1.0, 0.0, 0.0), 45.0);
    dataObject.c1->setTranslation(hduVector3Dd(0.0, 0.0, 0.0));
    dataObject.c1->setStiffness(0.6);
    dataObject.c1->setDamping(0.1);
    dataObject.c1->setFriction(0.0, 0.9);
    dataObject.c1->setMass(0.9);
    DisplayObject->tell(dataObject.c1);  // Tell quickhaptics that cube exists

    // Load cube2 model
    dataObject.c2 = new TriMesh("Models/c21.obj");
    dataObject.c2->setName("cube2");
    dataObject.c2->setShapeColor(0.1, 0.5, 0.65);
    dataObject.c2->setRotation(hduVector3Dd(1.0, 0.0, 0.0), 45.0);
    dataObject.c2->setTranslation(hduVector3Dd(50.0, 0.0, 0.0));
    dataObject.c2->setStiffness(0.6);
    dataObject.c2->setDamping(0.1);
    dataObject.c2->setFriction(0.0, 0.9);
    dataObject.c2->setMass(0.9);
    DisplayObject->tell(dataObject.c2);  // Tell quickhaptics that cube exists

    Text* text1 = new Text(20.0, "System Not Callibrated!", 0.15, 0.9);
    notCallibrate = text1;
    text1->setName("startupMsg");
    text1->setShapeColor(1.0, 0.05, 0.05);
    text1->setHapticVisibility(false);
    text1->setGraphicVisibility(true);
    DisplayObject->tell(text1);

    text1 = new Text(10.0, "Use right-click to push points (min 3 pts) then callibrate.", 0.1, 0.85);
    calibInstruction = text1;
    text1->setName("instructionMsg");
    text1->setShapeColor(1.0, 0.05, 0.05);
    text1->setHapticVisibility(false);
    text1->setGraphicVisibility(true);
    DisplayObject->tell(text1);

    Cursor* OmniCursor = new Cursor("Models/myCurser.obj");  // Load a cursor
    TriMesh* cursorModel = OmniCursor->getTriMeshPointer();
    OmniCursor->setName("devCursor");  // Give it a name

    cursorModel->setShapeColor(0.35, 0.35, 0.35);
    OmniCursor->scaleCursor(0.007);
    OmniCursor->setRelativeShapeOrientation(0.0, 0.0, 1.0, -90.0);

    //    OmniCursor->debugCursor(); //Use this function the view the location of the proxy inside the Cursor mesh
    DisplayObject->tell(OmniCursor);  // Tell QuickHaptics that the cursor exists

    DisplayObject->preDrawCallback(graphicsCallback);
    deviceSpace->startServoLoopCallback(startEffectCB, computeForceCB, stopEffectCB, &dataObject);  // Register the servoloop callback

    deviceSpace->button1UpCallback(button1UpCallback);

    // Create the GLUT menu
    glutCreateMenu(glutMenuFunction);
    // Add entries
    glutAddMenuEntry("push point", 0);
    glutAddMenuEntry("pop point", 1);
    glutAddMenuEntry("calibrate", 2);
    // attache to the menu
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    qhStart();  // Set everything in motion
    // return 0;
}

void button1DownCallback(unsigned int ShapeID) {
    TriMesh* modelTouched = TriMesh::searchTriMesh(ShapeID);
    Box* buttonTouched = Box::searchBox(ShapeID);
}

void button1UpCallback(unsigned int ShapeID) {
    // draggingGumModel = false;
    // draggingTeethModel = false;
    // draggingCavityFillModel = false;
}

void touchCallback(unsigned int ShapeID) {
    TriMesh* modelTouched = TriMesh::searchTriMesh(ShapeID);
}

void graphicsCallback() {
    // hduMatrix globalDragTransform;

    /////////////////////////////////////////////////////////////////////////////////////////////// getting cursor position
    Cursor* localDeviceCursor = Cursor::searchCursor("devCursor");  // Get a pointer to the cursor
    hduVector3Dd localCursorPosition;
    localCursorPosition = localDeviceCursor->getPosition();  // Get the local cursor position in World Space
    printf("--------------------------------------------------------- %f, %f, %f\n", localCursorPosition[0], localCursorPosition[1], localCursorPosition[2]);

    hcurr_temp[0] = (double)localCursorPosition[0];
    hcurr_temp[1] = (double)localCursorPosition[1];
    hcurr_temp[2] = (double)localCursorPosition[2];
    ///////////////////////////////////////////////////////////////////////////////////////////////

    if (matrix_flag) {
        notCallibrate->setGraphicVisibility(false);
        calibInstruction->setGraphicVisibility(false);
    }
}

/***************************************************************************************
 Servo loop thread callback.  Computes a force effect. This callback defines the motion
 of the purple skull and calculates the force based on the "real-time" Proxy position
 in Device space.
****************************************************************************************/
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache* cache, void* userdata) {
    DataTransportClass* localdataObject = (DataTransportClass*)userdata;  // Typecast the pointer passed in appropriately
    HDdouble instRate = 0.0;
    HDdouble deltaT = 0.0;
    static float counter = 0.0;
    float degInRad = 0.0;
    static int counter1 = 0;

    // Get the time delta since the last update.
    hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
    deltaT = 1.0 / instRate;
    counter += deltaT;
    degInRad = counter * 20 * 3.14159 / 180;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////// transforming object
    // hduVector3Dd ModelPos1 = localdataObject->c1->getTranslation();
    // localdataObject->c1->setTranslation(-ModelPos1);

    // hduVector3Dd ModelPos2 = localdataObject->c2->getTranslation();
    // localdataObject->c2->setTranslation(-ModelPos2);

    Eigen::Vector3f p = (opti->rigidObjects)[1]->position;

    ocurr_temp[0] = (double)p[0];
    ocurr_temp[1] = (double)p[1];
    ocurr_temp[2] = (double)p[2];

    // transform optitrack coordinate into haptic system
    Eigen::Vector3d opti_to_hapt(0., 0., 0.);
    Eigen::Vector3d opti_double;

    opti_double[0] = (double)p[0];
    opti_double[1] = (double)p[1];
    opti_double[2] = (double)p[2];

    if (matrix_flag) {
#pragma opm parallel
        {
            t->transformPointFromSysAtoSysB(opti_double, opti_to_hapt);
            // std::cout << "----------Transformation Matrix----------" << std::endl;
            // std::cout << t->transformMat << std::endl;
            // std::cout << "----------Error----------" << std::endl;
            // double ee = t->errorCalculation();
            // std::cout << ee << std::endl;
        }
    }

    // printf("************************************************************** %i", haptic_ptsB.size());

    // HDdouble x = (HDdouble)(-p[0]);
    // HDdouble y = (HDdouble)(p[1]);
    // HDdouble z = (HDdouble)(p[2]);

    HDdouble x = (HDdouble)(opti_to_hapt[0]);
    HDdouble y = (HDdouble)(opti_to_hapt[2]);
    HDdouble z = (HDdouble)(opti_to_hapt[1]);

    // printf("-------------------- %f, %f, %f\n", x, z, y);
    double rX;  // = (opti->rigidObjects)[1]->rotation[0] * 3.14159 / 180.0;
    double rY;  // = (opti->rigidObjects)[1]->rotation[1] * 3.14159 / 180.0;
    double rZ;  // = (opti->rigidObjects)[1]->rotation[2] * 3.14159 / 180.0;
#pragma opm parallel
    {
        rX = (opti->rigidObjects)[1]->rotation[0] * 3.14159 / 180.0;
        rY = (opti->rigidObjects)[1]->rotation[1] * 3.14159 / 180.0;
        rZ = (opti->rigidObjects)[1]->rotation[2] * 3.14159 / 180.0;
    }

    hduMatrix rZM;  // = hduMatrix::createRotationAroundZ(rZ);
    hduMatrix rYM;  // = hduMatrix::createRotationAroundY(rY);
    hduMatrix rXM;  // = hduMatrix::createRotationAroundX(rX);

    hduMatrix rxyz;  // = rZM * rYM * rXM;

#pragma opm parallel
    {
        rZM = hduMatrix::createRotationAroundZ(rZ);
        rYM = hduMatrix::createRotationAroundY(rY);
        rXM = hduMatrix::createRotationAroundX(rX);
        rxyz = rZM * rYM * rXM;
    }

    localdataObject->c1->setTransform(rxyz);  // rotate the skull with the optitrack trackerocurr_temp

    localdataObject->c1->setTranslation(x, z, y);  // move the skull with the optitrack tracker

    // localdataObject->c1->setScaleInPlace(0.3);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // WorldToDevice.multVecMatrix(localdataObject->Model->getTranslation(), skullPositionDS);  // Convert the position of the sphere from world space to device space

    // hlCacheGetDoublev(cache, HL_PROXY_POSITION, proxyPosition);  // Get the position of the proxy in Device Coordinates (All HL commands in the servo loop callback fetch values in device coordinates)

    // forceVec = forceField(localdataObject->c1->getTranslation(), localdataObject->c2->getTranslation(), multiplierFactor, chargeRadius);  // Calculate the force

    // counter1++;
    // if (counter1 > 2000)  // Make the force start after 2 seconds of program start. This is because the servo loop thread executes before the graphics thread.
    //                       // Hence global variables set in the graphics thread will not be valid for sometime in the begining og the program
    // {
    //     force[0] = forceVec[0];
    //     force[1] = forceVec[1];
    //     force[2] = forceVec[2];
    //     counter1 = 2001;
    // } else {
    force[0] = 0.0;
    force[1] = 0.0;
    force[2] = 0.0;
    // }
}

/******************************************************************************
 Servo loop thread callback called when the effect is started.
******************************************************************************/
void HLCALLBACK startEffectCB(HLcache* cache, void* userdata) {
    DataTransportClass* localdataObject = (DataTransportClass*)userdata;
    printf("Custom effect started\n");
}

/******************************************************************************
 Servo loop thread callback called when the effect is stopped.
******************************************************************************/
void HLCALLBACK stopEffectCB(HLcache* cache, void* userdata) {
    printf("Custom effect stopped\n");
}

/*******************************************************************************
 Given the position of the two charges in space,
 calculates the (modified) coulomb force.
*******************************************************************************/
hduVector3Dd forceField(hduVector3Dd Pos1, hduVector3Dd Pos2, HDdouble Multiplier, HLdouble Radius) {
    hduVector3Dd diffVec = Pos2 - Pos1;  // Find the difference in position
    double dist = 0.0;
    hduVector3Dd forceVec(0, 0, 0);

    HDdouble nominalMaxContinuousForce;
    hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &nominalMaxContinuousForce);  // Find the max continuous for that the device is capable of

    dist = diffVec.magnitude();

    if (dist < Radius * 2.0)  // Spring force (when the model and cursor are within a 'sphere of influence'
    {
        diffVec.normalize();
        forceVec = (Multiplier)*diffVec * dist / (4.0 * Radius * Radius);
        static int i = 0;
    } else  // Inverse square attraction
    {
        forceVec = Multiplier * diffVec / (dist * dist);
    }

    for (int i = 0; i < 3; i++)  // Limit force calculated to Max continuouis. This is very important because force values exceeding this value can damage the device motors.
    {
        if (forceVec[i] > nominalMaxContinuousForce)
            forceVec[i] = nominalMaxContinuousForce;

        if (forceVec[i] < -nominalMaxContinuousForce)
            forceVec[i] = -nominalMaxContinuousForce;
    }

    return forceVec;
}

void glutMenuFunction(int MenuID) {
    if (MenuID == 0) {
        haptic_ptsB.push_back(hcurr_temp);
        optitrack_ptsA.push_back(ocurr_temp);
    }

    if (MenuID == 1) {
        if (haptic_ptsB.size() >= 0)
            haptic_ptsB.pop_back();
        if (optitrack_ptsA.size() >= 0)
            optitrack_ptsA.pop_back();
    }

    if (MenuID == 2) {
        // calibrate
        t = std::make_shared<coordinateTransform>(optitrack_ptsA, haptic_ptsB);
        // t = new coordinateTransform(optitrack_ptsA, haptic_ptsB);
        // std::cout << "----------Transformation Matrix----------" << std::endl;
        t->calculateTransformMatrix();
        // std::cout << t.transformMat << std::endl;
        matrix_flag = true;
    }
}

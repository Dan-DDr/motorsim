#ifndef MAIN_H
#define MAIN_H

// Standard Headers
#include <iostream>
#include <cstdlib>

// Eigen Library
#include <Eigen/Dense>

// User Headers
#include <motorModel.h>
#include <logging.h>
#include <controller.h>

// RapidJson Include Headers
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/error/en.h>


// Global Constants
MotorModel *Motor;
PID *DCtrl, *QCtrl;

double dt = 0;
int steps = 0;

// Function Prototypes
int runDQSimulation();
int parseParams();
int testLogger();
#endif
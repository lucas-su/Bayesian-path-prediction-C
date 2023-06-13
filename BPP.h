#pragma once
#define _USE_MATH_DEFINES

#include <string>
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <Eigen>
#include <Eigenvalues>
#include <Cholesky> 
#include <QR>    
#include <Dense>

#include <logaddexp.h>
#include <dists.cpp>

#include <random>

#include <numeric>


double frequency = 100;
float timespan = 0.1;
double deltaT = 1/frequency;
int Nsteps = frequency*timespan;
int Niters = 100;

std::random_device rd {};
std::mt19937 randflt {rd()}; 

const double operator_m = 2;
const double max_acc = 2; 

std::vector<float> object_likelihoods; // for calculating most likely object based on data - merge with object posterior?

const std::vector<int> object_size = {2,2,2};
// # object locations are defined at their center
const std::vector<std::vector<float>> object_positions = {{1,1,1},{7,7,7},{9,9,9}};
const std::vector<float> object_priors = {0.1,0.0,0.9};



Eigen::Vector3d initial_pos;
Eigen::Vector3d initial_vel;
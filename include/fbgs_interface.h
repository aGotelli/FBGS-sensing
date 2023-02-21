/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#pragma once

#include <Eigen/Core>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <array>


#include "cosseratrodmodel.h"
#include "constantcurvaturemodel.h"
#include "piecewiseconstantcurvaturemodel.h"
#include "pseudorigidbodymodel.h"
#include "subsegmentcosseratrodmodel.h"


// This class implements a simple interface to the FBGS sensing system utilizing TCP sockets
class FBGSInterface
{
public:
	// Constructor 
	FBGSInterface();
	
	// Simple destructor
	~FBGSInterface();

private:


};


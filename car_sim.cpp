/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

 /**
  *    \file   /src/car_sim.cpp
  *    \author Haimin Hu
  *    \date   2017
  *    \content car_sim, for ME231A Project
  */

#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

#define pi 3.1415926535

int main( ){

    // Parameters:
    // -------------------------
    const int L = 1.6;                              // Wheel base

    // Disturbance:
    OnlineData w1;
    OnlineData w2;

    // DIFFERENTIAL STATES:
    // Kinematic bicycle -------------------------
    DifferentialState x;                           // nominal states
    DifferentialState y;
    DifferentialState v;
    DifferentialState phi;
     
    Control a;                                     // Acceleration 
    Control delta;                                 // Steering angle
    
    // // Dubins car -------------------------
    // DifferentialState x;
    // DifferentialState y;    
    // DifferentialState v;    
    // DifferentialState theta;

    // Control ua;                                     // Controls
    // Control us; 

    // DEFINE A DISCRETE−TIME SYTSEM (DOUBLE INTEGRATOR):
    // --------------------------------------------------
    DifferentialEquation f;
    f << dot(x)   == v*cos(phi);
    f << dot(y)   == v*sin(phi);
    f << dot(v)   == a;
    f << dot(phi) == v*tan(delta)/L;
    // f << dot(x) == v*cos(theta);                    // Linear velocity - X axis
    // f << dot(y) == v*sin(theta);                    // Linear velocity - Y axis
    // f << dot(v) == ua;                              // Linear acceleration
    // f << dot(theta) == us;                          // Angular velocity                                  // Acceleration

    SIMexport sim( 1, 0.1 );
    sim.setModel( f );
    
    sim.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    sim.set( GENERATE_MATLAB_INTERFACE, YES );
    sim.set( NUM_INTEGRATOR_STEPS, 20 );
    
    sim.exportCode( "car_sim_CG" );

    return EXIT_SUCCESS;
}
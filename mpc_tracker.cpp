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
  *    \file   /src/mpc_tracker.cpp
  *    \author Haimin Hu
  *    \date   2017
  *    \content mpc_tracker, for ME231A Project
  */

#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

#define pi 3.1415926535

int main( ){

    // Parameters:
    // -------------------------
    const int L = 1.6;                                       // Wheel base

    // DIFFERENTIAL STATES:
    // Kinematic bicycle -------------------------
    DifferentialState x;                           // nominal states
    DifferentialState y;
    DifferentialState v;
    DifferentialState phi;
     
    Control a;                                     // Acceleration 
    Control delta;                                 // Steering angle

    // Dubins car -------------------------
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
    // f << dot(theta) == us;                          // Angular velocity

    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h, hN;

    // h << x;
    // h << y;
    // h << v;
    // h << theta;
    // h << ua;
    // h << us;

    // hN << x;
    // hN << y;
    // hN << v;
    // hN << theta;

    h << x;
    h << y;
    h << v;
    h << phi;
    h << a;
    h << delta;

    hN << x;
    hN << y;
    hN << v;
    hN << phi;

    BMatrix W  = eye<bool>( h.getDim() );
    BMatrix WN = eye<bool>( hN.getDim() );

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 0.5, 5 );

    ocp.minimizeLSQ( W, h );
    ocp.minimizeLSQEndTerm( WN, hN );

    ocp.subjectTo( f );     
    ocp.setNU(2);   

    ocp.subjectTo( -15.0 <= x <= 15.0 );              // State & input Constraints
    ocp.subjectTo( -15.0 <= y <= 15.0 );
    ocp.subjectTo(  -2.0 <= v <= 2.0 );
    ocp.subjectTo( -2.0 <= phi <= 2.0 );

    // DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
    // ----------------------------------------------------------
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,         GAUSS_NEWTON       );
    mpc.set( DISCRETIZATION_TYPE,           MULTIPLE_SHOOTING  );
    mpc.set( SPARSE_QP_SOLUTION,            FULL_CONDENSING_N2 );
    //~ mpc.set( SPARSE_QP_SOLUTION,          SPARSE_SOLVER      );
    //~ mpc.set( INTEGRATOR_TYPE,             INT_IRK_GL2        );
    //~ mpc.set( IMPLICIT_INTEGRATOR_MODE,    LIFTED             );
    mpc.set( INTEGRATOR_TYPE,               INT_RK4            );
    mpc.set( NUM_INTEGRATOR_STEPS,          40                  );
    mpc.set( QP_SOLVER,                     QP_QPOASES3         );
    mpc.set( HOTSTART_QP,                   NO                 );
    mpc.set( LEVENBERG_MARQUARDT,           1e-10              );
    mpc.set( GENERATE_TEST_FILE,            YES                );
    mpc.set( GENERATE_MAKE_FILE,            YES                );
    mpc.set( GENERATE_MATLAB_INTERFACE,     YES                );
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO                 );

    if (mpc.exportCode( "mpc_tracker_CG" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}
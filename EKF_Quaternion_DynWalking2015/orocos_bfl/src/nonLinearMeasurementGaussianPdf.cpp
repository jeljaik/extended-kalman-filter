/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Jorhabib Eljaik
 * email:  jorhabib.eljaik@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "nonLinearMeasurementGaussianPdf.h"

namespace BFL{
nonLinearMeasurementGaussianPdf::nonLinearMeasurementGaussianPdf ( const Gaussian& additiveNoise )
: AnalyticConditionalGaussianAdditiveNoise (additiveNoise, NUMBEROFCONDITIONALARGUMENTS)
{

}

nonLinearMeasurementGaussianPdf::~nonLinearMeasurementGaussianPdf()
{

}

MatrixWrapper::ColumnVector nonLinearMeasurementGaussianPdf::ExpectedValueGet() const
{
    // This is pretty much the measurement model (Accelerometer in this particular case)
    // State variables and input are private members of ConditionalPDF
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector angVel = ConditionalArgumentGet(1);
    
    MatrixWrapper::ColumnVector expectedValue(4);
    MatrixWrapper::Quaternion tmpQuat(state);
    MatrixWrapper::Matrix Q(3,3);
    Q = tmpQuat.toRotation();
    cout << "[BFL::nonLinearMeasurementGaussianPdf::ExpectedValueGet] Rotation matrxi from quaternion: " << Q ;
    MatrixWrapper::ColumnVector g0(3); g0(1) = 0; g0(2) = 0; g0(3) = 9.81;
    expectedValue = Q*g0;
    return expectedValue; 

}

MatrixWrapper::Matrix nonLinearMeasurementGaussianPdf::dfGet ( unsigned int i ) const
{

}


} // BFL namespace


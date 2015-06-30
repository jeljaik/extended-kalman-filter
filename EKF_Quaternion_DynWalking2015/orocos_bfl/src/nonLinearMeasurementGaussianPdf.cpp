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
    MatrixWrapper::ColumnVector expectedValue(4);
    MatrixWrapper::Quaternion tmpQuat(state);
    MatrixWrapper::Matrix Q(3,3);
    if(!tmpQuat.getRotation(Q))
        std::cout << "[BFL::nonLinearMeasurementGaussianPdf::ExpectedValueGet] Rotation matrix from quaternion could not be computed. " << std::endl;
    MatrixWrapper::ColumnVector g0(3); g0(1) = 0.0; g0(2) = 0.0; g0(3) = 1.0;
    // Transpose or inverse since Q is actually a symmetric matrix
    expectedValue = Q.transpose()*g0;
    
    return expectedValue + AdditiveNoiseMuGet(); 
}

MatrixWrapper::Matrix nonLinearMeasurementGaussianPdf::dfGet ( unsigned int i ) const
{
    MatrixWrapper::ColumnVector state  = ConditionalArgumentGet(0);
    // Remapping for easier copy
    double q0 = state(1);
    double q1 = state(2);
    double q2 = state(3);
    double q3 = state(4);
    if (i == 0) { // Derivative with respect to the first conditional argument (i.e. the 4-dim state)
        // Allocating space for Jacobian
        MatrixWrapper::Matrix dq(3,12);
        dq = 0.0;
        MatrixWrapper::Matrix dQdq0(3,3), dQdq1(3,3), dQdq2(3,3), dQdq3(3,3);
        // NOTE Gravity unity vector. Need to review why I set this to [0 0 1]' instead of [0 0 9.8]' in the Matlab code
        MatrixWrapper::ColumnVector gravUnitVec(3);
        gravUnitVec = 0.0; 
        gravUnitVec(3) = 1.0;
        
        // Partial derivative of Q with respect to q0
        dq(1,1) = 2.0*q0; dq(1,2) = -q3   ; dq(1,3) =  q2   ;
        dq(2,1) =  q3   ; dq(2,2) = 2.0*q0; dq(2,3) = -q1   ;
        dq(3,1) = -q2   ; dq(3,2) =  q1   ; dq(3,3) = 2.0*q0;
        dQdq0 = (dq.sub(1,3,1,3))*2.0;
        
        // Partial derivative of Q with respect to q1
        dq(1,4) = 2.0*q1; dq(1,5) = q2    ; dq(1,6) = q3    ;
        dq(2,4) = q2    ; dq(2,5) = 0.0   ; dq(2,6) = -q0   ;
        dq(3,4) = q3    ; dq(3,5) = q0    ; dq(3,6) = 0.0   ;
        dQdq1 = (dq.sub(1,3,4,6))*2.0;
        
        // Partial derivative of Q with respect to q2
        dq(1,7) = 0.0   ; dq(1,8) = q1    ; dq(1,9) = q0     ;
        dq(2,7) = q1    ; dq(2,8) = 2.0*q2; dq(2,9) = q3     ;
        dq(3,7) = -q0   ; dq(3,8) = q3    ; dq(3,9) = 0.0    ;
        dQdq2 = (dq.sub(1,3,7,9))*2.0;
        
        // Partial derivative of Q with respect to q3
        dq(1,10) = 0.0  ; dq(1,11) = -q0  ; dq(1,12) = q1    ;
        dq(2,10) = q0   ; dq(2,11) = 0.0  ; dq(2,12) = q2    ;
        dq(3,10) = q1   ; dq(3,11) = q2   ; dq(3,12) = 2.0*q3;
        dQdq3 = (dq.sub(1,3,10,12))*2.0;
        
        MatrixWrapper::ColumnVector dhdq_col1 = dQdq0.transpose()*gravUnitVec;
        MatrixWrapper::ColumnVector dhdq_col2 = dQdq1.transpose()*gravUnitVec;
        MatrixWrapper::ColumnVector dhdq_col3 = dQdq2.transpose()*gravUnitVec;
        MatrixWrapper::ColumnVector dhdq_col4 = dQdq3.transpose()*gravUnitVec;
        
        MatrixWrapper::Matrix dhdq(3,4); dhdq = 0.0;
        dhdq.setColumn(dhdq_col1, 1);
        dhdq.setColumn(dhdq_col2, 2);
        dhdq.setColumn(dhdq_col3, 3);
        dhdq.setColumn(dhdq_col4, 4);

        return dhdq;
    } else {
        if (i >= NumConditionalArgumentsGet()) {
            cout << "This pdf only has " << NumConditionalArgumentsGet() << "conditional arguments \n";
            exit(-BFL_ERRMISUSE);
        } else {
            cout << "dq is not implemented for the "<< i <<"th conditional argument\n";
            exit(-BFL_ERRMISUSE);
        }
    }
    
}


} // BFL namespace


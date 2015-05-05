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

#ifndef __NONLINEARMEASUREMENTGAUSSIANPDF_H__
#define __NONLINEARMEASUREMENTGAUSSIANPDF_H__

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>
#include <bfl/pdf/gaussian.h>
#define NUMBEROFCONDITIONALARGUMENTS 2

namespace BFL {
    class nonLinearMeasurementGaussianPdf: public BFL::AnalyticConditionalGaussianAdditiveNoise
    {
    public:
        nonLinearMeasurementGaussianPdf(const BFL::Gaussian& additiveNoise);
        virtual ~nonLinearMeasurementGaussianPdf();
        
        // Redefinition of necessary virtual methods
        virtual MatrixWrapper::ColumnVector    ExpectedValueGet()     const;
        virtual MatrixWrapper::Matrix          dfGet(unsigned int i)  const;
    };
}

#endif

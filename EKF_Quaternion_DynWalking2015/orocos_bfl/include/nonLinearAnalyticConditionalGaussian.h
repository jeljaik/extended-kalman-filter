#ifndef __NONLINEARANALYTICCONDITIONALGAUSSIAN_H__
#define __NONLINEARANALYTICCONDITIONALGAUSSIAN_H__

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>
#define NUMBEROFCONDITIONALARGUMENTS 2

namespace BFL 
{    
    class nonLinearAnalyticConditionalGaussian: public AnalyticConditionalGaussianAdditiveNoise 
    {
    public:
        // Constructors
        // Distribution not yet known
        nonLinearAnalyticConditionalGaussian ( int dim = 0);
        // Distribution known beforehand
        nonLinearAnalyticConditionalGaussian ( const Gaussian& gaus);
        virtual ~nonLinearAnalyticConditionalGaussian();
        
        // Redefinition of virtual methods
        virtual MatrixWrapper::ColumnVector    ExpectedValueGet()     const;
        virtual MatrixWrapper::Matrix          dfGet(unsigned int i)  const;
        
        // Necessary operators
        MatrixWrapper::ColumnVector OmegaOperator(const MatrixWrapper::ColumnVector omg) const;
        // TODO This should take as input a Quaternion! To be added to the MatrixWrapper class
        MatrixWrapper::Matrix       XiOperator(const MatrixWrapper::Quaternion) const;
    };
}


#endif


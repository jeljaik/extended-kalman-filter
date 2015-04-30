
#include <bfl/filter/extendedkalmanfilter.h>
#include"nonLinearAnalyticConditionalGaussian.h"

namespace BFL {
nonLinearAnalyticConditionalGaussian::nonLinearAnalyticConditionalGaussian ( const BFL::Gaussian& gaus) 
        : AnalyticConditionalGaussianAdditiveNoise ( gaus, NUMBEROFCONDITIONALARGUMENTS )
{
}

nonLinearAnalyticConditionalGaussian::~nonLinearAnalyticConditionalGaussian()
{

}

MatrixWrapper::ColumnVector nonLinearAnalyticConditionalGaussian::ExpectedValueGet() const
{
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector angVel = ConditionalArgumentGet(1);
    // TODO Should I add the DeltaT ?? YES!!! In the tutorial it is just assumed to be one
    state += static_cast<MatrixWrapper::ColumnVector>(0.5*OmegaOperator(angVel)) + AdditiveNoiseMuGet();
    
    // TODO The following lines are just a test
    MatrixWrapper::Quaternion q(1,2,3,4);
    cout << "Real part of the quaternion: " << q.real() << endl;
    cout << "Imaginary part of the quaternion " << q.unreal() << endl;
    cout << q + q;
}

MatrixWrapper::Matrix nonLinearAnalyticConditionalGaussian::dfGet ( unsigned int i ) const
{
  return BFL::AnalyticConditionalGaussian::dfGet ( i );
}

MatrixWrapper::ColumnVector nonLinearAnalyticConditionalGaussian::OmegaOperator (const MatrixWrapper::ColumnVector omg ) const
{

}

MatrixWrapper::Matrix nonLinearAnalyticConditionalGaussian::XiOperator (const MatrixWrapper::Quaternion ) const
{

}

} // BFL namespace


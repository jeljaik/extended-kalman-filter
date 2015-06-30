
#include <bfl/filter/extendedkalmanfilter.h>
#include"nonLinearAnalyticConditionalGaussian.h"

namespace BFL {
nonLinearAnalyticConditionalGaussian::nonLinearAnalyticConditionalGaussian(int dim) 
        : AnalyticConditionalGaussianAdditiveNoise (dim, NUMBEROFCONDITIONALARGUMENTS),
        m_localA(dim, dim)
{
}

nonLinearAnalyticConditionalGaussian::nonLinearAnalyticConditionalGaussian ( const BFL::Gaussian& gaus) 
: AnalyticConditionalGaussianAdditiveNoise ( gaus, NUMBEROFCONDITIONALARGUMENTS ), m_localA(4,4)
{
    //FIXME m_localA can't be allocated like this!!! How do I get the dimension of this state in this class or base class?
}

nonLinearAnalyticConditionalGaussian::~nonLinearAnalyticConditionalGaussian()
{

}

MatrixWrapper::ColumnVector nonLinearAnalyticConditionalGaussian::ExpectedValueGet() const
{
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector angVel = ConditionalArgumentGet(1);
    MatrixWrapper::Matrix identity(4,4);
    identity.toIdentity();
    // TODO I need state dimension here instead of hardcoding 4
    MatrixWrapper::Matrix tmpA(4,4);
    OmegaOperator(angVel, tmpA);
    tmpA = tmpA*(0.5*m_threadPeriod);
    state = state + tmpA*state + AdditiveNoiseMuGet();
    
    // Normalizing the quaternion
    MatrixWrapper::Quaternion tmpQuat(state);
    tmpQuat.normalize();
    // The other  methods of the library use inputs of type ColumnVector, therefore a copy of the quaternion is necessary
    MatrixWrapper::ColumnVector ret(tmpQuat);
    return ret;
}

MatrixWrapper::Matrix nonLinearAnalyticConditionalGaussian::dfGet ( unsigned int i ) const
{
  // NOTE In this case, since this is rather a linear system, dfGet actually corresponds to the 
    //    discrete-time transition matrix. 
    MatrixWrapper::ColumnVector angVel = ConditionalArgumentGet(1);
    MatrixWrapper::Matrix identity(4,4); identity.toIdentity();
    MatrixWrapper::Matrix tmp(4,4);
    OmegaOperator(angVel, tmp);
    MatrixWrapper::Matrix tmpA = identity + tmp*(0.5*m_threadPeriod);
    return tmpA;
}

void nonLinearAnalyticConditionalGaussian::setPeriod ( int period )
{
    m_threadPeriod = period/1000.0;
}

bool nonLinearAnalyticConditionalGaussian::OmegaOperator (const MatrixWrapper::ColumnVector omg, MatrixWrapper::Matrix& Omega) const
{
    bool ret = false;
    if (Omega.size1() == Omega.size2()) {
        if (!Omega.size1() == 4)
            return ret;
    } else {
        cout << "[nonLinearAnalyticConditionalGaussian::OmegaOperator] Expected a 4x4 matrix";
        return ret;
    }
    
    //TODO The size of the state used to build this Omega matrix should be passed to this class somehow.
    // HARDCODED TEMPORARILY TO 4
    Omega(1,1) = 0.0;      Omega(1,2) = -omg(1);   Omega(1,3) = -omg(2);   Omega(1,4) = -omg(3);
    Omega(2,1) = omg(1);   Omega(2,2) =  0.0;      Omega(2,3) =  omg(3);   Omega(2,4) = -omg(2);
    Omega(3,1) = omg(2);   Omega(3,2) = -omg(3);   Omega(3,3) =  0.0;      Omega(3,4) =  omg(1);
    Omega(4,1) = omg(3);   Omega(4,3) =  omg(2);   Omega(4,3) = -omg(1);   Omega(4,4) =  0.0;
    return true;
}

MatrixWrapper::Matrix nonLinearAnalyticConditionalGaussian::XiOperator (const MatrixWrapper::Quaternion ) const
{

}

} // BFL namespace


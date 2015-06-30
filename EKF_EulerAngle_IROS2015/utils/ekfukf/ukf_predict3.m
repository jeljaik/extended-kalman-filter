%UKF_PREDICT3  Augmented (state, process and measurement noise) UKF prediction step
%
% Syntax:
%   [M,P,X,w] = UKF_PREDICT3(M,P,f,Q,R,f_param,alpha,beta,kappa)
%
% In:
%   M - Nx1 mean state estimate of previous step
%   P - NxN state covariance of previous step
%   f - Dynamic model function as inline function,
%       function handle or name of function in
%       form a([x;w],param)
%   Q - Non-singular covariance of process noise w
%   R - Measurement covariance.
%   f_param - Parameters of f               (optional, default empty)
%   alpha - Transformation parameter      (optional)
%   beta  - Transformation parameter      (optional)
%   kappa - Transformation parameter      (optional)
%   mat   - If 1 uses matrix form         (optional, default 0)
%
% Out:
%   M - Updated state mean
%   P - Updated state covariance
%   X - Sigma points of x
%   w - Weights as cell array {mean-weights,cov-weights,c}
% 
% Description:
%   Perform augmented form Unscented Kalman Filter prediction step
%   for model
%
%    x[k+1] = a(x[k],w[k],param)
%
%   Function a should be such that it can be given
%   DxN matrix of N sigma Dx1 points and it returns 
%   the corresponding predictions for each sigma
%   point. 
%
% See also:
%   UKF_PREDICT1, UKF_UPDATE1, UKF_PREDICT2, UKF_UPDATE2, UKF_UPDATE3
%   UT_TRANSFORM, UT_WEIGHTS, UT_MWEIGHTS, UT_SIGMAS 

% Copyright (C) 2003-2006 Simo S�rkk�
% Copyright (C) 2007 Jouni Hartikainen
%
% $Id: ukf_predict3.m 480 2010-10-18 07:45:48Z jmjharti $
%
% This software is distributed under the GNU General Public
% Licence (version 2 or later); please refer to the file
% Licence.txt, included with the software, for details.

function [M,P,X,w,C] = ukf_predict3(M,P,f,Q,R,f_param,alpha,beta,kappa,mat)

  %
  % Check which arguments are there
  %
  if nargin < 2
    error('Too few arguments');
  end
  if nargin < 3
    f = [];
  end
  if nargin < 4
    Q = [];
  end
  if nargin < 5
    R = [];
  end
  if nargin < 6
    f_param = [];
  end
  if nargin < 7
    alpha = [];
  end
  if nargin < 8
    beta = [];
  end
  if nargin < 9
    kappa = [];
  end
  if nargin < 10
    mat = [];
  end

  %
  % Apply defaults
  %
  if isempty(mat)
    mat = 0;
  end

  %
  % Do transform
  % and add process and measurement noises
  %
  MA = [M;zeros(size(Q,1),1);zeros(size(R,1),1)];
  PA = zeros(size(P,1)+size(Q,1)+size(R,1));
  i1 = size(P,1);
  i2 = i1+size(Q,1);
  PA(1:i1,1:i1) = P;
  PA(1+i1:i2,1+i1:i2) = Q;
  PA(1+i2:end,1+i2:end) = R;
  
  tr_param = {alpha beta kappa mat};
  [M,P,C,X_s,X_pred,w] = ut_transform(MA,PA,f,f_param,tr_param);
    
  % Save sigma points
  X = X_s;
  X(1:size(X_pred,1),:) = X_pred;
  
  
% Modified from the copyright version to allow parameter estimation
% Copyright (C) 2002-2006 Simo S�rkk�
%
% $Id: ekf_predict1.m 111 2007-09-04 12:09:23Z ssarkka $
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [M,P] = ekf_predictparam1(X,M,P,A,Q,a,W,param)

  %
  % Check arguments
  %
  if nargin < 4
    A = [];
  end
  if nargin < 5
    Q = [];
  end
   if nargin < 6
    a = [];
  end
  if nargin < 7
    W = [];
  end
  if nargin < 8
    param = [];
  end
  
  
  %
  % Apply defaults
  %
  if isempty(A)
    A = eye(size(M,1));
  end
  if isempty(Q)
    Q = zeros(size(M,1));
  end
  if isempty(W)
    W = eye(size(M,1),size(Q,2));
  end

  if isnumeric(A)
    % nop
  elseif isstr(A) | strcmp(class(A),'function_handle')
    A = feval(A,X,param);
  else
    A = A(X,param);
  end
  %
  % Perform prediction
  %

  if isempty(a)
    M = A*M;
  elseif isnumeric(a)
    M = a;
  elseif isstr(a) | strcmp(class(a),'function_handle')
    M = feval(a,X,param);
  else
    M = a(X,param);
  end
    


  if isnumeric(W)
    % nop
  elseif isstr(W) | strcmp(class(W),'function_handle')
    W = feval(W,X,param);
  else
    W = W(X,param);
  end
    
  P = A * P * A' + W * Q * W';

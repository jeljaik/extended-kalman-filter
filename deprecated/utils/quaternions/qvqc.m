function v_out=qvqc(q,v)
% QVQc(Q,V) performs the operation Q*V*qconj(Q)
%     where the vector is treated as a quaternion with a scalar element of
%     zero. 
%
%     Q and V can be vectors of quaternions and vectors, but they must
%     either be the same length or one of them must have a length of one.
%     The output will have the same shape as V.  Q will be passed through
%     QNORM to ensure it is normalized.
%
% See also QcQV, QNORM

% Release: $Name: quaternions-1_3 $
% $Revision: 1.1 $
% $Date: 2009-07-24 19:14:44 $
 
% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.


if nargin~=2
  error('Two input arguments required');
else
  q     = qconj(q);
  v_out = qcvq(q, v);
end

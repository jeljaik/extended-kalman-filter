function v_out=qvxform(q,v)
% QVXFORM(Q,V) transforms the vector V using the quaternion Q.
%     Specifically performs the operation qconj(Q)*V*Q, where the vector
%     is treated as a quaternion with a scalar element of zero.
%
%     Q and V can be vectors of quaternions and vectors, but they must
%     either be the same length or one of them must have a length of one.
%     The output will have the same shape as V.  Q will be passed through
%     QNORM to ensure it is normalized.
%
% See also QcQV, QVROT.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.14 $
% $Date: 2009-07-24 19:14:44 $
 
% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

if nargin~=2
  error('Two input arguments required');
else
  v_out = qcvq(q, v);
end
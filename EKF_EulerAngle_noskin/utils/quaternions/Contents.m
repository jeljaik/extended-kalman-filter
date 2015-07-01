% Quaternions Toolbox
% Version 1.3 (JASP) 26-Jul-2009
% ========================================================================
% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.
% This software is licensed under the terms of the BSD license.
% See the file license.txt that came with the software for more details.
% ========================================================================
%
% For purposes of these tools, a quaternion, q, is just a four element
% vector where q(1:3) is the "imaginary" or "vector" portion of the
% hypercomplex number, and q(4) is the "real" or "scalar" portion.
% Consequently, if q represents a rotation, then:
%
%   q(1) = v1*sin(phi/2)
%   q(2) = v2*sin(phi/2)
%   q(3) = v3*sin(phi/2)
%   q(4) =    cos(phi/2)
%
% where phi is the amount of rotation about the unit vector [v1 v2 v3].
%
% All tools are vectorized, so "vectors" of quaternions (4xN or Nx4
% matrices) can be handled as well.  Since it is most common to work with
% normalized quaternions (also referred to as "unit quaternions" and
% "versors"), if a set of 4 quaternions, i.e., a 4x4 matrix, is input, the
% tools will attempt to determine the shape of the component quaternions
% (4x1 or 1x4) based on whether the rows or columns are normalized.
%
% Of course, some of the tools, like QDECOMP, only make sense for normalized
% quaternions, and thus those tools enforce normality via QNORM.
%
%   isq     - determines whether or not input is a quaternion
%   isnormq - determines whether or not input is a normalized quaternion
%
%   qconj   - quaternion conjugate
%   qnorm   - normalize quaternion
%   qmult   - multiply quaternions
%
%   qdecomp - decompose quaternion into unit vector and rotation angle
%
%   qcvq    - operation on vector: qconj(q) v q
%   qvqc    - operation on vector: q v qconj(q)
%
% Because the author uses the convention described in "Spacecraft Attitude
% Determination and Control" (Wertz, 1978), the following aliases exist:
%
%   qvxform - quaternion/vector transform (alias for qcvq)
%   qvrot   - quaternion/vector rotation (alias for qvqc)
%
% Likewise, the following operations assume the relationship between the
% DCM and the quaternion is: R*v = qvxform(q, v) = qcvq(q, v).  That is,
% the q that performs the equivalent operation on v is the "right hand
% quaterion".
%
%   q2dcm   - quaternion to direction cosine matrix
%   dcm2q   - direction cosine matrix to quaternion
%
% Note that many more recent applications, particularly computer graphics
% libraries, choose the opposite convention.  That is, the equivalent "q"
% is the "left hand quaternion" and consequently the qvxform/qvrot aliases
% and q2dcm and dcm2q functions would be "backward".
%
% See also QLIB, the Quaternion block library for Simulink.

% Package: $Name: quaternions-1_3 $
% File: $Revision: 1.17 $
% $Date: 2009-07-26 20:25:26 $
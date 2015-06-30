function q=dcm2q(R)
% DCM2Q(R) converts direction cosine matrices into quaternions.
%
%     The resultant quaternion(s) will perform the equivalent vector
%     transformation as the input DCM(s), i.e.:
%
%       qconj(Q)*V*Q = R*V
%
%     where R is the DCM, V is a vector, and Q is the quaternion.  Note that
%     for purposes of quaternion-vector multiplication, a vector is treated
%     as a quaterion with a scalar element of zero.
%
%     If the input is a 3x3xN array, the output will be a vector of
%     quaternions where input direction cosine matrix R(:,:,k) corresponds
%     to the output quaternion Q(k,:).
%
%     Note that this function is meaningless for non-orthonormal matrices!
%
% See also Q2DCM.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.11 $
% $Date: 2009-07-25 04:28:18 $
 
% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

% Thanks to Tatsuki Kashitani for pointing out the numerical instability in
% the original implementation.  His suggested fix also included a check for
% the "sr" values below being zero.  But I think I've convinced myself that
% this isn't necessary if the input matrices are orthonormal (or at least
% very close to orthonormal).

if nargin~=1
  error('One input argument required');
else
  size_R=size(R);
  if ( size_R(1)~=3 || size_R(2)~=3 || length(size_R)>3 )
    error('Invalid input: must be a 3x3xN array')
  end
end

q = zeros( 4, size( R, 3 ) );

for id_dcm = 1 : size( R, 3 )
  dcm = R( :, :, id_dcm );
  if trace( dcm ) > 0
    % Positve Trace Algorithm
    sr  = sqrt( 1 + trace( dcm ));
    sr2 = 2*sr;
    q(1,id_dcm) = ( dcm(2,3) - dcm(3,2) ) / sr2;
    q(2,id_dcm) = ( dcm(3,1) - dcm(1,3) ) / sr2;
    q(3,id_dcm) = ( dcm(1,2) - dcm(2,1) ) / sr2;
    q(4,id_dcm) = 0.5 * sr;
  else
    % Negative Trace Algorithm
    if ( dcm(1,1) > dcm(2,2) ) && ( dcm(1,1) > dcm(3,3) )
      % Maximum Value at DCM(1,1)
      sr  = sqrt( 1 + (dcm(1,1) - ( dcm(2,2) + dcm(3,3) )) );
      sr2 = 2*sr;
      q(1,id_dcm) = 0.5 * sr;
      q(2,id_dcm) = ( dcm(2,1) + dcm(1,2) ) / sr2;
      q(3,id_dcm) = ( dcm(3,1) + dcm(1,3) ) / sr2;
      q(4,id_dcm) = ( dcm(2,3) - dcm(3,2) ) / sr2;
    elseif dcm(2,2) > dcm(3,3)
      % Maximum Value at DCM(2,2)
      sr  = sqrt( 1 + (dcm(2,2) - ( dcm(3,3) + dcm(1,1) )) );
      sr2 = 2*sr;
      q(1,id_dcm) = ( dcm(2,1) + dcm(1,2) ) / sr2;
      q(2,id_dcm) = 0.5 * sr;
      q(3,id_dcm) = ( dcm(2,3) + dcm(3,2) ) / sr2;
      q(4,id_dcm) = ( dcm(3,1) - dcm(1,3) ) / sr2;
    else
      % Maximum Value at DCM(3,3)
      sr  = sqrt( 1 + (dcm(3,3) - ( dcm(1,1) + dcm(2,2) )) );
      sr2 = 2*sr;
      q(1,id_dcm) = ( dcm(3,1) + dcm(1,3) ) / sr2;
      q(2,id_dcm) = ( dcm(2,3) + dcm(3,2) ) / sr2;
      q(3,id_dcm) = 0.5 * sr;
      q(4,id_dcm) = ( dcm(1,2) - dcm(2,1) ) / sr2;
    end
  end
end

% Make quaternion vector a column of quaternions
q=q.';

q=real(q);



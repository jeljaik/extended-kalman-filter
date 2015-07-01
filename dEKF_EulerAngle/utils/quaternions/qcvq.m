function v_out=qcvq(q,v)
% QcVQ(Q,V) performs the operation qconj(Q)*V*Q
%     where the vector is treated as a quaternion with a scalar element of
%     zero.
%
%     Q and V can be vectors of quaternions and vectors, but they must
%     either be the same length or one of them must have a length of one.
%     The output will have the same shape as V.  Q will be passed through
%     QNORM to ensure it is normalized.
%
% See also QVQc, QNORM, QMULT.

% Note that QNORM is invoked by QMULT, therefore QcQV does not invoke
% it directly.
  
% Release: $Name: quaternions-1_3 $
% $Revision: 1.2 $
% $Date: 2009-07-26 20:05:12 $
 
% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.


if nargin~=2
  error('Two input arguments required.');
else

  qtype=isq(q);
  if ( qtype == 0 )
    error('Input Q must be a quaternion or a vector of quaternions')
  end

  size_v=size(v);
  if ( length(size_v)~=2 || max(size_v==3)~=1 )
    error(['Invalid input: second input must be a 3-element vector', 10, ...
           'or a vector of 3-element vectors'])
  end

end

% Make sure q is a column of quaternions
if ( qtype==1 )
  q=q.';
end

% Make sure v is a column of vectors
row_of_vectors = (size_v(2) ~= 3);
if ( row_of_vectors )
  v=v.';
  size_v=size(v);
end

size_q=size(q);

if (  size_q(1)~=size_v(1) && size_q(1)~=1 && size_v(1)~=1 )
  error(['Inputs do not have the same number of elements:', 10, ...
         '   number of quaternions in q = ', num2str(size_q(1)), 10,...
         '   number of vectors in v     = ', num2str(size_v(1)), 10,...
         'Inputs must have the same number of elements, or', 10, ...
         'one of the inputs must have a single element.']) 
elseif ( size_q(1)==1 && size_v(1)==3 )
  if ( qtype==1 )
    warning(...
      'qcvq:assumingVcols', ...
      'Q is 4x1 and V is 3x3: assuming vectors are column vectors')
    row_of_vectors = 1;
    v=v.';
  else
    warning(...
      'qcvq:assumingVrows', ...
      'Q is 1x4 and V is 3x3: assuming vectors are row vectors')
  end
elseif ( qtype==3 && size_v(1)==1 )
  if ( row_of_vectors )
    warning(...
      'qcvq:assumingQcols', ...
      'Q is 4x4 and V is 3x1: assuming quaternions are column vectors')
    q=q.';
  else
    warning(...
      'qcvq:assumingQrows', ...
      'Q is 4x4 and V is 1x3: assuming quaternions are row vectors')
  end  
end

% Build up full vectors if one input is a singleton
if ( size_q(1) ~= size_v(1) )
  ones_length = ones(max(size_q(1),size_v(1)),1);
  if ( size_q(1) == 1 )
    q = [q(1)*ones_length ...
         q(2)*ones_length ...
         q(3)*ones_length ...
         q(4)*ones_length ];
  else % size_v(1) == 1
    v = [v(1)*ones_length ...
         v(2)*ones_length ...
         v(3)*ones_length ];    
  end
end

% Add an element to V
v(:,4)=zeros(size_v(1),1);

% Turn off warnings before calling qconj (it has simillar warnings as
% qvxform, so all warnings would just be duplicated).  Save current state of
% warnings, though.
warning_state = warning; warning('off', 'qconj:indeterminateShape');
local_warning = lastwarn;

% Perform transform
vt=qmult(qconj(q),qmult(v,q));

% Restore warning state to original state
warning(warning_state);
lastwarn(local_warning);

% Eliminate last element of vt for output
v_out = vt(:,1:3);

% Make sure output vectors are the same shape as input vectors
if ( row_of_vectors )
  v_out = v_out.';
end

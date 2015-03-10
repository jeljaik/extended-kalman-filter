%TEST_DCM2Q runs unit tests for the DCM2Q function.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.8 $
% $Date: 2009-07-26 20:05:12 $

% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

test_title = 'dcm2q';
disp_test_title(test_title);

failures=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'One input argument required';
fct_call     = 'dcm2q';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Invalid Input: scalar');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'Invalid input: must be a 3x3xN array';
fct_call     = 'dcm2q(1)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Algorithm');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t=0:2*pi/16:2*pi;
a=sin(t)';
b=cos(t)';
z=0*a;

Qin = qnorm(2*rand(10,4)-1);

Qin = [Qin; [a b z z]];
Qin = [Qin; [a z b z]];
Qin = [Qin; [a z z b]];

Qin = [Qin; [b a z z]];
Qin = [Qin; [z a b z]];
Qin = [Qin; [z a z b]];

Qin = [Qin; [b z a z]];
Qin = [Qin; [z b a z]];
Qin = [Qin; [z z a b]];

Qin = [Qin; [b z z a]];
Qin = [Qin; [z b z a]];
Qin = [Qin; [z z b a]];

% make sure all quaternions have q4>=0
for count=1:length(Qin)
  if Qin(count,4)<0
    Qin(count,:)=-Qin(count,:);
  end
end
Qin, disp(' ') %#ok<NOPTS>

A='q2dcm(Qin)', disp(' ') %#ok<NOPTS>
A=eval(A);

T=(1:length(Qin))*0;
for count=1:length(Qin)
  T(count) = trace(A(:,:,count));
end

Qout='dcm2q(A)', disp(' ') %#ok<NOPTS>
Qout=eval(Qout);

qdiff='qmult(qconj(Qout),Qin)', disp(' ') %#ok<NOPTS>
qdiff=eval(qdiff);

truth_value = '[0.0  0.0  0.0]';
test_value  = 'max(abs(qdiff(:,1:3)))';

failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp_num_failures(test_title, failures)


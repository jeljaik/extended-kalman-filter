%TEST_Q2DCM runs unit tests for the Q2DCM function.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.6 $
% $Date: 2009-07-26 20:05:12 $

% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

test_title = 'q2dcm';
disp_test_title(test_title);

failures=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'q2dcm() requires one input argument';
fct_call     = 'q2dcm';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Invalid Input: scalar');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
    ['Invalid input: must be a quaternion or a vector of' ...
     ' quaternions'];
fct_call     = 'q2dcm(1)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Ambiguous Input: 4x4 non-normalized');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_warn = ...
    'Component quaternion shape indeterminate, assuming row vectors';
fct_call      = 'q2dcm(ones(4,4));';
failures=failures+check_warn(fct_call, expected_warn);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Column of two quaternions');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q=qnorm(2*rand(2,4)-1), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3], disp(' ') %#ok<NASGU,NOPTS>
truth_value = 'qvxform(Q, v)';
A='q2dcm(Q)', disp(' ') %#ok<NOPTS>
A=eval(A); %#ok<NASGU>
test_value='[A(:,:,1)*v.'' A(:,:,2)*v.''].''', disp(' ') %#ok<NOPTS>
failures=failures+check_value(truth_value, test_value, 10*eps);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Row of two quaternions');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q=qnorm(2*rand(4,2)-1), disp(' ') %#ok<NOPTS>
v=rand(3,2), disp(' ') %#ok<NOPTS>
truth_value = 'qvxform(Q, v)';
A='q2dcm(Q)', disp(' ') %#ok<NOPTS>
A=eval(A);
test_value  = '[A(:,:,1)*v(:,1) A(:,:,2)*v(:,2)]';
failures=failures+check_value(truth_value, test_value, 10*eps);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp_num_failures(test_title, failures)


%TEST_QcVQ runs unit tests for the QcVQ function.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.3 $
% $Date: 2009-07-26 20:05:12 $

% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

test_title = 'qcvq';
disp_test_title(test_title);

failures=0;

q=[0 0 0 1], disp(' ') %#ok<NOPTS>


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Number of inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments: no arguments');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'Two input arguments required';
fct_call     = 'qcvq';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments: one argument');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'Two input arguments required';
fct_call     = 'qcvq(1)';
failures=failures+check_err(fct_call, expected_err);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% qaternion invalid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Invalid Input: q is a scalar');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
    'Input Q must be a quaternion or a vector of quaternions';
fct_call     = 'qcvq(1, q)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% vector invalid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Invalid Input: v is a scalar');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
    ['Invalid input: second input must be a 3-element vector', 10, ...
     'or a vector of 3-element vectors'];
fct_call     = 'qcvq(q, 1)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Invalid Input: v is 2D, but neither dim is size 3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
    ['Invalid input: second input must be a 3-element vector', 10, ...
     'or a vector of 3-element vectors'];
fct_call     = 'qcvq(q, [1 2 3 4; 4 5 6 7])';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Invalid Input: v is 3D');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
    ['Invalid input: second input must be a 3-element vector', 10, ...
     'or a vector of 3-element vectors'];
fct_call     = 'qcvq(q, ones(3,3,3))';
failures=failures+check_err(fct_call, expected_err);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% q and v mismatched
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('number of q ~= number of v');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q=[q;q], disp(' ') %#ok<NOPTS>
V=[1 2 3; 4 5 6; 7 8 9; 10 11 12], disp(' ') %#ok<NOPTS>
expected_err = ...
  ['Inputs do not have the same number of elements:', 10, ...
   '   number of quaternions in q = ', num2str(size(Q,1)), 10,...
   '   number of vectors in v     = ', num2str(size(V,1)), 10,...
   'Inputs must have the same number of elements, or', 10, ...
   'one of the inputs must have a single element.'];
fct_call      = 'qcvq(Q, V);';
failures=failures+check_err(fct_call, expected_err);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4x4 quaternion inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Q is 4x1 and V is 3x3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_warn = ['Q is 4x1 and V is 3x3: assuming vectors' ...
                 ' are column vectors'];
fct_call      = 'qcvq(q.'', ones(3,3));';
failures=failures+check_warn(fct_call, expected_warn);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Q is 1x4 and V is 3x3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_warn = ['Q is 1x4 and V is 3x3: assuming vectors' ...
                 ' are row vectors'];
fct_call      = 'qcvq(q, ones(3,3));';
failures=failures+check_warn(fct_call, expected_warn);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Q is 4x4 and V is 3x1');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_warn = ['Q is 4x4 and V is 3x1: assuming quaternions are' ...
                 ' column vectors'];
fct_call      = 'qcvq(ones(4,4), [1; 2; 3]);';
failures=failures+check_warn(fct_call, expected_warn);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Q is 4x4 and V is 1x3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_warn = ['Q is 4x4 and V is 1x3: assuming quaternions are' ...
                 ' row vectors'];
fct_call      = 'qcvq(ones(4,4), [1 2 3]);';
failures=failures+check_warn(fct_call, expected_warn);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Singlets
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('row q, row v');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 -1 2 0.5]),disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3],disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v 0], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(1:3)';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('row q, col v');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 -1 2 0.5]),disp(' ') %#ok<NASGU,NOPTS>
v=[1; 2; 3],disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v; 0], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(1:3).''';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Vector of q, single v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('vector of q (rows), single v (row)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 2 3 4; 5 6 7 8]), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3], disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v 0], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(:,1:3)';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('vector of q (columns), single v (row)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 2 3 4; 5 6 7 8].'), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3], disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v 0], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(1:3,:).''';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('vector of q (columns), single v (column)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 2 3 4; 5 6 7 8].'), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3].', disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v; 0], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(1:3,:)';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('vector of q (rows), single v (column)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 2 3 4; 5 6 7 8]), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3].', disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v; 0], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(:,1:3).''';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Single q, vector of v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('single q (row), vector of v (rows)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 -1 2 0.5]), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3; 4 5 6], disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v [0; 0]], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(:,1:3)';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('single q (column), vector of v (rows)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 -1 2 0.5].'), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3; 4 5 6], disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v [0; 0]], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(1:3,:).''';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('single q (column), vector of v (columns)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 -1 2 0.5].'), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3; 4 5 6].', disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v; [0 0]], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(1:3,:)';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('single q (row), vector of v (columns)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([1 -1 2 0.5]), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3; 4 5 6].', disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v; [0 0]], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(:,1:3).''';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Vector of q, vector of v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('vector of q (rows), vector of v (rows)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([-0.5 -1 2 1; 5 6 7 8]), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3; 4 5 6], disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v [0; 0]], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(:,1:3)';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('vector of q (columns), vector of v (rows)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([-0.5 -1 2 1; 5 6 7 8].'), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3; 4 5 6], disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v [0; 0]], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(1:3,:).''';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('vector of q (columns), vector of v (columns)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([-0.5 -1 2 1; 5 6 7 8].'), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3; 4 5 6].', disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v; [0 0]], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q); %#ok<NASGU>
truth_value = 'qconj_v4_q(1:3,:)';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('vector of q (rows), vector of v (columns)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([-0.5 -1 2 1; 5 6 7 8]), disp(' ') %#ok<NASGU,NOPTS>
v=[1 2 3; 4 5 6].', disp(' ') %#ok<NASGU,NOPTS>
qconj_v4_q = 'qmult(qconj(q), qmult([v; [0 0]], q))',disp(' ') %#ok<NOPTS>
qconj_v4_q = eval(qconj_v4_q);
truth_value = 'qconj_v4_q(:,1:3).''';
test_value  = 'qcvq(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_num_failures(test_title, failures)

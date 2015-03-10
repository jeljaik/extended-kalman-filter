%TEST_QMULT runs unit tests for the QMULT function.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.8 $
% $Date: 2009-07-26 20:05:13 $

% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

test_title = 'qmult';
disp_test_title(test_title);

failures=0;

q=[0 0 0 1], disp(' ') %#ok<NOPTS>


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Invalid inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments: no arguments');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'qmult() requires two input arguments';
fct_call     = 'qmult';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments: one argument');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'qmult() requires two input arguments';
fct_call     = 'qmult(1)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('q1 is invalid');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
    ['Invalid input: q1 must be a quaternion or a vector of',...
     ' quaternions'];
fct_call     = 'qmult(1, q)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('q2 is invalid');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
    ['Invalid input: q2 must be a quaternion or a vector of',...
     ' quaternions'];
fct_call     = 'qmult(q, 1)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('q1 and q2 are vectors of different lengths');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
  ['Inputs do not have the same number of elements:', 10, ...
   '   number of quaternions in q1 = 5', 10,...
   '   number of quaternions in q2 = 3', 10,...
   'Inputs must have the same number of elements, or', 10, ...
   'one of the inputs must be a single quaternion (not a', 10, ...
   'vector of quaternions).'];
fct_call      = 'qmult(ones(4,5), ones(3,4));';
failures=failures+check_err(fct_call, expected_err);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Products
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('quaternions are row vectors');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q1=qnorm([ 1 2.0 3  4]), disp(' ') %#ok<NOPTS>
q2=qnorm([-1 0.5 2 -2]), disp(' ') %#ok<NOPTS>
q1q2 = [ ...
    'q1*[ q2(4) -q2(3)  q2(2) -q2(1)', 10, ...
    '     q2(3)  q2(4) -q2(1) -q2(2)', 10, ...
    '    -q2(2)  q2(1)  q2(4) -q2(3)', 10, ...
    '     q2(1)  q2(2)  q2(3)  q2(4) ]'],disp(' ') %#ok<NOPTS>
q1q2 = eval(q1q2);
truth_value = 'q1q2';
test_value  = 'qmult(q1, q2)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('q1 is a column vector, q2 is a row vector');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = 'q1q2.''';
test_value  = 'qmult(q1.'', q2)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('q1 is a row vector, q2 is a column vector');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = 'q1q2';
test_value  = 'qmult(q1, q2.'')';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('quaternions are column vectors');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q3=qnorm([0.2 -1.0  0.5 0.3].'),disp(' ') %#ok<NOPTS>
q4=qnorm([2.0  1.5 -1.0 0.5].'),disp(' ') %#ok<NOPTS>
q3q4 = [ ...
    '(q3.''*[ q4(4) -q4(3)  q4(2) -q4(1)', 10, ...
    '        q4(3)  q4(4) -q4(1) -q4(2)', 10, ...
    '       -q4(2)  q4(1)  q4(4) -q4(3)', 10, ...
    '        q4(1)  q4(2)  q4(3)  q4(4) ]).'''],disp(' ') %#ok<NOPTS>
q3q4 = eval(q3q4);
truth_value = 'q3q4';
test_value  = 'qmult(q3, q4)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Multiply two vectors of quaternions');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q1=[q1.' q3  ], disp(' ') %#ok<NASGU,NOPTS>
Q2=[q2;  q4.'], disp(' ') %#ok<NASGU,NOPTS>
truth_value = '[q1q2.'' q3q4]';
test_value  = 'qmult(Q1, Q2)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Multiply vector of quaternions by a single quaternion');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q1=q1, disp(' ') %#ok<NASGU,NOPTS>
Q2=[q2.' q4], disp(' ') %#ok<NASGU,NOPTS>
q1q4 = [ ...
    '(q1*[ q4(4) -q4(3)  q4(2) -q4(1)', 10, ...
    '      q4(3)  q4(4) -q4(1) -q4(2)', 10, ...
    '     -q4(2)  q4(1)  q4(4) -q4(3)', 10, ...
    '      q4(1)  q4(2)  q4(3)  q4(4) ])'],disp(' ') %#ok<NOPTS>
q1q4 = eval(q1q4);
truth_value = '[q1q2; q1q4]';
test_value  = 'qmult(Q1, Q2)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Q1 is of indeterminate shape, Q2 is a row');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q1=[q1; ones(3,4)],disp(' ') %#ok<NASGU,NOPTS>
Q2=q2,disp(' ') %#ok<NASGU,NOPTS>
truth_value = 'q1q2';
Q1Q2 = 'qmult(Q1, Q2)',disp(' ') %#ok<NOPTS>
Q1Q2 = eval(Q1Q2); %#ok<NASGU>
test_value  = 'Q1Q2(1,:)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Q1 is of indeterminate shape, Q2 is a column');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q1=[q3 ones(4,3)],disp(' ') %#ok<NASGU,NOPTS>
Q2=q4,disp(' ') %#ok<NASGU,NOPTS>
truth_value = 'q3q4';
Q1Q2 = 'qmult(Q1, Q2)',disp(' ') %#ok<NOPTS>
Q1Q2 = eval(Q1Q2);
test_value  = 'Q1Q2(:,1)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Both inputs 4x4, normalized differently');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q1=[q1; q3.'; q1; q3.'],disp(' ') %#ok<NASGU,NOPTS>
Q2=[q2; q4.'; q2; q4.'].',disp(' ') %#ok<NASGU,NOPTS>
truth_value = '[q1q2; q3q4.''; q1q2; q3q4.'']';
test_value  = 'qmult(Q1, Q2)';
failures=failures+check_float(truth_value, test_value, 1e-15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Both inputs 4x4, and of indeterminate shape');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q1=ones(4),disp(' ') %#ok<NOPTS>
Q2=ones(4),disp(' ') %#ok<NOPTS>
truth_value = '2*[ones(4,3) -ones(4,1)]';
test_value  = 'qmult(Q1, Q2)';
failures=failures+check_float(truth_value, test_value, 1e-15);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp_num_failures(test_title, failures)


%TEST_QVQc runs unit tests for the QVQc function.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.3 $
% $Date: 2009-07-26 20:05:13 $

% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

test_title = 'qvqc';
disp_test_title(test_title);

failures=0;

q=[0 0 0 1], disp(' ') %#ok<NASGU,NOPTS>


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments: no arguments');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'Two input arguments required';
fct_call     = 'qvqc';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments: one argument');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'Two input arguments required';
fct_call     = 'qvqc(1)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Algorithm check');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=qnorm([-1 0.5 1 2; 5 6 7 8]), disp(' ') %#ok<NOPTS>
v=[1 2 3; 4 5 6].', disp(' ') %#ok<NOPTS>
truth_value = 'qvxform(qconj(q), v)';
test_value  = 'qvqc(q, v)';
failures=failures+check_float(truth_value, test_value, 1e-15);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_num_failures(test_title, failures)

%TEST_QCONJ runs unit tests for the QCONJ function.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.8 $
% $Date: 2009-07-26 20:05:12 $

% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

test_title = 'qconj';
disp_test_title(test_title);

failures=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'qconj() requires one input argument';
fct_call     = 'qconj';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Invalid Input');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = ...
    ['Invalid input: must be a quaternion or a vector of' ...
     ' quaternions'];
fct_call     = 'qconj(1)';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Column of two quaternions');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = '[ -ones(2,3) ones(2,1) ]';
test_value  = 'qconj(ones(2,4))';
failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Row of 6 quaternions');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = '[ -ones(3,6); ones(1,6) ]';
test_value  = 'qconj(ones(4,6))';
failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Ambiguous Input: 4x4 normalized in both directions');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_warn = ['Component quaternion shape indeterminate, assuming' ...
                 ' row vectors'];
fct_call      = 'qconj(0.5*ones(4,4));';
failures=failures+check_warn(fct_call, expected_warn);
truth_value = '0.5*[-ones(4,3)  ones(4,1)]';
warning_state = warning; warning('qconj:indeterminateShape', 'off');
test_value  = 'qconj(0.5*ones(4,4))';
failures=failures+check_value(truth_value, test_value);
warning(warning_state);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp_num_failures(test_title, failures)


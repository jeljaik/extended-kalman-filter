%TEST_ISQ runs unit tests for the ISQ function.

% Release: $Name: quaternions-1_3 $
% $Revision: 1.6 $
% $Date: 2009-07-24 19:14:44 $

% Copyright (c) 2000-2009, Jay A. St. Pierre.  All rights reserved.

test_title = 'isq';
disp_test_title(test_title);

failures=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Insufficient Arguments');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expected_err = 'isq() requires one input argument';
fct_call     = 'isq';
failures=failures+check_err(fct_call, expected_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is a scalar');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = '0';
test_value  = 'isq(1)';
failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is 2D, but neither dim is size 4');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = '0';
test_value  = 'isq(ones(3,5))';
failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is 3D');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = '0';
test_value  = 'isq(ones(3,5,4))';
failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is 4xN, N~=4');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = '1';
test_value  = 'isq(rand(4,5)-0.5)';
failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is 4x4, only columns are normalized');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set_val('M', '1+rand(4)')
set_val('qmag', 'sqrt(sum(M.^2,1))')
set_val('qmag', '[qmag; qmag; qmag; qmag]')
set_val('q', 'M./qmag')
truth_value = '1';
test_value  = 'isq(q)';
failed=check_value(truth_value, test_value);
if failed; error('FAILED'); end
failures=failures+failed;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is Nx4, N~=4');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
truth_value = '2';
test_value  = 'isq(rand(3,4))';
failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is 4x4, only rows are normalized');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set_val('M', '1+rand(4)')
set_val('qmag', 'sqrt(sum(M.^2,2))')
set_val('qmag', '[qmag qmag qmag qmag]')
set_val('q', 'M./qmag')
truth_value = '2';
test_value  = 'isq(q)';
failed=check_value(truth_value, test_value);
if failed; error('FAILED'); end
failures=failures+failed;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is 4x4, both rows and columns are normalized');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set_val('q', '0.5*ones(4)')
truth_value = '3';
test_value  = 'isq(q)';
failures=failures+check_value(truth_value, test_value);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_test_name('Input is 4x4, neither rows nor columns are normalized');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set_val('q', 'ones(4)')
truth_value = '3';
test_value  = 'isq(q)';
failures=failures+check_value(truth_value, test_value);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp_num_failures(test_title, failures)

if failures; error('FAILED'); end

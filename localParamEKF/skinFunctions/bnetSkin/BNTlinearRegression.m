function BNTlinearRegression()
%% Example linear regression through Bayesian Network using the BNT Toolbox
% This example makes use of the BNT toolbox
% Sourcce: https://github.com/bayesnet/bnt

% Assume the following simple linear model with two variables X1, X2 and
% output Y. They can all be treated as gaussian random variables, where in
% particuar Y|X=x ~ N(mu + W x, Sigma). In linear regression, given a
% dataset with samples for x1, x2 and y, we are interested in learning the
% parameters of Y, in particular, W, which corresponds to the regression
% matrix. The structure of the network is as follows:
% 
%   X1     X2 
%   |      |
%   |      |
%   |      |
%    \    /
%      Y
%
% All arrows pointing down.

%% Directed acyclic graph (DAG)
% First thing to do is to define the structure of the network, a.k.a,
% define the structure of the DAG we just drew in the summary of the
% problem.

% Number of nodes.
N = 3;
% Initialization of the DAG.
dag = zeros(N,N);
% Definition of the nodes with ordinal number in topologycal order.
X1 = 1; X2 = 2; Y = 3;
% Ones to define the connections from x1 to y and x2 to y
dag([X1 X2], Y) = 1;

%% Specify size and type of each node. In this case they're all
% All nodes are continuous.
continuous_nodes = 1:N;
% The size of each node is 1.
node_sizes = 1*ones(1,N);

%% Specifying probability distributions
% Create the Bayesian Network with no discrete nodes
bnet = mk_bnet(dag, node_sizes, 'discrete', []);
% Assign probability distributions to each variable. X1 and X2 have
% 'root_CPD' for being root nodes.
bnet.CPD{X1} = root_CPD(bnet, X1);
bnet.CPD{X2} = root_CPD(bnet, X2);
% The output Y will be assumed to have a gaussian distribution. Since what
% we are interested in is just the regression matrix, we define the
% distribution with 'clamp_mean' so that the mean is not adjusted during
% learning and 'clamp_cov' to do the same for the covariance, while
% 'cov_type' has been specified to 'dia' so that the estimated Sigma has a
% diagonal structure and to make it easier for the learning.
bnet.CPD{Y}  = gaussian_CPD(bnet, Y, 'clamp_mean', 1, 'clamp_cov', 1, 'cov_type', 'diag');


%% Creating the dataset.
% We now generate a random dataset with 'nsamples'
nsamples = 1000;
w1 = 2;
w2 = 2;
dataset = createDataset(w1, w2, nsamples, 'yes');

%% Training the network
samples = num2cell(dataset);
learntBnet = learn_params(bnet, samples);

%% Viewing the learned parameters
disp('Estimated weights for Y are: ');
%% Violating object privacy 
% because class gaussian_CPD does not have getter methods
s = struct(learntBnet.CPD{Y}); % Little hack to violate variables privacy
weights = s.weights;
disp(weights)

disp('Real weights for fake dataset were: ');
disp([w1 w2]);

end


%% DATASET FUNCTION
function dataset = createDataset(w1,w2,nsamples,plot)
    % The dataset must be built in a particular way. Each column must
    % correspond to a sample, while each row represents a different
    % variable(node) in the network.
    % plot = 'yes' or 'no'
    rng('shuffle');
    dataset = zeros(3, nsamples);
    dataset(1:2, :) = randn(2, nsamples);
    dataset(3, :) = sum(w1*dataset(1, :) + w2*dataset(2,:), 1);
    if (strcmp(plot,'yes'))
        plot3(dataset(1,:), dataset(2,:), dataset(3,:), '.');
    end
end

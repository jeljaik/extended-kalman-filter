function [bnWeights] = bayNetRegressor(loadData)
close all;
clc;

if(nargin<1)
    loadData='no';
end

if(strcmp(loadData,'yes'))
    %% Load dataset
    addpath(genpath([pwd,filesep,'../src']));
    addpath([pwd,filesep,'../../skinFunctions']);

    phase_load=true;
    phase_filter=true;
    phase_show_compensated=true;
    f = filesep;

    load_path = ['..',f,'dump',f];
    load_path_locally_stimulated = ['..',f,'OneFoot',f,'SimulinkScopedStimulation',f];
    part = 'right';

    %% Load mesh
    if strcmp(part,'right')
        rightFoot = 1;
    else
        rightFoot = 0;
    end

    [V,Tr,temp] = read_plot_foot_mesh(rightFoot, 0, [pwd '/..']);

    %% Load taxels' positions matrix
    % the taxelPos variable represents the position (x, y z) of the takels on
    % the foot

    taxelPos = load('../Tmatrix.mat');
    taxelPos = taxelPos.ans;

    %% load the large datasets
    % here we load the datasets used for testing. In particular since different
    % datasets have been saved into a single file, the function parse_log_file alredy divides
    % the datasets having a look to the derivative of the time-stamps
    datasets = parse_log_file([load_path part]);

    %% load the locally stimulated dataset
    % here we load the locally stimulated data (meaning that we stimulated the
    % skin with a smalll (2 mm) probe
    dataset_local = parse_log_file([load_path_locally_stimulated part]);
    dataset_local = dataset_local{1};

    %% Filtering training dataset
    % The following function filters returns taxels with size n x 384,
    % forces n x 6 and activation n x 384 (Matrix of taxels that were effectively active)
    [taxels,forces,activation,idxTaxelsNeverActive]=load_n_filter(dataset_local,false);

    %The following vector will contain the indeces of the taxels that remain
    %active after this first filtering.
    activeTaxelsIndeces = 1:size(dataset_local.dataSkin(:,3:end),2);
    activeTaxelsIndeces(idxTaxelsNeverActive) = [];

    %% CHANGED BY JORHABIB. The size of subsample has been decided empirically,
    % such that rank(Xtr) = size(Xtr,2). This started happening after 32000
    % samples rather than 30000. get_input_output() could have an additional
    % sampling method called 'fullRank' that subsamples 'taxels' based on the
    % smallest full column rank matrix that can be extracted.
    subsample = 34000;
    [Xtr,ytr] = get_input_output(taxels,forces,activation,subsample,'start');
end

if(strcmp(loadData,'no'))
    load trainingData.mat;
end
    
%% Bayesian Linear Regression

%% Directed acyclic graph (DAG)
% First thing to do is to define the structure of the network, a.k.a,
% define the structure of the DAG we just drew in the summary of the
% problem.

% Number of nodes.
N = size(Xtr,2) + 1;
% Initialization of the DAG.
dag = zeros(N,N);
dag(1:end-1, end) = 1;
% Definition of the nodes with ordinal number in topologycal order.
% X1 = 1; X2 = 2; Y = 3;
% Ones to define the connections from x1 to y and x2 to y
% dag([X1 X2], Y) = 1;
Y = N;

%% Specify size and type of each node. In this case they're all
% All nodes are continuous.
continuous_nodes = 1:N;
% The size of each node is 1.
node_sizes = 1*ones(1,N);

%% Specifying probability distributions
disp('Assigning probability distributions...');
% Create the Bayesian Network with no discrete nodes
bnet = mk_bnet(dag, node_sizes, 'discrete', []);
% Assign probability distributions to each variable. X1 and X2 have
% 'root_CPD' for being root nodes.
for (i=1:N-1)
    bnet.CPD{i} = root_CPD(bnet, i);
end

% The output Y will be assumed to have a gaussian distribution. Since what
% we are interested in is just the regression matrix, we define the
% distribution with 'clamp_mean' so that the mean is not adjusted during
% learning and 'clamp_cov' to do the same for the covariance, while
% 'cov_type' has been specified to 'dia' so that the estimated Sigma has a
% diagonal structure and to make it easier for the learning.
bnet.CPD{Y}  = gaussian_CPD(bnet, Y, 'clamp_mean', 1, 'clamp_cov', 1, 'cov_type', 'diag');

%% Creating the dataset
disp('Creating dataset...');
dataset = Xtr';
dataset(end+1, :) = ytr';

%% Training the network
disp('Learning dataset. Bear with me!...');
samples = num2cell(dataset);
learntBnet = learn_params(bnet, samples);

%% Viewing the learned parameters
disp('Estimated weights for Y are: ');
%% Violating object privacy 
% because class gaussian_CPD does not have getter methods
s = struct(learntBnet.CPD{Y}); % Little hack to violate variables privacy
weights = s.weights;
disp(weights);



end
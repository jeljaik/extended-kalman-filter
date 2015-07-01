%IMM_PREDICT  Interacting Multiple Model (IMM) Filter prediction step
%
% Syntax:
%   [X_p,P_p,c_j,X,P] = EIMM_PREDICT(X_ip,P_ip,MU_ip,p_ij,ind,dims,A,a,param,Q)
%
% In:
%   X_ip  - Cell array containing N^j x 1 mean state estimate vector for
%           each model j after update step of previous time step
%   P_ip  - Cell array containing N^j x N^j state covariance matrix for 
%           each model j after update step of previous time step
%   MU_ip - Vector containing the model probabilities at previous time step
%   p_ij  - Model transition matrix
%   ind   - Indices of state components for each model as a cell array
%   dims  - Total number of different state components in the combined system
%   A     - Dynamic model matrices for each linear model and Jacobians of each
%           non-linear model's measurement model function as a cell array
%   a     - Function handles of dynamic model functions for each model
%           as a cell array
%   param - Parameters of a for each model as a cell array
%   Q     - Process noise matrices for each model as a cell array.
%
% Out:
%   X_p  - Predicted state mean for each model as a cell array
%   P_p  - Predicted state covariance for each model as a cell array
%   c_j  - Normalizing factors for mixing probabilities
%   X    - Combined predicted state mean estimate
%   P    - Combined predicted state covariance estimate
%   
% Description:
%   IMM-EKF filter prediction step. If some of the models have linear
%   dynamics standard Kalman filter prediction step is used for those.
%
% See also:
%   EIMM_UPDATE, EIMM_SMOOTH

% History:
%   09.01.2008 JH The first official version.
%
% Copyright (C) 2007,2008 Jouni Hartikainen
%
% $Id: imm_update.m 111 2007-11-01 12:09:23Z jmjharti $
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [X_p,P_p,c_j,X,P] = eimm_predict(X_ip,P_ip,MU_ip,p_ij,ind,dims,A,a,param,Q)
    % The number of models
    m = size(X_ip,2);
    
    % Construct empty cell arrays for ekf_update if a is not specified
    if isempty(a)
        a = cell(1,m);
    end
    
    % Same for a's parameters
    if isempty(param)
        param = cell(1,m);
    end
    
    % Number of models 
    m = length(X_ip);
    
    % Default values for state mean and covariance
    MM_def = zeros(dims,1);
    PP_def = diag(20*ones(dims,1));

    % Normalizing factors for mixing probabilities
    c_j = zeros(1,m);
    for j = 1:m
        for i = 1:m
            c_j(j) = c_j(j) + p_ij(i,j).*MU_ip(i);
        end
    end

    % Mixing probabilities
    MU_ij = zeros(m,m);
    for i = 1:m
        for j = 1:m
            MU_ij(i,j) = p_ij(i,j) * MU_ip(i) / c_j(j);
        end
    end

    % Calculate the mixed state mean for each filter
    X_0j = cell(1,m);
    for j = 1:m
        X_0j{j} = zeros(dims,1);
        for i = 1:m
            X_0j{j}(ind{i}) = X_0j{j}(ind{i}) + X_ip{i}*MU_ij(i,j);
        end
    end
    
    % Calculate the mixed state covariance for each filter
    P_0j = cell(1,m);
    for j = 1:m
        P_0j{j} = zeros(dims,dims);
        for i = 1:m
            P_0j{j}(ind{i},ind{i}) = P_0j{j}(ind{i},ind{i}) + MU_ij(i,j)*(P_ip{i} + (X_ip{i}-X_0j{j}(ind{i}))*(X_ip{i}-X_0j{j}(ind{i}))');
        end
    end

    % Space for predictions
    X_p = cell(1,m);
    P_p = cell(1,m);

    % Make predictions for each model
    for i = 1:m
        [X_p{i}, P_p{i}] = ekf_predict1(X_0j{i}(ind{i}),P_0j{i}(ind{i},ind{i}),A{i},Q{i},a{i},[],param{i});
        %[X_p{i}, P_p{i}] = kf_predict(X_0j{i}(ind{i}),P_0j{i}(ind{i},ind{i}),A{i},Q{i});
    end

    % Output the combined predicted state mean and covariance, if wanted.
    if nargout > 3
        % Space for estimates
        X = zeros(dims,1);
        P = zeros(dims,dims);
        
        % Predicted state mean
        for i = 1:m
            X(ind{i}) = X(ind{i}) + MU_ip(i)*X_p{i};
        end

        % Predicted state covariance
        for i = 1:m
            P(ind{i},ind{i}) = P(ind{i},ind{i}) + MU_ip(i)*(P_p{i} + (X_ip{i}-X(ind{i}))*(X_i{i}-X(ind{i}))');
        end
    end
    
    
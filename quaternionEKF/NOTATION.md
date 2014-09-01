This document is intended to merge the notation used in the reference papers
uused to write this code. 


| LaTeX syntax          |  Variable Syntax | Description |
|:---------------------:|:----------------:|:------------|
|\bar{q}                | q_bar            | regular quaternion  |
|\hat{x}_{k+1,k}        | x_hat_1          | Estimate of the state x at time k+1             |
|\hat{x}_{k, k}         | x_hat_k          | Estimate of the state x at time k            |
|\hat{b}_{k+1, k}       | b_hat_q          | Estimate of the gyroscope bias at time k+1 |
|\hat{\bat{q}}_{k+1, k} | q_bar_hat_1      | Propagated quaternion at time k+1 |
|\Phi                   | Phi              | State transition matrix |
|Q_d                    | Q_d              | Discrete time noise covariance matrix |
|P_{k+1, k}             | P_1              | State covariance matrix at time k+1  |
|H(k)                   | H                | Measurement matrix |
|r                      | r                | Residual |
|z(k+1)                 | z_1              | Current measurement|
|S                      | S                | Covariance of the residual |
|R                      | R                | Measurement noise covariance matrix |
|K                      | K                | Kalman gain |
|\Delta \hat{x}(+)      | Delta_x_hat      | Correction term |
|\delta \hat{q}         | delta_q_hat      | Error quaternion. Difference between quaternion and quaternion estimate|
|\delta \bar{q}         | delta_q_bar      | Small rotation associated with the error quaternion [0.5 \delta\theta ; 1]|
|\delta \bar{q}         | delta_theta      | Error angle vector. 3-Dimensional |
|\tilde{x}              | x_tilde          | Error vector [\delta\theta ; \Delta b]|
|\hat{\bar{q}}_{k+1,k+1}| q_bar_hat_1_1    | Updated quaternion |
|\hat{b}_{k+1,k+1}      | b_hat_1_1        | Updated bias |
|\hat{\omega}_{k+1,k+1} | omega_1_1        | Updated turn rate |
|\omega_{m_{k+1}}       | omega_m_1        | Turn rate measurement |                 
|P_{k+1,k+1}            | P_1_1            | Updated covariance matrix |

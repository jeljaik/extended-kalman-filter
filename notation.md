---
layout:			page
title:			Notation
permalink: 		/notation/
description: 	Notation used in this site.
imagefeature:	equations-bckg.jpg
mathjax: 		true
---


$$
\begin{array}{c|c|l}
\text{LaTex Syntax} & \text{MATLAB Variable Syntax} & \text{Description} \\
\hline
 \bar{q}                  &  q\_bar               & \text{Regular 4-Dimensional quaternion}                                          \\
 \hat{x}_{k+1,k}          &  x\_hat\_1            & \text{Estimate of the state x at time k+1}                                       \\
 \hat{x}_{k, k}           &  x\_hat               & \text{Estimate of the state x at time k}                                         \\
 \hat{b}_{k+1, k}         &  b\_hat\_1            & \text{Estimate of the gyroscope bias at time k+1}                                \\
 \hat{\bar{q}}_{k+1, k}   &  q\_bar\_hat\_1       & \text{Propagated quaternion at time k+1}                                         \\
 \Phi                     &  Phi                  & \text{State transition matrix}                                                   \\
 Q_d                      &  Q\_d                 & \text{Discrete time noise covariance matrix}                                     \\
 P_{k+1, k}               &  P\_1                 & \text{State covariance matrix at time k+1}                                       \\
 H(k)                     &  H                    & \text{Measurement matrix}                                                        \\
 r                        &  r                    & \text{Residual}                                                                  \\
 z(k+1)                   &  z\_1                 & \text{Current measurement}                                                       \\
 S                        &  S                    & \text{Covariance of the residual}                                                \\
 R                        &  R                    & \text{Measurement noise covariance matrix}                                       \\
 K                        &  K                    & \text{Kalman gain}                                                               \\
 \Delta \hat{x}(+)        &  Delta\_x\_hat        & \text{Correction term}                                                           \\
 \delta \hat{q}           &  delta\_q\_hat        & \text{Error quaternion. }                                                        \\
 \delta \bar{q}           &  delta\_q\_bar        & \text{Small rotation associated with the error quaternion}                       \\
 \delta \theta{q}           &  delta\_theta         & \text{Error angle vector. 3-Dimensional}                                       \\
 \tilde{x}                &  x\_tilde             & \text{Error vector $[\delta\theta ; \Delta b]$}                                  \\
 \hat{\bar{q}}_{k+1,k+1}  &  q\_bar\_hat\_1\_1    & \text{Updated quaternion}                                                        \\
 \hat{b}_{k+1,k+1}        &  b\_hat\_1\_1         & \text{Updated bias}                                                              \\
 \hat{\omega}_{k+1,k+1}   &  omega\_1\_1          & \text{Updated turn rate}                                                         \\
 \omega_{m_{k+1}}         &  omega\_m\_1          & \text{Current turn rate measurement}                                             \\
 P_{k+1,k+1}              &  P\_1\_1              & \text{Updated covariance matrix}                                                 \\
 \end{array}
$$




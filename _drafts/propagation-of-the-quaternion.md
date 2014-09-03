---
layout: 		post
title:  		"Propagation Of The Quaternion"
description:    "In this post we will see how to propagate the quaternion, the discrete time state transition and noise covariance matrix before passing to the Kalman Filter propagation equations as done by Trawny N. and Roumeliotis S."
imagefeature: 	4dquaternion.jpg
mathjax: 		true
share: 			true
comments: 		true
modified: 		2014-09-03
---

>&quot;The quaternion is an ideal global attitude representation, since it varies continuously over $S3$ as the attitude changes, avoiding the jumps required by some three-dimensional parametrizations.&quot;
><small><cite title="F. Landis Markley">F. Landis Markley</cite></small>

<section>
  <header>
    <h1 >Sections</h1>
  </header>
<div id="drawer" markdown="1">
*  Auto generated table of contents
{:toc}
</div>
</section>


# Propagation of the Quaternion

Trawny and Roumoliotis use the data from an inertial measurement unit (IMU) as a dynamic model replacement, since it provides measurements of the translational accelerations and rotational velocities. 

The rotational dynamics of a body whose rotation is expressed in quaternions and for which we have an Intertial Measurement Unit (IMU) can be expressed using the definition of the quaternion derivative in conjunction with the gyro noise model. 

$$\begin{align}
\hat{\bar{q}}&=\frac{1}{2} \Omega(\omega_m - b - n_r)\bar{q}(t)\\
\dot{b}&=n_W
\end{align}$$

The previous equations will be integrated in order to get the *state equations*, which are then defined as a seven-element vector composed of the quaternion and the gyro bias:

$$
x(t)=\begin{bmatrix}
\bar{q}\\
b(t)
\end{bmatrix}$$

>&quot;**NOTE TO SELF** In MATLAB code this is equivalent to the integrateForwardDynamics() function in /positionFilter_toolbox/main.m. For this I will have to create a model struct with the right variables and use ODE for the integration. Should I use the first order quaternion integrator instead?.&quot;
><small><cite title="Jorhabib">Jorhabib</cite></small>


# Discrete Time State Transition Matrix

# Noise Covariance Matrices


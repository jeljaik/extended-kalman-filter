---
layout: 		post
title:  		"Quaternion-Based Multiplicative Extended Kalman Filter"
description:    "In this post we analyze how it could be possible to implement a multiplicative quaternion-based Extended Kalman Filter to determine the orientation of a rigid body system."
imagefeature: 	4dquaternion.jpg
mathjax: 		true
share: 			true
comments: 		true
category: 		Multiplicative-EKF
modified: 		2014-09-07
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

Trawny and Roumoliotis use the data from an inertial measurement unit (IMU) as a dynamic model replacement, since it provides measurements of the translational accelerations and rotational velocities. In particular, the simplified gyro noise model relating turn rate $\omega_m$ and real angular velocity $\omega$ as:

$$\mathbf{\omega_m} = \mathbf{\omega} + \mathbf{b} + \mathbf{n}_r \label{gyromodel}$$

Where $\mathbf{n}_r$ stands for the rate noise, assumed to be Gaussian with distribution:

$$\begin{align}
E[\mathbf{n}_r]&=\mathbf{0}_{3 \times 1}\\
E[\mathbf{n}_r(t+\tau)\mathbf{n}^T_r(t)] &= \mathbf{N}_r \delta(\tau)\\
\end{align}$$


Where $\mathbf{N}_r=\sigma^2_r \cdot \mathbf{I}_3$

While the gyro bias is simulated as a random walk process, such that:
$$\begin{align}
\dot{\mathbf{b}}&=\mathbf{n}_w\\
E[\mathbf{n}_w]&=\mathbf{0}_{3\times1}\\
E[\mathbf{n}_w(t+\tau)\mathbf{n}^T_w(t)]&=\mathbf{N}_w \delta(\tau)
\end{align}$$

Where $\mathbf{N}_{\omega}=\sigma^2_w \cdot \mathbf{I}_3$

The bias **needs to be estimated along with the quaternion**! Thus, our state vector will be:

$$\begin{equation}
	\mathbf{x}= \left[\begin{array}{c}
	\mathbf{q}(t)\\
	\mathbf{b}(t)
	\end{array}\right]
\end{equation}$$


The rotational dynamics of a body whose rotation is expressed in quaternions and for which we have an Inertial Measurement Unit (IMU) can be expressed using the definition of the quaternion derivative in conjunction with the gyro noise model, whose expectation yields the prediction equations for the state in the EKF.

$$\begin{align}
\dot{\hat{\mathbf{q}}}&=\frac{1}{2} \Omega(\hat{\omega})\mathbf{q}(t)\\
\dot{\mathbf{b}}&=\mathbf{n}_W
\end{align}$$

Where $\hat{\mathbf{\omega}} = \mathbf{\omega}_m - \hat{\mathbf{b}}$

The previous equations will be integrated with a first order integrator using $\hat{\mathbf{\omega}}$ instead of $\mathbf{\omega}$ in order to get the *state equations*.


>&quot;**NOTE TO SELF** In MATLAB code this is equivalent to the integrateForwardDynamics() function in /positionFilter_toolbox/main.m.&quot;
><small><cite title="Jorhabib">Jorhabib</cite></small>


## Discrete Time State Transition Matrix

## Noise Covariance Matrices

# Extended Kalman Filter

## Update

### Measurement Models

Inspired by [^1] and the measurement models in [^2] we can construct a measurement model for our rigid body system from an Inertial Measurement Unit (IMU), by stacking the accelerometer and magnetometer measurement vectors.

$$\begin{equation}
\mathbf{z} = \left[ \begin{array} {c} 
\mathbf{a} \\
\mathbf{m}
\end{array} \right] 
= f(\mathbf{x}) + \mathbf{v} \\
\end{equation}$$


$$\begin{equation}
\mathbf{z}= \left[ \begin{array}{cc}
C^G_L(q) & 0\\
0        & C^G_L(q)
\end{array} \right] 
\left[ \begin{array}{c}
g\\
h
\end{array} \right] + 
\left[\begin{array}{c}
\mathbf{b}^a\\
\mathbf{b}^m
\end{array} \right] +
\left[ \begin{array}{c}
\mathbf{v}^a\\
\mathbf{v}^m
\end{array}\right]
\end{equation}$$

Where: $x^G = C^G_L(q)x^n$.

The important question is now hw to obtain a relationship betweeen the measurement model error and the state vector, i.e.

$$\begin{equation}
\tilde{\mathbf{z}} = \mathbf{z} - \hat{\mathbf{z}}
\end{equation}$$

$$\begin{equation}
z = f(x)
\end{equation}$$

$$\begin{align}
\mathbf{a} &= C^G_L(q)\mathbf{g} + \mathbf{b}^a + \mathbf{v}^a\\
\mathbf{m} &= C^G_L(q)\mathbf{h} + \mathbf{b}^m + \mathbf{v}^m
\end{align}$$

$$\begin{equation}
\tilde{\mathbf{z}} =
\left[\begin{array}{c}
\mathbf{a} - \hat{\mathbf{a}}\\
\mathbf{m} - \hat{\mathbf{m}}
\end{array}\right] =
\left[\begin{array}{c}
(C^G_L(q) - C^G_L(\hat{q}))\mathbf{g} + \mathbf{b}^a + \mathbf{v}^a\\
(C^G_L(q) - C^G_L(\hat{q}))\mathbf{h} + \mathbf{b}^m + \mathbf{v}^m
\end{array}\right]
\end{equation}$$

Now, taking $\mathbf{q}=\delta\mathbf{q}\otimes\hat{\mathbf{q}}$

$$\begin{equation}
C^L_G = C^L_G(\delta\mathbf{q} \otimes \hat{\mathbf{q}})
\end{equation}$$

Recalling that the rotation of the product of two quaternions is equal to the product of their respective rotations and also that $\delta\mathbf{q}= \delta\mathbf{q}^L_{\hat{L}}$ and $\hat{\mathbf{q}} = \hat{\mathbf{q}}^{\hat{L}}_G$ we thus have:

$$\begin{equation}
C^L_G(\mathbf{q}) = C^L_{\hat{L}}(\delta\hat{\mathbf{q}})C^{\hat{L}}_G (\mathbf{\hat{q}})
\end{equation}$$

For very small rotations $\delta\mathbf{q}$, the small angle approximation can be used, i.e.

$$\begin{equation}
\delta \mathbf{q} = 
\left[ \begin{array}{c}
\delta \mathbf{q}\\
\delta q_4
\end{array} \right]
= \left[\begin{array}{c}
R sin(\delta \mathbf{\theta}/2)\\
cos(\delta \mathbf{\theta}/2)
\end{array}\right]
= \left[ \begin{array}{c}
\frac{1}{2} \delta \mathbf{\theta}\\
1
\end{array} \right]
\end{equation}$$


$$\begin{equation}
\Rightarrow C^L_G(\delta \mathbf{q}) = \mathbf{I}_{3\times3} - S(\delta \mathbf{\theta})
\end{equation}$$

Hence,

$$\begin{align}
(\mathbf{I} - S(\delta\mathbf{\theta})) C^{\hat{L}}_G C(\hat{\mathbf{q}}) &= C^{\hat{L}}_G (\hat{\mathbf{q}}) - S(\delta \mathbf{\theta}) C^{\hat{L}}_G (\hat{\mathbf{q}})\\
C^L_G(\mathbf{q}) - C^L_G(\mathbf{\hat{q}}) &= C^L_{\hat{L}}(\delta \mathbf{q})C^{\hat{L}}_G(\hat{\mathbf{q}}) - C^{\hat{q}}_G (\hat{\mathbf{q}})\\
&=(C^L_{\hat{L}}(\delta\mathbf{q}) - \mathbf{I}) C^{\hat{L}}_G(\hat{q})\\
&=-S(\delta \mathbf{\theta})C^{\hat{L}}_G(\hat{\mathbf{q}})\\
\end{align}$$

Therefore,

$$\begin{equation}
\tilde{z} = \left[\begin{array}{c}
-S(\delta \mathbf{\theta}) C^{\hat{L}}_G(\hat{q})\mathbf{g} + \mathbf{b}^a + \mathbf{v}^a\\
-S(\delta\mathbf{\theta})C^{\hat{L}}_G(\hat{\mathbf{q}})\mathbf{h} + \mathbf{b}^m + \mathbf{v}^m
\end{array}\right]
\end{equation}$$

Taking advantage of the anti-commutativiy of the skew symmetric matrix $S(\cdot)$

$$\begin{equation}
\tilde{z} = \left[\begin{array}{c}
S(C^{\hat{L}}_G(\hat{\mathbf{q}})\mathbf{g})\delta\mathbf{\theta} + \mathbf{b}^a + \mathbf{v}^a\\
S(C^{\hat{L}}_G(\hat{\mathbf{q}})\mathbf{h})\delta\mathbf{\theta}+ \mathbf{b}^m + \mathbf{v}^m
\end{array}\right]
\end{equation}$$

Which finally gives us:

$$\begin{equation}
\tilde{z} = 
\left[\begin{array}{cc}
S(C^{\hat{L}}_G(\hat{\mathbf{q}}))\mathbf{g} & 0\\
S(C^{\hat{L}}_G(\hat{\mathbf{q}}))\mathbf{h} & 0
\end{array}\right]
\left[\begin{array}{c}
\delta\mathbf{\theta}\\
\tilde{\mathbf{b}}
\end{array}\right]
+ \mathbf{b}^* + \mathbf{v}^*
\end{equation}$$

Where $\mathbf{b}^*= [\mathbf{b}^a \; \mathbf{b}^m]^T$

and   $\mathbf{v}^*= [\mathbf{v}^a \; \mathbf{v}^m]^T$

[^1]: Indirect Kalman Filter for 3D Attitude Estimateoin. A Tutorial for Quaternion Algebra. *Nikolas Trawny and Stergios I. Roumeliotis*.

[^2]: Quaternion-Based Extended Kalman Filter for Determining Orientation by Inertial and Magnetic Sensing. Angelo M. Sabatini. 

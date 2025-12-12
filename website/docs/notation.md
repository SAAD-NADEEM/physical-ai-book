---
title: Mathematical Notation
description: Definition of mathematical notation used throughout the course
keywords: [notation, mathematics, robotics, ai]
sidebar_position: 4
---

# Mathematical Notation

This document defines the mathematical notation used throughout the Physical AI and Humanoid Robotics course.

## General Notation

- $ \mathbb{R} $: Set of real numbers
- $ \mathbb{R}^n $: n-dimensional real vector space
- $ \mathbf{x} $: Vector quantity (bold lowercase)
- $ \mathbf{A} $: Matrix quantity (bold uppercase)
- $ x_i $: i-th component of vector $ \mathbf{x} $
- $ A_{ij} $: Element in row i, column j of matrix $ \mathbf{A} $
- $ \mathbf{I} $: Identity matrix
- $ \mathbf{0} $: Zero vector or zero matrix (context-dependent)
- $ \mathbf{1} $: Vector of ones
- $ \top $: Matrix transpose

## Vector and Matrix Operations

- $ \mathbf{a} \cdot \mathbf{b} $: Dot product of vectors $ \mathbf{a} $ and $ \mathbf{b} $
- $ \mathbf{a} \times \mathbf{b} $: Cross product of vectors $ \mathbf{a} $ and $ \mathbf{b} $
- $ \| \mathbf{x} \| $: Euclidean norm (2-norm) of vector $ \mathbf{x} $
- $ \| \mathbf{x} \|_1 $: L1 norm of vector $ \mathbf{x} $
- $ \text{tr}(\mathbf{A}) $: Trace of matrix $ \mathbf{A} $
- $ \det(\mathbf{A}) $: Determinant of matrix $ \mathbf{A} $
- $ \mathbf{A}^{-1} $: Inverse of matrix $ \mathbf{A} $
- $ \mathbf{A}^\dagger $: Pseudo-inverse of matrix $ \mathbf{A} $

## Kinematics and Dynamics

- A_p_B: Position vector of point B expressed in frame A
- A_R_B: Rotation matrix from frame B to frame A
- A_T_B: Homogeneous transformation matrix from frame B to frame A
- J: Jacobian matrix
- q_dot: Time derivative of joint angles q
- q_ddot: Second time derivative of joint angles q
- omega: Angular velocity vector
- v: Linear velocity vector
- M(q): Mass/inertia matrix
- C(q, q_dot): Coriolis and centrifugal forces matrix
- g(q): Gravity forces vector
- tau: Joint torque/force vector

## Probabilities and Statistics

- P(A): Probability of event A
- P(A | B): Conditional probability of A given B
- p(x): Probability density function of random variable x
- mu: Mean of a distribution
- Sigma: Covariance matrix
- N(mu, Sigma): Multivariate normal distribution with mean mu and covariance Sigma
- E[.]: Expected value operator
- Var[.]: Variance operator

## Calculus and Optimization

- grad_f: Gradient of function f
- grad2_f: Hessian matrix of function f
- partial_f_partial_x: Partial derivative of f with respect to x
- d_f_dt: Total derivative of f with respect to t
- integral_dx: Integral with respect to x
- argmin_x f(x): Value of x that minimizes f(x)
- argmax_x f(x): Value of x that maximizes f(x)

## Control Theory

- s: Laplace variable
- G(s): Transfer function
- zeta: Damping ratio
- omega_n: Natural frequency
- A, B, C, D: State-space matrices
- x: State vector
- u: Control input vector
- y: Output vector
- x_dot: Time derivative of state vector
- K: Feedback gain matrix
- L: Observer gain matrix

## Robotics-Specific Notation

- SE(3): Special Euclidean group in 3D (rigid body motions)
- SO(3): Special Orthogonal group in 3D (rotations)
- se(3): Lie algebra of SE(3)
- so(3): Lie algebra of SO(3)
- xi: Twist vector (in se(3))
- Ad_T: Adjoint transformation matrix
- ad_xi: Adjoint operator
- exp(.): Matrix exponential
- log(.): Matrix logarithm

## Sets and Logic

- in: "is an element of"
- subset: Subset relation
- union: Union of sets
- intersection: Intersection of sets
- not: Logical NOT
- and: Logical AND
- or: Logical OR
- implies: Logical implication
- iff: Logical equivalence

## Conventions

1. Coordinate frames are denoted with superscripts (e.g., A_p_B represents the position of point B in frame A)
2. Vectors representing spatial quantities (positions, velocities, forces) are column vectors by default
3. All angles are expressed in radians unless otherwise specified
4. Time derivatives are denoted with dots (e.g., x_dot) or with d/dt notation
5. Unit vectors are denoted with hats (e.g., x_hat), though this is used sparingly
# Curve Optimization

The aim of this folder is to support the optimization of a TCurve to match a
sequence of timed poses. The way that we do this is by creating an Error Model which penalizes deviation of the trajectory from a given sequence of timed poses.


## Error Model

The error model is the sum of the errors at a series of observed times $t_0, t_1, ..., t_n$. The error is given by:

$$

$$

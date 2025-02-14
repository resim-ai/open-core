# Curve Optimization

The aim of this folder is to support the optimization of a TCurve to match a
sequence of timed poses. The way that we do this is by creating an Error Model which penalizes deviation of the trajectory from a given sequence of timed poses.


## Error Model

The error model is the sum of the errors at a series of observed times $t_0, t_1, ..., t_n$. At these times, we've observed poses $p_0, p_1, ..., p_n$. Let's say that the curve $\gamma$ is defined by parameters $X$. The error is given by:

$$
E(X) = \sum_{i=0}^N \log(p_i \gamma(X, t_i))
$$

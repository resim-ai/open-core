# Curve Optimization

The aim of this folder is to support the optimization of a TCurve to match a
sequence of timed poses. The way that we do this is by creating an Error Model which penalizes deviation of the trajectory from a given sequence of timed poses.


## Error Model

The error model is the collection of the errors at a series of observed times $t_0, t_1, ..., t_n$. At these times, we've observed poses $p_0, p_1, ..., p_n$. Let's say that the curve $\gamma$ is defined by parameters $X$. The error is given by:

$$
E_i(X) = \log(p_i \gamma(X, t_i)^{-1})
$$

As we know from [Ethan Eade](https://ethaneade.com/optimization.pdf), the Jacobian we want for each $i$ is:

$$
J = \frac{d \gamma(X, t_i)}{dX}
$$

Which we compute in the `//resim/curves/optimization:t_curve_differential` library once we've selected the relevant previous and next points.

We're going to be tracking a param

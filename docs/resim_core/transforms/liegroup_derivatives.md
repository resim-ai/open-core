# Lie Group Derivatives

## Introduction

This document provides some mathematical background on Lie groups and Lie
algebras. In particular, this document covers how differentiation works on Lie
groups, the Lie group exponential, left and right tangent spaces, the adjoint
representations of a Lie group and its algebra, and the chain rule for Lie groups.

## Differentiating Curves on Lie Groups

By definition, a Lie group is any differentiable manifold which is also a
group. A differentiable manifold is not generally
[diffeomorphic](https://en.wikipedia.org/wiki/Diffeomorphism) to Euclidean space
(e.g. $\mathbb{R}^N$), but it is locally diffeomorphic to Euclidean space in the
sense that we can define a diffeomorphism $\phi_p$ with $\mathbb{R}^N$ in the neighborhood of any
point $p$ in our group $G$. We don't know how to take derivatives directly on
the group $G$, but we *do* know how to do this on Euclidean space. So we define
the derivative with respect to a **particular** choice of $\phi_p$ in the
neighborhood of the point $p$ where we're taking the derivative. For a
trajectory $g:\mathbb{R}\rightarrow G$ with $g(t) = p$, this looks like:

$$
\frac{d^{\phi_p}g(t)}{dt} \equiv \frac{d}{dt}(\phi_p \circ g)(t)
$$

Since $\phi_p \circ g$ is just a function from $\mathbb{R}$ to $\mathbb{R}^N$,
we can take its derivative in the standard way, and the typical properties of
derivatives (e.g. the chain rule) all apply. We just need to come up with a
consistent way of picking $\phi_p$ for whatever point $p$ we're at. For a Lie
group, one way we can do this is to first define $\phi_e$ for the identity
element $e\in G$ and then we can define $\phi_p$ in the neighborhood of any
point $p$, since Lie group elements are guaranteed to have an inverse. We can do
this like so:

$$
\phi_p(g) = \phi_e(p^{-1} g)
$$

So our derivative at a point $p = g(t)$ along our trajectory is given by:

$$
\frac{d^{\phi_p}g(t)}{dt} = \frac{d}{d\epsilon} \phi_e (g(t)^{-1} g (t + \epsilon))
$$

This happens to be a nice choice for $\phi_p$ for each point $p$ because it has
a property called left invariance. This means that the derivative is the same
for a curve that is left multiplied by a **constant** element of $G$. Say that
we're differentiating $f(t) = hg(t)$ for constant $h\in G$ and
$g:\mathbb{R}\rightarrow G$. In this case, $p = f(t) = h g(t)$ and we have:

$$
\begin{aligned}
\frac{d^{\phi_p} f}{dt} &= \frac{d}{dt} \phi_p(h g(t)) \\
&= \frac{d}{dt} \phi_e(p^{-1} h g(t)) \\
&= \frac{d}{d\epsilon} \phi_e((g(t)^{-1} h^{-1}) h g(t + \epsilon)) \\
&= \frac{d}{d\epsilon} \phi_e(g(t)^{-1} g(t + \epsilon)) \\
&= \frac{d^{\phi_{g(t)}} g}{dt}
\end{aligned}
$$

Which demonstrates the left invariance.

## Lie Group Exponential

Now, let's define a curve $\gamma: \mathbb{R} \rightarrow G$ with the properties
that $(d^{\phi_p}\gamma / dt)$ as defined above is **constant** at all points
along the curve and that $\gamma(0) = e$. We claim without proof that it's
possible to uniquely define such a curve. If we define $\gamma_2(t) = g^{-1}
\gamma(t)$ for some $g$ in the range of $\gamma$, then we can conclude that
$\gamma_2(t) = \gamma(t - \gamma^{-1}(p))$ because $\gamma_2$ also goes through
the origin (at $t = \gamma^{-1}(p)$) and has the same constant value for
$(d^{\phi_p} \gamma_2/dt)$. Therefore, the curves have the same range, and
$\gamma_2(0) = p^{-1}$ is also a member of the range of $\gamma$. We can do this
for any $p$ in the range of $\gamma$. We can conclude that the range
of $\gamma$ is a subgroup of $G$ because it contains the identity, every element
in the subgroup has an inverse, and it is closed under composition with other
members of the subgroup. This is called a **one parameter subgroup** of $G$.

Now, we can go further by defining the *Lie Group Exponential* as the function
$\exp: \mathbb{R}^N \rightarrow G$ satisfying $\exp(X) = \gamma(1)$ where
$\gamma$ is defined as above with $X = (d^{\phi_p}\gamma /dt)$. In other terms,
the exponential tells you where you end up if you follow the same constant
derivative for one unit of time through the Lie group. The standard exponential
on real numbers falls out of this definition if you consider curves with
constant derivatives that intersect the multiplicative identity:

$$
\begin{aligned}
\frac{d^{\phi_p}y}{dt} &= \frac{d}{d\epsilon} \left[y(t)^{-1} y(t + \epsilon)\right] = k \\
y(0) &= 1 \\
\end{aligned}
$$

With some rearrangement and variable substitution:

$$
\begin{aligned}
y'(t) &= ky \\
y(0) &= 1 \\
\end{aligned}
$$

which has a known solution $y = \exp(t)$. Therefore, the Lie group exponential
can be interpreted as a natural extension of the familiar concept of the
exponential. In fact, for matrix Lie groups (Lie groups which have a matrix
representation), the Lie group exponential is identical to the matrix
exponential. Furthermore, just as the ordinary exponential has an inverse, the
logarithm, there is a Lie group logarithm, which is the inverse of the Lie group
exponential. Conceptually, the logarithm tells you "what constant velocity would
I have to move at to arrive at this point in one unit of time from the origin."

It's worth noting that $\mathbb{R}^N$ is not formally the input to the
exponential. The domain of the exponential is actually called the **Lie
Algebra** $\mathfrak{g}$ corresponding to the Lie group $G$. It is a vector
space and hence it is homeomorphic to $\mathbb{R}^N$, so we casually treat them
as interchangeable.

The exponential and logarithm define a correspondence between the Lie group and
a vector space $\mathbb{R}^N$ called the Lie algebra. This connection allows us
to leverage much of the mathematics derived in vector spaces on Lie groups which
can be incredibly powerful. For instance, one can define a Gaussian probability
distribution on the Lie algebra $\mathfrak{so}(3)$ and take the exponential of
samples from it to sample orientations in the Lie group $\text{SO(3)}$. Because
they are commonly useful, we provide implementations of the exponential and
logarithm for each Lie group we implement.

## Left and Right Tangents

So far, we have been working with the left invariant definition for the
derivative:

$$
\frac{d^{\phi_p}g(t)}{dt} = \frac{d}{d\epsilon} \phi_e (g(t)^{-1} g (t + \epsilon))
$$

However, we've been very vague about what we may pick for $\phi_e$. We know
it needs to be a diffeomorphism defined in an open set around $e$. It turns out
that the Lie group logarithm is a reasonable choice for this, so this is what we
use. Hence:

$$
\phi_p(g) = \log(p^{-1} g)
$$

As noted before, this definition is left invariant since we can multiply on the
left by any member of the group without changing the value of the
derivative. This is practically quite useful because if $g$ represents a
transform of `world_from_robot` as a function of time, it doesn't actually
matter where we decide the "world" frame is when we're computing derivatives as
long as our choice does not move relative to some agreed-upon world frame
(remember that $h$ must be constant for the above to work. Confusingly, although
this choice for $\phi_p$ yields *left* invariance, $(d^{\phi_p}g/dt)$ is often
referred to as the "right tangent" or "right tangent space derivative" of
$g$. This is because it can also be defined in terms of the *right* perturbation
to $g$ that the trajectory is "following" at that moment.

$$
g(t + \epsilon) = g(t) \exp\left[\frac{d^{\phi_p}g}{dt} \epsilon\right]
$$

which when rearranged gives:

$$
\frac{d^{\phi_p}g}{dt} = \frac{\log\left[g(t)^{-1} g(t + \epsilon
)\right]}{\epsilon}
$$

In the limit as $\epsilon \rightarrow 0$, we get:

$$
\frac{d^{\phi_p}g}{dt} = \frac{d}{d\epsilon} \log(g(t)^{-1}
g(t + \epsilon))
$$

which is equivalent to what we had before. For this reason, we henceforth refer
to this definition of $\phi_p$ as $\phi_p^R$. The reason for the special
notation, is that we could equally well have chosen:

$$
\phi_p(g) = \phi_p^L(g) = \log(g p^{-1})
$$

As before, $g p^{-1}$ is in the neighborhood of the identity $e$ when $g$ is in
the neighborhood of $p$, so this works. With this, we get:

$$
\begin{aligned}
\frac{d^{\phi_p^L} g}{dt} &= \frac{d}{dt} \log(gp^{-1}) \\
&= \frac{d}{d\epsilon} \log(g(t + \epsilon) g(t)^{-1})
\end{aligned}
$$

$(d^{\phi_p}g/dt)$ is often referred to as the "left tangent" or "left tangent
space derivative" of $g$, and one can verify that it can be defined in terms of
a left perturbation to $g$ that the trajectory is following and that it has a
**right** invariance property such that multiplying the trajectory on the
**right** by a constant element of the group does not affect its value.

In summary, we have left and right tangent space derivatives that have right and
left invariance respectively. 

## The Adjoint

In practice both the left and right tangent space derivatives happen to be
useful in particular cases, so one might ask if it's possible to easily convert
between the right tangent of a trajectory and the left tangent of the
trajectory. If we want the left tangent space derivative, we can see:

$$
\begin{aligned}
\frac{d^{\phi_p^L}g}{dt} &= \frac{d}{dt}\log(g p^{-1}) \\
&= \frac{d}{dt}\log((p p^{-1}) g p^{-1}) \\
&= \frac{d}{dt}\log(p (p^{-1} g) p^{-1}) \\
&= \frac{d}{dt}\log(p \exp[\log(p^{-1} g)] p^{-1}) \\
 \end{aligned}
$$

Applying the chain rule to $h(k(t))$ where $k(t) = \log(p^{-1} g)$ and $h(k) =
\log(p \exp(k) p^{-1})$ are both functions to and from Euclidean space, we have:

$$
\begin{aligned}
\frac{d^{\phi_p^L}g}{dt} &= \left[\frac{d}{dk} \log(p \exp(k) p^{-1})\right] \frac{d^{\phi_p^R}g}{dt}
\end{aligned}
$$

so we can convert from right to left tangent space by multiplying by this
Jacobian matrix:

$$
\text{Ad}_g \equiv \left[\frac{d}{dk} \log(g \exp(k) g^{-1})\right] 
$$

where we define this Jacobian to be the adjoint representation of the element
$g\in G$. Technically, the adjoint representation is the map that produces such
matrices given inputs from $G$. This map can also be defined as the derivative
of a curve $f(g) = p g p^{-1}$ at the identity. Since this is a map from $G$ to
$G$, one can show this by assuming that $g$ is a curve passing through the
identity and using the chain rule as above to find what $(df/dg) = \text{Ad}_p$
is. There are a number of properties that the adjoint has that are worth noting:


$$
\text{Ad}_{g^{-1}} = \text{Ad}_g^{-1}
$$

$$
\text{Ad}_{gh} = \text{Ad}_{g} \text{Ad}_{h}
$$

which is basically just a reminder that the adjoint is a **representation** of
G. In the case where the Lie group has a matrix representation (which is true
for all the Lie groups we use), one can simplify our definition to be:

$$
\text{Ad}_g X = g X g^{-1};\quad\quad g\in G,\,\,X\in \mathfrak{g}
$$

for any $X$ in the Lie Algebra $\mathfrak{g}$ of $G$. Note that this is a matrix
representation of the Lie algebra element, not just a vector in
$\mathbb{R}^N$. Expressions for $\text{Ad}_g$ are derived for all the Lie groups
we implement since it is so commonly needed.

It's often the case that we need to take the derivative of $\text{Ad}_g$ with
respect to time. We define the adjoint representation of the algebra
$\text{ad}_X$ based on a curve $g(t)$ going through the identity.

$$
\left.\frac{d \text{Ad}_g}{dt}\right|_{g = e} = \text{ad}_X;\quad\quad X =
\left.\frac{dg}{dt}\right|_{g = e}
$$

It doesn't matter whether we use the right or left tangent space derivative here
for $g$ since they are equivalent at the identity, as one can verify by
inspecting their definitions. The algebra adjoint is related to the Lie bracket:

$$
\text{ad}_X Y = [X, Y]
$$

For matrix Lie groups, the bracket is the commutator on the matrix
representation of algebra elements.

$$
\text{ad}_X Y = [X, Y] = XY - YX
$$

The algebra adjoint is somewhat commonly used, so we provide it as a static
member function in our Lie group objects.

## The Chain Rule

Let's look at how one might differentiate the composition of two Lie group
elements in right tangent space, as an example. In other words, take the time
derivative of $f(t) = g(t)h(t)$:

$$
\begin{aligned}
\frac{d^Rf}{dt} &= \frac{d^R}{dt} [g(t)h(t)] \\
&= \frac{d}{d \epsilon} \log [ h(t)^{-1} g(t)^{-1} g(t + \epsilon) h(t + \epsilon)]
\end{aligned}
$$

To help ourselves, let's define a function $c(\delta_1, \delta_2)$ like so:

$$
c(\delta_1, \delta_2) = \log [ h(t)^{-1} g(t)^{-1} g(t + \delta_1) h(t + \delta_2)]
$$

where $\delta_1$ and $\delta_2$ are functions of $\epsilon$. Taking the
derivative with respect to $\epsilon$ with the multivariable chain rule gives:

$$
\frac{d}{d\epsilon}c(\delta_1, \delta_2) =
\text{Ad}_{h^{-1}} \frac{d^R g}{d t} \frac{d\delta_1}{d\epsilon} + \frac{d^R h}{dt} \frac{d\delta_2}{d\epsilon}
$$

Of course, the derivative of $c$ is useful to us if $\delta_1 = \delta_2 =
\epsilon$ so:

$$
\frac{d^R f}{dt} = \text{Ad}_{h^{-1}} \frac{d^R g}{dt} + \frac{d^R h}{dt}
$$

which is the chain rule in the right tangent space. There is also a chain rule
for the left tangent space:

$$
\frac{d^L f}{dt} = \frac{d^L g}{dt} + \text{Ad}_g \frac{d^L h}{dt}
$$

One can verify the left and right invariance properties using these expressions
and assuming one of the group elements is constant.

---
layout: post
title: But What is the Kalman Filter?
custom_excerpt: > 

notes: >
    Flow:
    - Precusory Knowledge
        X State space models
        X The Solution to the First Order ODEs
        - Modern controls
        - Full state feedback
        - LQR
    - Observers
        - Why
        - How -- Luenberger
        - Dynamics (just another control system)
    - Noise & Quantisation
    - Estimation
    - Prediction
        - Uncertainty -- How it modifies the reliance on model and measurement
    - Update
    - LQG 
        - Gotta mention the goat paper "LQG margins -- Doyle"
    - Usage in systems other than control
        - Rader and all
    - Other Kalman Filters (Overview)
        - Extended
        - Unscented 
---

## Introduction

As someone doing robotics projects, the term "Kalman Filter" is definitely a familar one. It's generally used anytime you need to extract some kind of information from sensors. However, at least in the last few years I see the controls folks getting AI to write the filter code, and the software folks using it through SLAM without actually understanding what it is. What are Q and R? Where do we use a linear kalman? An extended kalman? And what on Earth in an unscented Kalman? The first time I read about it, I thought Bed Bath and Bodyworks made a new line of candles aimed towards engineers. 

But fear not, dear reader. This blog is to demystify the concept, building up from the basic concepts in a *hopefully* intuitive form. The assumed knowledge will be basic linear algebra, linear systems, the concept of feedback, and some basic classical controls. If you're familar with full-state control feel free to skip forward, but to cater to the software folks -- or anyone who doesn't know what those words mean --  we need to start at a more base level.

## Precursory knowledge

This section might feel a little fast. If you want to go more in detail, I would recommend a lovely set of lectures by Dr. Steve Brunton, which can be found [here](https://youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m). In fact, he goes into what we're doing today and *more*.

### Modelling a linear system in state space
With that out of the way, let's do a quick recap of linear systems.

The basic idea of the linear system is that its dynamics can be modelled with a linear differential equation. That is, any equation where the state and its derivatives appear only linearly (multiplied by constants or functions of time, but not by themselves). The highest order of the differential term is the order of the equation. 

For example, a classic mass-spring-damper equation may look like this:
$$ M\ddot{x} + B\dot{x} + Kx = f_{in}$$
This equation is of second order, since we have an $\ddot{x}$. 

In classical controls, we generally take the laplace transform to convert things to the s-domain, giving us (assuming all initial conditions to be 0):
$$Ms^2 + Bs + K = F_{in}, \ F_{in} = \mathscr{L}(f_{in})$$

Now this is all well and good, for when we have a single input, single output (SISO) system. If we have something more complicated, like a mulitple input, multiple output (MIMO) system, we need to adjust. 

For example, let's say we have both $x$ and $y$ values. Now, we have:
$$ 
M\ddot{x} + B_x\dot{x} + K_xx = f_{x}\\
M\ddot{y} + B_y\dot{y} + K_yy = f_{y}
$$
> Note that the mass remains constant whether it's x or y.

Notice that we have two second order differential equations now. Those are... annoying. To solve it, we have to find the complimentary and particular solution. If it were first order equations, the solution would be very simple. So let's transform the higher order equation to a set of first order ones. 

Let's define some new variables. Particularly, $v_x$, and $v_y$. They are $\.x$ and $\.y$ respectively. This gives us two equations already:
$$
\.x = v_x\\
\.y = v_y
$$

At this point, it's a good moment to introduce the concept of a *state*. In control theory, a state is a set of variables that fully describe the system at any given instant. For our 2D mass-spring-damper, the natural states are the positions and velocities in each direction:

$$
X = \begin{bmatrix} x \\ v_x \\ y \\ v_y \end{bmatrix}
$$

With these four states, knowing $X$ at any time allows us to predict all future behavior of the system (assuming we know the inputs $f_x$ and $f_y$).  

> Notice that $\.v_x = \ddot{x}$, and same for y

Subbing that into our old equations, we get:
$$ 
M\.v_x + B_xv_x + K_xx = f_{x}\\
M\.v_y + B_yv_y + K_yy = f_{y}
$$
Rearranging and recalling the other two equations:
$$ 
\begin{aligned}
\.v_x &= -\frac{B_x}{M}v_x - \frac{K_x}{M}x + \frac{1}{M}f_{x}\\
\.v_y &= -\frac{B_y}{M}v_y - \frac{K_y}{M}y + \frac{1}{M}f_{y} \\
\.x &= v_x \\
\.y &= v_y
\end{aligned}
$$

Finally, putting it into matrix form gives us:

$$
\begin{bmatrix}
\dot{x} \\ \dot{v}_x \\ \dot{y} \\ \dot{v}_y
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
-\frac{K_x}{M} & -\frac{B_x}{M} & 0 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & -\frac{K_y}{M} & -\frac{B_y}{M}
\end{bmatrix}
\begin{bmatrix}
x \\ v_x \\ y \\ v_y
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 \\
\frac{1}{M} & 0 \\
0 & 0 \\
0 & \frac{1}{M}
\end{bmatrix}
\begin{bmatrix}
f_x \\ f_y
\end{bmatrix}
$$

Or in more general notation:
$$
\dot{X}=AX+BU
$$
where $X$ is the vector containing all the states, $\dot{X}$ is the vector of the derivatives of the states, and $U$ is the vector of inputs to the system.

$A$ is the **state transition matrix**, which determines how the current state influences the rate of change of the state.

$B$ is the **input matrix**, which determines how the system inputs affect the rate of change of the state.

Moving forward, we will be using arbitrary states and matrices, so understanding what each of these are is important. Take some time to really internalise their significance.

### The Solution to the First Order ODEs

Let's simplify our model even further. Let's remove the external inputs and just see how the system would behave by itself.

$$
\dot{X}=AX
$$

This is a classic setup, and what I meant when I said that first order differential equations are very easy to solve. In fact, you probably remember this from class 12 math:

$$
X = C_0e^{At}
$$

Now recall that $A$ is a matrix. How on earth does taking something to the power of a matrix make any sense? Well, we can use the taylor definition of the $e^x$:

$$
e^{At} = \sum_{n=0}^{\infty}\frac{(At)^n}{n!} = \sum_{n=0}^{\infty}\frac{A^nt^n}{n!}
$$

Now, this doesn't look very easy, but at least it exists!

However, we don't know if this series converges or diverges. Taking such a high power (infinity is a rather large number) is hard. If only there was a quick rule, like having the poles in the left hand side of the s-plane... 

With all that build up, you can probably guess that there is a quick rule -- and it's actually pretty ingenious. You need to diagonalise the matrix. In particular, set $A = PDP^{-1}$.

> Note that this is possible as $A$ is guaranteed to be square. Also, $A$ does *not* need to be invertible to be diagonalised. 

Now if we take $A$ to some power $n$:
$$
\begin{aligned}
A^n &= (PDP^{-1})^n \\
    &= PD\cancel{P}^{-1}\cancel{P}D\cancel{P}^{-1}\cdots\cancel{P}^{-1}\cancel{P}DP^{-1} \\
    &= PDD\cdots DP^{-1} \\
    &= PD^nP^{-1}
\end{aligned}
$$

Therefor
$$
\sum_{n=0}^{\infty}\frac{A^nt^n}{n!} = P\sum_{n=0}^{\infty}\left(\frac{D^nt^n}{n!}\right)P^{-1}
$$

Recall that matrix $D$ is a diagonal matrix with it's diagonal entries being the eigenvalues of $A$. Also recall that multiplying two diagonal matrices is just like multiplying each corresponding element. Therefor:

$$
D^n =
\begin{bmatrix}
\lambda_1^n & 0 & \cdots & 0 \\
0 & \lambda_2^n & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & \lambda_m^n
\end{bmatrix}
$$

Also notice that:
$$
\sum_{n=0}^{\infty}\frac{D^nt^n}{n!} = 
\begin{bmatrix}
e^{\lambda_1t} & 0 & \cdots & 0 \\
0 & e^{\lambda_2t} & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & e^{\lambda_mt}
\end{bmatrix}
$$

Notice how diagonalizing $A$ reduces the problem to independent scalar exponentials. Each eigenvalue evolves separately, which is why stability is easy to read off.

It's pretty easy to see that the matrix will blow up to infinity for $\lambda > 0$ for $t \rightarrow\infty$, and it'll go to 0 for $\lambda < 0$ for $t \rightarrow\infty$. 

This is the analog we were looking for. The eigenvalues of the A matrix give us the **poles** of the system. If they're negative, the system is stable. If they're positive, the system is unstable. It's just like classical controls. 

$$
\begin{array}{c|c}
\textbf{Eigenvalue } \lambda_i & \textbf{Behavior as } t \to \infty \\
\hline
\lambda_i < 0 & \text{Decays to 0 (stable)} \\
\lambda_i = 0 & \text{Remains constant (marginally stable)} \\
\lambda_i > 0 & \text{Grows to } \infty \text{ (unstable)}
\end{array}
$$

### Modern Controls

Modelling is all fine and dandy, but how exactly does this factor into controls? Also, how can we remove the external inputs from the model? That's what we need to use to control the system!

The big problem we need to solve is to figure out what inputs we need to achieve a particular set of states. For simplicity, let's assume we want to drive all the states to 0.

Recall the concept of feedback, where we use a sensor to figure out where our system actually is, then use that as an input to our controller to achieve some desired value.

In MIMO systems, it's exactly the same. Instead of just one sensor, we use multiple to get an idea of the entire state (ideally), and then put that into our controller to achieve some specified state. In the case of our mass-spring-dampner system with position and velocity in x/y states, we could use a GNSS sensor for x,y position, and some kind of velocity sensor (like a tachometer) for the velocities.

The sensed values are put into a controller, which computes the required inputs. This is called **full-state control**. Let's analyse it with some math.

### Full state control

Let's assume: 
$$U = -KX$$
Where $K$ is some gain matrix that we multiply the current state with.

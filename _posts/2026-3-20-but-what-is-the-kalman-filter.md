---
layout: post
title: But What is the Kalman Filter?
custom_excerpt: > 
    If you’ve ever worked with sensors or control systems, you’ve probably seen the Kalman filter pop up everywhere. From robotics and tracking to signal processing and navigation. In this post, I break down what the Kalman filter actually is, starting from basic linear systems and feedback control, and building up to estimation under noise. Along the way, we look at how uncertainty shapes everything: from Luenberger observers to LQG, and why tools like the EKF and UKF exist in the first place.

biblio: >
    1. https://youtu.be/Fj4xO0K19fE

notes: >
    Flow:
    x Precusory Knowledge
        X State space models
        X The Solution to the First Order ODEs
        X Modern controls
        X Full state feedback
        X LQR
    x Observers
        x Why
        x How -- Luenberger
        x Dynamics (just another control system)
    x Noise & Quantisation
    x The Kalman Filter
        x Prediction
        x Estimation
            x Uncertainty -- How it modifies the reliance on model and measurement
        x Update
    x LQG 
        x placement of poles
        x Gotta mention the goat paper "LQG margins -- Doyle"
    x Usage in systems other than control
        x Rader and all
    x Other Kalman Filters (Overview)
        x Extended
        x Unscented 
---

## Table of Contents

1. [Introduction](#introduction)  
2. [Precursory Knowledge](#precursory-knowledge)  
   - [Modelling a Linear System in State Space](#modelling-a-linear-system-in-state-space)  
   - [The Solution to First Order ODEs](#the-solution-to-first-order-odes)  
   - [Modern Controls](#modern-controls)  
   - [Full State Control](#full-state-control)  
   - [LQR](#lqr)  
3. [Observers](#observers)
4. [The Kalman Filter](#the-kalman-filter)
5. [LQG](#lqg)  
6. [Kalman Filters Beyond Control](#kalman-filters-beyond-control)  
7. [Beyond the Kalman Filter](#beyond-the-kalman-filter)
8. [Conclusion](#conclusion)  

## Introduction

As someone doing robotics projects, the term "Kalman Filter" is definitely a familiar one. It's generally used anytime you need to extract some kind of information from sensors. However, at least in the last few years I see the controls folks getting AI to write the filter code, and the software folks using it through SLAM without actually understanding what it is. What are Q and R? Where do we use a linear kalman? An extended kalman? And what on Earth in an unscented Kalman? The first time I read about it, I thought Bed Bath and Bodyworks made a new line of candles aimed towards engineers. 

But fear not, dear reader. This blog is to demystify the concept, building up from the basic concepts in a *hopefully* intuitive form. The assumed knowledge will be basic linear algebra, linear systems, the concept of feedback, and some basic classical controls. If you're familiar with full-state control feel free to skip forward, but to cater to the software folks -- or anyone who doesn't know what those words mean --  we need to start at a more base level.

## Precursory knowledge

This section might feel a little fast. If you want to go more in detail, I would recommend a lovely set of lectures by Dr. Steve Brunton, which can be found [here](https://youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m). In fact, he goes into what we're doing today and *more*.

### Modelling a linear system in state space
With that out of the way, let's do a quick recap of linear systems.

The basic idea of the linear system is that its dynamics can be modelled with a linear differential equation. That is, any equation where the state and its derivatives appear only linearly (multiplied by constants or functions of time, but not by themselves). The highest order of the differential term is the order of the equation. 

For example, a classic mass-spring-damper equation may look like this:

$$ M\ddot{x} + B\dot{x} + Kx = f_{in}$$

This equation is of second order, since we have an $\ddot{x}$. 

In classical controls, we generally take the laplace transform to convert things to the s-domain, giving us (assuming all initial conditions to be 0):

$$Ms^2 + Bs + K = F_{in}, \ F_{in} = \mathcal{L}(f_{in})$$

Now this is all well and good, for when we have a single input, single output (SISO) system. If we have something more complicated, like a multiple input, multiple output (MIMO) system, we need to adjust. 

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

Therefore:

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

> Note that eigenvalues that are more negative decay to 0 faster.

### Modern Controls

Modelling is all fine and dandy, but how exactly does this factor into controls? Also, how can we remove the external inputs from the model? That's what we need to use to control the system!

The big problem we need to solve is to figure out what inputs we need to achieve a particular set of states. For simplicity, let's assume we want to drive all the states to 0.

Recall the concept of feedback, where we use a sensor to figure out where our system actually is, then use that as an input to our controller to achieve some desired value.

In MIMO systems, it's exactly the same. Instead of just one sensor, we use multiple to get an idea of the entire state (ideally), and then put that into our controller to achieve some specified state. In the case of our mass-spring-dampner system with position and velocity in x/y states, we could use a GNSS sensor for x,y position, and some kind of velocity sensor (like a tachometer) for the velocities.

The sensed values are put into a controller, which computes the required inputs. This is called **full-state control**. Let's analyse it with some math.

### Full state control

Let's assume: 

$$U = -KX$$

Where $K$ is some gain matrix that we multiply the current state with. Let's just work through the maths, and the reason for this structure will make sense.

$$
\begin{aligned}
\dot{X} &= AX + BU \\
\dot{X} &= AX + B(-KX) \\
\dot{X} &= AX - BKX \\
\dot{X} &= (A-BK)X
\end{aligned}
$$

Notice that this is the exact same structure of first order ODE that we've already worked though. It follows then that similarly to the $\dot{X}=AX$ case, the eigenvalues of $A-BK$ will determine the stability of the closed loop system. The difference here though is that while $A$ is something innate to the system, we can choose any $K$ we want. 

Now the question is, what exactly can we do with $K$? The answer is *a lot*. In fact, if the pair $(A,B)$ are **controllable** (I would highly recommend watching the [Dr. Steve Brunton video about this](https://youtu.be/u5Sv7YKAkt4), as this preamble is already getting rather long), we can force the eigenvalues to be *anything*.

This is a crucial result. It means that we can take positive eigenvalues and make them negative -- turning an unstable system into a stable one -- or moving already negative poles closer or further from 0. In effect, we can make the system behave *however we want*.

This is such a common operation that Matlab has a function ```K = place(A, B, p)```, that gives you the gain matrix $K$ for any $(A,B)$ pair to place the poles, with the desired locations given as $p$.

But hang on a minute... Earlier I said that poles that are more negative will settle to 0 faster. Wouldn't we want to move those poles as far left as we could? Maybe even to $-\infty$? 

Well, there are two issues. In practice, many controllers are implemented digitally, which introduces sampling and bandwidth limitations. (a topic for another blog post: discrete controls). More relevant to us however (working in the continuous realm), is that giving inputs takes energy. The more we move the poles, the more energy they will consume. This also makes intuitive sense: gently nudging the system will take less energy than dragging it somewhere and holding it there by forcing the system to move rapidly or counteracting strong dynamics.

So we have the ability to make the system more stable, but making it more stable requires more energy? This sounds like a classic optimisation problem!

### LQR 

Ah the good-ol' **Linear Quadratic Regulator**, with its confusing name. Is it linear? Is it quadratic? What are those even refering to? Who knows? Well, dear reader, I hope you will after reading this subsection.

It's actually way more simple than it seems. 'Linear' is refering to the system dynamics. 'Quadratic' refers to the cost function we're trying to minimise. I know the AI people's ears just perked up. That's right, *cost function*. Unfortunately it's not something like least-squares, but the intuition remains the same.

Before defining the cost function, let's define the costs. The obvious one is the cost of the input. This could be cost of fuel, battery drain, whatever makes it *undesirable* to have a large input. The other one, is the state cost. It's how we quantify how important our system being settled is on a per state basis. For example, with a submersible, staying upright is very important, so those would have high state costs, while maybe something like your vertical displacement will have a lower cost.

These costs are put into our cost matrices: **Q** (state cost), and **R** (input cost). High values in $Q$ will aggressively penalise state, while high values in $R$ aggressively penalise input.

Then we combine the two costs into a cost function $J$, and try to minimise it. 

$$
J = \int_0^\infty (X^TQX + U^TRU)dt
$$

We are looking for a feedback law of the form $U=−KX$ that minimises this cost.

The quadratic comes from us multiplying $X, U$ with themselves. Let's see what $X^TQX$ looks like.

$$
\begin{aligned}
    \text{Let: } X &= 
    \begin{bmatrix}
        x_1 \\
        x_2 \\
        x_3
    \end{bmatrix} \\
    Q &=
    \begin{bmatrix}
    q_{11} & q_{12} & q_{13} \\
    q_{21} & q_{22} & q_{23} \\
    q_{31} & q_{32} & q_{33} \\ 
    \end{bmatrix} 
\end{aligned} \\ 

\begin{aligned}
    \therefore X^TQX &= 
    \begin{bmatrix}
        x_1 & x_2 & x_3
    \end{bmatrix}
    \begin{bmatrix}
        q_{11} & q_{12} & q_{13} \\
        q_{21} & q_{22} & q_{23} \\
        q_{31} & q_{32} & q_{33}
    \end{bmatrix}
    \begin{bmatrix}
        x_1 \\
        x_2 \\
        x_3
    \end{bmatrix} \\
    &=
    \begin{bmatrix}
        x_1 & x_2 & x_3
    \end{bmatrix}
    \begin{bmatrix}
        q_{11}x_1 + q_{12}x_2 + q_{13}x_3 \\
        q_{21}x_1 + q_{22}x_2 + q_{23}x_3 \\
        q_{31}x_1 + q_{32}x_2 + q_{33}x_3 
    \end{bmatrix} \\
    &= 
    q_{11}x_1^2 + q_{12}x_1x_2 + q_{13}x_1x_3 \\
    &+
    q_{21}x_1x_2 + q_{22}x_2^2 + q_{23}x_2x_3 \\
    &+
    q_{31}x_1x_3 + q_{32}x_2x_3 + q_{33}x_3^2 \\
\end{aligned}
$$

Notice that each term is of the form $q_{ij}x_ix_j$. That represents the coupling between the states, with $q$ representing how much we care about that pair. For example, if we don't care about the coupling and only the actual states matter, we can use a diagonal matrix.

$U^TRU$ gives us a similar result. As for the integration, we are essentially adding up the total cost for all of time and trying to minimise that. 

This will give us the optimal $K$ (gain) matrix, balancing state error and large inputs.

> This is done using the Riccati equation, but that's slightly beyond the scope of this (intuitive) blog post.

## Observers

Now we finally get to the interesting part. Buckle in, dear reader.

So far, we've made a *huge* assumption. 

**That we know the full state**. 

However, this is almost *never true*. We don’t measure velocity directly. Sensors are noisy. Some states aren’t measurable at all.

So what do we do now? Our entire control system was based on knowing $X$.

This is where observers come in. They *estimate* what the state is given sensor values and the model. 

Even if we can’t measure velocity directly, we can *infer* it from how position changes. Even if sensors are noisy, we can smooth things out using the model.

So instead of using the true state \(X\), we use an estimate -- $\hat{X}$ and plug that into our controller instead.

But how do we design this? It's actually far more similar to what we've been doing than you may think.

### Modelling Sensors

Since we're working with sensors now, we need a way to model the sensor output. 

Let's call the sensor output $Y$. Sensors are sensing internal state, so $Y=f(X)$. Let's model it as:

$$
Y = CX
$$

Where $C$ is some matrix. For example, if we have state $X = \left[x,v\right]^T$ and we're only measuring position, we'd get an expression something like:

$$
\begin{aligned}
Y &= CX \\
Y &=
\begin{bmatrix}
1 & 0
\end{bmatrix}
\begin{bmatrix}
x \\ v
\end{bmatrix} 
\end{aligned}
$$

Notice here that at the end, we get $Y = x$, which is what we'd expect if we're only measuring position. 

Let's look at another example. Say we have state $X = [i_m, i_t]^T$, where $v_m$ is the current draw of a microcontroller and $v_t$ is the current draw of a thruster. However, we have a current sensor for total current ($i_m + i_t$) and for the thruster only (i.e. $Y = [i_{tot}, i_t]$). No microcontroller current. Then we'd model our sensors as:

$$
Y = 
\begin{bmatrix}
i_{tot} \\ i_t
\end{bmatrix}
=
\begin{bmatrix}
1 & 1 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
i_m \\ i_t
\end{bmatrix}
$$ 

> Remember that this is without noise. We'll get to that soon.

### The Luenberger Observer 

Our objective now is given $Y$ and knowing $C$, we need to find $X$. 

You may think this is easy, after all we've all done linear algebra. All we have to do is:

$$
\begin{aligned}
Y &= CX \\
X &= C^{-1}Y
\end{aligned}
$$

Unfortunately, this assumes two things. That C is square, and that C is invertible. 

The AI folks might be thinking we can still overcome that with a least squares style setup.

$$
\begin{aligned}
Y &= CX \\
C^TY &= C^TCX \\
X &= (C^TC)^{-1}C^TY
\end{aligned}
$$

However this assumes that $C^TC$ is invertible, which is not always the case. Moreover, even if this would work, it tries to figure out the state of a dynamic system using only one measurement.

So instead of solving for $X$ in one shot, we can *build an estimate that evolves over time*, just like the real system.
Let’s call this estimate $\hat{X}$

If we compare what output our estimate would give us to the output we've actually gotten, we get a metric for how far off we are.

$$
Y-C\hat{X}
$$

This is called the **innovation**. It's the discrepancy between measured and modelled output.

> Note that if a state is not being directly sensed, it will not show up in $C$. However, if other states rely on the unmeasured state, it can still be estimated as it has an impact on the measured state. This is called *observability*, and I would once again recommend watching the Dr Steve Brunton video on this.

Now, we simulate the system, and how wrong we are. Recall that we still know what the input to the system is, so we can keep track of how the estimate should change over time.

$$
\dot{\hat{X}} = A\hat{X} + BU + L(Y-C\hat{X})
$$

Where $L$ is a matrix we choose that signifies how much the state estimate changes because of error between measured and modelled values. Large values of $L$ would correspond to the estimated states changing rapidly, i.e. we trust the measurement (low noise, since that would get amplified). If $L$ is small though, we move slowly, trusting the model more. 

You can think of that entire term as a sort of "correction term". What we want to do is drive that error to 0.

The way that we achieve this will look very familar. 

Firstly, we define a new variable, $\~{X} = X - \hat{X}$, called the estimation error. Then we differentiate and sub in the expressions we have for $X$ and $\hat{X}$.

$$
\begin{aligned}
\~{X} &= X - \hat{X} \\
\dot{\~{X}} &= \dot{X} - \dot{\hat{X}} \\
&= AX + \cancel{BU} - A\hat{X} - \cancel{BU} - L(CX-C\hat{X}) \\
&= AX - A\hat{X} - LC(X-\hat{X}) \\
\dot{\~{X}} &= (A-LC)\~{X}
\end{aligned}
$$

The same intuition as above holds here. This is just pole placement. If the eigenvalues of $A-LC$ are negative, then we have a system where $\~X \rightarrow 0$, which means that $\hat{X} \rightarrow X$, which is exactly our goal.

### Noise

Remember how I said we'd get to noise later? Well, we're here now and our entire world is gonna turn upside down. 

Instead of assuming no noise, let's see what happens to the math above if we add process ($w$) and measurement ($v$) noise.

$$
\begin{aligned}
\dot{X}&=AX+BU \rightarrow AX+BU + w \\
Y &= CX \rightarrow CX + v
\end{aligned}
$$

Therefore our estimate equation becomes:

$$
\begin{aligned}
\dot{\hat{X}} &= A\hat{X} + BU + L(Y-C\hat{X}) \\
&= A\hat{X} + BU + L(CX + v - C\hat{X}) \\
&= A\hat{X} + BU + LC(X - \hat{X}) + Lv 
\end{aligned}
$$

And when we sub it into the estimation error:

$$
\begin{aligned}
\~{X} &= X - \hat{X} \\
\dot{\~{X}} &= \dot{X}-\dot{\hat{X}} \\
&= AX + \cancel{BU} + w - A\hat{X} - \cancel{BU} - LC(X-\hat{X}) - Lv \\
\dot{\~{X}}&= (A-LC)\~{X} - Lv + w
\end{aligned}
$$

We can see two competing effects:
- The model error \(w\), which pushes us away from the true state  
- The measurement noise \(v\), which gets amplified by \(L\)

This does **not** go to 0. Even if $A-LC$ is perfectly stable.

Infact this shows us exactly we we can't make $L$ arbitrarily large. A high value of $L$ will make the system 'converge' faster, but it'll also *amplify the noise more*.

To put it in simple terms. Sometimes the process noise is stronger (i.e. the model is wrong) -- we must trust the measurements more. Sometimes the measurement noise is stronger (i.e. the measurements are wrong) -- we must trust the model more.

Sounds like the full-state to LQR transition we made earlier, right?

So, how do we choose an *optimal* $L$? My dear reader, **that's exactly what the Kalman filter does**.

## The Kalman Filter

Congratulations! You've made it through 3000 words of build up. Buckle in, because the Kalman filter is pretty *dense*.

The idea is that we want to determine how uncertain we are with our estimate $\hat{X}$. To do this, we need to quanitfy the noise we just talked about.

$$
\begin{aligned}
w &\sim \mathcal{N}(0, Q) \\
v &\sim \mathcal{N}(0, R)
\end{aligned}
$$

Where $Q$ and $R$ are how uncertain we are about the process and measurement respectively.

To measure something, we should probably name it. So let's call it $P$. A large $P$ shows that we're very uncertain about our estimate and vis-versa.

Another important piece of notation is whether we're talking about a quanity before or after incorporating the measurement. That is, whether the quantity is purely from the model, or from our mix. 

The important places you'll see this are in our two estimates: state ($\hat{X}^-$), and uncertainty ($P^-$). 

With that out of the way, our algorithm has two parts:

### Prediction
Our model's best guess of the state is:

$$
\hat{X}^- = A\hat{X} + BU
$$

But as we simulate it forward, the noise adds up. That means we get less and less certain. To measure that, we have:

$$
P^- = APA^T + Q
$$

$APA^T$ measures how uncertain we are right now, and $Q$ adds the uncertainty gained by simulating forward one step.

This means that if we keep on using our model to simulate forward, we're getting more and more uncertain.

### Update

Now we find the **Kalman gain** ($K$). It looks intimidating, but once you break it down, it's pretty intuitive.

$$
K = P^-C^T(CP^-C^T+R)^{-1}
$$

$CP^-C^T$ is the uncertainty in the predicted measurement caused by uncertainty in the state. $R$ is the uncertainty from the sensor noise. So the inside of the bracket gives us the *total measurement uncertainty*.

For $P^-C^T$ let's break it up even further. $C^T$ is telling us which states are affect the measurement. Then $P^-$ tells us the uncertainty in the states, so $P^-C^T$ tells us how much should each state respond to a measurement error, given how uncertain it is and how it affects the measurement. 

You can think of the terms as two parts.

Recall that $CP^-C^T+R$ is basically the total measurement uncertainty. That means that its inverse is a measure of how much we can trust the measurement. 

$P^-C^T$ is measurement of how much we should modify the states based on our uncertainty. 

So together, we get:

$$
K = \frac{\text{How much the noise should change the state}}{\text{How noisy the measurement is}}
$$

Now, we update the state estimate:

$$
\hat{X} = \hat{X}^- + K(Y-C\hat{X}^-)
$$

Notice that there are two cases:
1. **Measurement noise dominates**
    - $R$ is large, so the measurement is noisy
    - $K$ becomes small
    - $\hat{X} \approx \hat{X}^-$
2. **Process noise dominates**
    - $Q$ is large
    - $P$ grows
    - $P^-C^T$ becomes large
    - $K$ becomes large
    - $\hat{X} \approx Y$

> Note: The Kalman filter never tries to completely trust either the model or the measurement. Instead, it computes *how much to trust each one* at every step. The maths behind it is bascially optimising for least-squares.

In summary:

- $P^-$ tells us how uncertain we are about our state
- $Q$ tells us how much uncertainty the model introduces
- $R$ tells us how noisy the measurements are

The Kalman gain $K$ automatically balances these uncertainties to give us the best possible estimate.

> The same way that the optimal full-state feedback control system is called LQR (Linear Quadratic Regulator), the Kalman filter is called LQE (Linear Quadratic *Estimator*).

## LQG

LQR requires knowledge of the full state, and the Kalman filter provides that. So all that's left is to put them together. When we do, we get the **Linear Quadratic Gaussian** (LQG), named as such because of the linear system model, quadratic cost function, and the assumed gaussian noise to model uncertainty.

LQG is the optimal control for when our system is noisy and we don't have access to the full state... *which is most systems*.

An important point to note about LQG and one of the reasons it's so easy to design, is the separation between the two control loops. Those being of the system and of the state error.

Notice that:

$$
\begin{aligned}
A&-BK \rightarrow \text{Control Dynamics from LQR}\\
A&-LC \rightarrow \text{Estimation Dynamics from Kalman}
\end{aligned}
$$

The two systems are not related in any way. So it is possible to design them completely separately. This is called the *separation principle*

However, if measurement noise is high, the Kalman filter must reduce its gain to avoid amplifying noise. This also slows down the estimator.

If the estimator becomes too slow relative to the system dynamics, the controller begins acting on outdated or inaccurate information. In such cases, we may need to slow down the control system itself to maintain stability and performance.

This reflects a fundamental tradeoff between faster systems require better (and faster) state estimates and noisier measurements limit how fast we can reliably estimate the state. 

This ties in very directly with loop bandwidth limitations, which is yet another topic I will talk about in the post about discrete controls.

Another point to note about LQG is what we lose. With LQR we have a *gain margin* of $-6dB \ (\frac{1}{2})$ to $\infty dB$. That means that the input to the system can be up to half as powerful to infinity more powerful and the system will remain stable. 

Similarly for *phase margin*, we get a phase margin of at least $60\degree$. This is related to how much the control signal can be delayed before the system becomes unstable, but it's difficult to relate it to time without more information. If you want to learn more about this, I would highly recommend [this video by Brian Douglass](https://youtu.be/ThoA4amCAX4) that goes into more detail. 

With LQG however, we don't have any such guarantees. The gain and phase margins need to be measured on a system to system basis. This is a huge loss.

> This result was actually pretty shocking to the controls world when it was discovered. In a seminal paper by Dr. John C Doyle entitled "Guaranteed Margins for LQG Regulators", the abstract simply read: There are none.

This is what led to the development of **robust control theory**. Unlike LQG, which focuses on optimal performance under assumed noise models, robust control focuses on ensuring stability and performance even when the model is wrong.

In systems where failure is not acceptable, robustness is often more important than optimality, even if it comes at the cost of performance. This tradeoff between performance and robustness is a central theme in modern control, and we will explore it in a later post.

## Kalman Filters Beyond Control

Although we introduced the Kalman filter in the context of control systems, it is much more general. At its core, the Kalman filter is a **recursive estimator** for systems with noise.

This makes it useful in many fields:
- Navigation systems (GPS, inertial navigation)
- Signal processing
- Radar and tracking systems
- Econometrics and financial modeling
- Robotics and SLAM

In all these cases, the goal is the same: Estimate an unknown state from noisy observations.

The Kalman filter provides an optimal solution (under Gaussian assumptions) to this problem.

For example, lets look at a system called a *bearing only tracker*. 

In this setup, we only measure the **direction** of an object, not its distance using an array of microphones. With beamforming, we can estimate the **direction of arrival** of a signal.

This gives us a measurement of the form: $\theta = \tan^{-1}\left(\frac{y}{x}\right)$. However, this measurement does not tell us how far away the object is or how fast it is moving. A single bearing measurement corresponds to a **line of possible positions**. This means the system is not fully observable from one measurement alone. This is where we use the Kalman filter.

As the system moves and we collect multiple noisy measurements over time, the Kalman filter predicts how the object should move and updates the estimate using each new bearing measurement. Over time, the uncertainty shrinks and the estimate converges to the true trajectory.

This makes the Kalman filter ideal for problems like:
- radar tracking
- passive sonar
- audio source localization

## Beyond the KF

The standard Kalman filter assumes that the system dynamics are linear and the measurements are linear. However, many real-world systems are **nonlinear**.

For example:
- bearing measurements involve a tangent function
- robot dynamics are often nonlinear
- many physical systems have nonlinear constraints

To handle this, we extend the Kalman filter.

### Extended Kalman Filter (EKF)

The EKF handles nonlinear systems by **linearizing them locally**.

Consider a nonlinear system:

$$
x_{k+1} = f(x_k) + w_k
$$

$$
y_k = h(x_k) + v_k
$$

The EKF approximates these using first-order Taylor expansions.

Instead of using fixed matrices $A$ and $C$, we use:

$$
A_k = \frac{\partial f}{\partial x} \Big|_{x = \hat{x}_k}
$$

$$
C_k = \frac{\partial h}{\partial x} \Big|_{x = \hat{x}_k}
$$

These are called **Jacobians**.

This allows us to pretend the system is linear around the current estimate at each time step.

The EKF is nice because it's computationally efficient, easy to implement and it works well when the system is *approximately linear*. But linearization can introduce error. 

The estimates diverge if the system is highly nonlinear and it depends heavily on the quality of the initial estimate. The EKF assumes that the uncertainty is "small enough" that the nonlinear system can be approximated as linear over that region. If the uncertainty grows too large, this assumption breaks down, and the filter can diverge.

### Unscented Kalman Filter (UKF)

The Extended Kalman Filter works well in many cases, but it relies on one key idea: **local linearization**. For highly nonlinear systems, this approximation can become inaccurate.

The Unscented Kalman Filter takes a different approach. Instead of approximating the system, it tries to better capture how **uncertainty itself transforms** through a nonlinear system.

The core idea is simple: rather than linearizing the function, we propagate a set of carefully chosen sample points (called sigma points) through the nonlinear dynamics.

These points are selected to represent the mean and covariance of the state. After passing them through the nonlinear system, we recombine them to recover an updated estimate.

Think of it like this. The EKF tells us what's the best linear approximation of this system. The UKF tells us how uncertainty actually flows through this system.

The UKF is generally more accurate than the EKF for strongly nonlinear systems, but comes at a slightly higher computational cost.

If you’re interested in going deeper into this, I’d highly recommend exploring:
- sigma point methods
- nonlinear transformation of distributions
- and how uncertainty propagates through nonlinear functions

This topic is closely tied to how we think about estimation in modern systems (like SLAM), and I may revisit it in a future post once I’ve explored it in more depth.

## Conclusion

If there’s one idea to take away from all of this, it’s this. The Kalman filter is just a way of *balancing uncertainty*.

On one hand, we have a model that tells us how the system should behave. On the other, we have measurements that tell us what the system *actually* did and neither is perfect since the model accumulates error over time and the measurements are noisy and imperfect  

The Kalman filter doesn’t try to ignore either of them. Instead, it continuously figures out how much to trust the model vs the measurement.

From control systems to radar tracking to robotics and beyond, this idea shows up everywhere. Whether you're designing controllers, building SLAM systems, or estimating hidden variables in a signal, the same principles apply. 

And if you zoom out far enough, the Kalman filter is really just a system that makes the best possible guess about reality, given incomplete and noisy information.

If you’re interested in going deeper, the natural next steps are:
- Nonlinear systems and estimation (EKF, UKF in practice)
- Robust control and why LQG can fail
- Information theory and optimal estimation from a probabilistic perspective

You can either wait for the inevitable blog post or look into it yourself!

But for now I hope you can finally answer the question, "But what is the Kalman Filter?"
---
layout: post
title: A Deeper Look at Moving Averages
custom_excerpt: > 

notes: >
    Flow
    x Introduction
    - The Simple Moving Average (SMA) as an FIR Filter
    - Convolution View: Why It’s FIR
    - FIR and IIR Filters
    - The Exponential Moving Average (EMA) as an IIR Filter
    - Recursive Form and Memory
    - The DTFT
    - Frequency Response of the SMA
    - Other window types
    - FIR vs IIR: A Structural Comparison
    - Time Constant, Intuition, and Physical Meaning
    - EMA as a Discrete-Time Low-Pass Filter
    - Connection to State-Space and Estimation
    - When to Use What
    - Closing Thoughts
---

## Introduction

Moving averages are everywhere. Anytime you need to work with data, one of the first things you might do is average the last few samples. It smooths out noise and makes underlying trends easier to see.

But there’s a problem. What is it actually doing?

If the window is too large, the output starts to feel… off. It clearly suppresses rapid fluctuations, but how exactly does that depend on the window size? Why are all samples weighted equally? And what is an Exponential Moving Average? What are we really trading off when we change these choices?

Let’s approach the problem with a bit more rigour than it is generally given.

## The Simple Moving Average

I'll do this quickly, since I believe most people need at most a reminder of what the simple moving average (reffered to as the SMA henceforth) is. However, there will be some constructs made that aid in the maths down the line.

Let's say we have some data $x[n]$. The SMA with window size $N$ is defined as:

$$
\mathrm{SMA}_N\left(x[n]\right) = \frac{1}{N}\sum_{i=0}^{N-1} x[n-i]
$$

Notice that this definition uses the *last* $N$ samples. This makes the operation *causal*, meaning that the output at time $n$ depends only on the present and past inputs:

$$
y[n] \text{ depends only on } x[n], x[n-1], \dots
$$

This is a crucial property for real-time systems. If data is arriving continuously, we need to be able to compute the output immediately, without access to future samples.

We’ll come back to this idea later, but it’s worth keeping in mind from the start.

Another crucial property this has is that there are a finite number of samples used. This means that the filter has a **Finite Impulse Response**. I think the finite part makes sense, but what is an impulse response?

## Impulse Response 

When it comes to signal processing, the impulse reponse is a fundamental property of a filter. To really understand it though, we need to understand 3 things. The impulse and two other properties of filters:

1. Linearity
2. Time invariance

### Linearity 

A system is said to be *linear* if it satisfies:
$$
f(ax + by) = a f(x) + b f(y)
$$

That is, a linear combination of inputs produces the same linear combination of outputs.

Some familiar examples of linear operators include differentiation and integration. Importantly for us, the SMA is also linear.

On the other hand, many real-world “filters” are not linear. A common example is overdrive or distortion in audio systems. Since these effects rely on clipping, they do not preserve linearity. For instance, there are cases where:

$$
f(x + y) \ne f(x) + f(y)
$$

This lack of linearity means we cannot analyze them using the same tools we use for systems like the SMA.

### Time invariance

A time invariant system satisfies the following property:

$$
\begin{aligned}
    \text{Given that: } y(t) &= f(x(t)) \\
    y(t-t_0) &= f(x(t-t_0)) 
\end{aligned}
$$

That is, delaying the input signal gives the same result as delaying the output signal. 

An easy example for a *time variant* signal would be something like $y(t) = t\sin(x(t))$. To check time invariance, what we do is check the output when we delay the input, then check a delayed output.

Delaying the input:

$$
\begin{aligned}
    x(t) \rightarrow x(t-t_0) \\
    y'(t) = t\sin(x(t-t_0))
\end{aligned}
$$

Now we can check a delayed output:

$$
\begin{aligned}
    y(t) &= t\sin(x(t))\\
    t &\rightarrow t-t_0 \\
    \therefore y(t-t_0) &= (t-t_0)\sin(x(t-t_0)) 
\end{aligned}
$$

We can see here that $y'(t) \ne y(t-t_0)$, therefore the system is not time invariant, it is time variant.

Now let's check another system: $y(t) = sin(x(t))$

Delaying the input:

$$
\begin{aligned}
    x(t) \rightarrow x(t-t_0) \\
    y'(t) = \sin(x(t-t_0))
\end{aligned}
$$

Delaying the output:

$$
\begin{aligned}
    y(t) &= \sin(x(t))\\
    t &\rightarrow t-t_0 \\
    \therefore y(t-t_0) &= \sin(x(t-t_0)) 
\end{aligned}
$$

In this case, $y'(t) = y(t-t_0)$, so the system is time invariant. The intuitive understanding of this is essentially that if the system or filter depends on something that changes with time, not just the output, it's time variant. Think of something like temperature, which changes throughout the day.

### The Impulse

I've saved the best for last. Understanding what an impulse is and how it behaves is quite possibly one of the most important parts of learning systems. 
---
layout: post
title: The Equations that Control the World
custom_excerpt: > 
    If you’ve ever wondered how machines manage to behave so precisely in a messy, unpredictable world, the answer is usually feedback. In this post, I build an intuition for feedback control, explain how we decide whether a system is behaving well, and show why simple control ideas are so widely used, even when the systems themselves are anything but simple.

notes: >
    Flow
    - What is feedback
    - Quantifying system performance
        - Rise time
        - Peak/Peak time 
        - Settling time
        - Steady state error
    - What are controllers
    - PID terms and how they work
    - Where PID falls
---

## Introduction
Have you ever looked around and noticed how many machines -- old or new -- move with almost unsettling precision? Systems that track, stabilize, and correct themselves better than a human ever could? If you’ve built projects before, you might remember manually controlling each actuator. One input per motor, one knob per output. So how do real systems do it automatically? How do they decide how much to move, when to stop, and how fast to react?

Well, buckle in dear reader, for all the answers are in a small set of ideas and equations that quietly run cars, drones, factories, medical devices, and just about everything else that moves on purpose.

# Feedback

Often when I talk to people about controls, they have this idea that you have to solve for something then invert that equation to figure out the input. *This may work*, but the work associated with it is immense and if there are any stochastic events, it would fall apart instantly. 

Going with this approach at face value, let's look at an example.

Imagine designing a shower system that can automatically set the water temperature. To get an exact set of equations for the temp, we would need to describe how the ambient temperature affects the water in the water tank, how pipes of variable lengths and thicknesses would lose heat, and various other processes. Not to mention random events like wear and tear, power cuts that affect pumps and water heaters, how many people are using the water in the tank, etc.

Could this be done? No, not to any real effect, since we have so many variables, most being *random, time-varying, and dependent on the installation*. 

Lets take a step back and see how we do it. I want you to think about your own shower. It has one knob, pressing it in and out controls the water pressure and turning it controls the water temperature. For now, imagine the water pressure is either constant or inconsequential -- just focus on the temperature. 

If I ask you how to set the temperature, chances are you would not be able to do it on your first go. You'd put the water on, see if it's too hot or cold, then adjust accordingly. Why can't we make our system do the same thing? 

Firstly, we need to shift our mindset from what the actual temperature is to *how different it is from what we want*. In formal terms, that's the **error**. Depending on this error, we know if we need to make the water hotter or cooler. 

But to see how different the temperature is, we need to *sense* it. When we're using our own showers, we are the sensors. But for an automatic system, we need an external **sensor**. 

Then, depending on how large the error is, we need to take some action. This is called the **controller**. In the shower example, we are the controller. We decide how much to change the ratio of hot and cold water by turning the knob.

This action is then applied back to the physical system itself -- the water, heater, pipes, and tank. In control systems, this entire thing being acted upon is called the **plant**. The name comes from industrial automation, where the “plant” referred to the factory or process being controlled.

The process of continuously measuring what the system is doing, comparing it to what we want it to do, and using that difference to adjust the input is called **feedback**. Instead of trying to predict the world perfectly, feedback lets the system react to how the world actually behaves. It is quite possibly *the most important concept in control systems*, and nearly everything that follows builds directly on this idea.

## Quantifying system performance

Before we talk about how to control things, we need to understand what we're controlling. We need some way to quantify how a system behaves. For this, we look at another example.

Imagine a spring with a mass at the end. I know a lot of physics classes have this, showing how the mass oscilates between two points forever as a demo for potential energy and kinetic energy, but that's always with a huge caviet: *We don't consider friction*. 

In actuality, what we expect is for the spring to oscilate, with each peak getting smaller and smaller. This is called *damping*, and there are actually four different variations of it. 

<p align="center">
  <img src="/assets/images/damping.png" alt="Damping diagram">
</p>

Okay, I know this graph seems really noisy but let's walk through it step by step. 

This is what's known as a step response. It's the output of the system to a *constant input*, which in this case is a constant *force*. Think of it like holding the mass steady and then suddenly letting it go. Gravity gives it a constant input.

The frictionless case I mentioned corresponds to the undamped plot. It's slow to start, but quickly gets up to speed. Once it passes the midpoint, the spring will start pulling it back, decelerating the mass until it eventually moves up. 

Then, with the  

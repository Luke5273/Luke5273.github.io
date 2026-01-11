---
layout: post
title: From Hobby Projects to Real Embedded Firmware 
custom_excerpt: >
    If you’ve ever played around with Arduino or a robotics kit, you probably noticed that it’s fun and easy, but real embedded firmware is a whole different world. In this post, I share what I learned from building an AUV and doing an embedded programming internship. The little mistakes that can cause big headaches, and the practices that make code reliable. Plus how thinking carefully about your system -- and its limited resources -- changes the way you write code for microcontrollers.

notes: >
    The points I want to make are:
        x lack of asserts and return value checking 
        x no watchdogs
        x using state machines
        x using interrupts instead of delay
            x MENTION JITTER AND HYGIENE
        x DMA
        x being resource aware
        x Arduino HAL vs anything else
---

## Table of Contents

1. [Introduction](#Introduction)
2. [Asserts and Return Value Checking](#1-asserts-and-return-value-checking)
3. [Watchdog Timers](#2-watchdog-timers)
4. [State Machines](#3-state-machines)
5. [Interrupts](#4-interrupts)
    - [Timer Interrupts](#timer-interrupts)
    - [Interrupt-Based Serial](#interrupt-based-serial)
    - [ISR Hygiene](#isr-hygiene)
6. [DMA (Direct Memory Access)](#5-dma)
7. [Conclusion](#conclusion)

## Introduction

I think it's pretty common for kids interested in programming to be put into a robotics class at some point in their life. These often feature some kind of block-based programming a-la something like scratch. Eventually they end up writing arduino code, but then the progression kinda... stops. Everyone knows block-based isn't the endgame solution, but why do people not think the same for beginner arduino style code?

I think I can try and answer this, as someone in college trying to get into the industry and having done an internship (which is impressive in this economy) and projects. Hopefully I can give industry vets a glimpse into what it's like getting into the industry now -- where the barrier for entry is so low -- to people still stuck on the hobby side of things a few reasons why they should consider giving an honest effort to level up not only their coding game, but their systems level view on these projects.

Before we *jump right into it*, a bit about my experience. I am the electrical subsystem head at the AUV student project in my college. This involves not only a decent amount of coding myself, but also a lot of reviewing other people's code. I also recentely completed an internship where we wrote a bootloader for the STM32 blue pill (F103C8T6) to enable over-the-air (OTA) updates using a GSM module. Before the internship I knew how to think in systems because of the AUV work, but the internship was my first exposure to real embedded code and it felt like my brain expanded three times at least. I want to share some common pitfalls and design decisions that someone who hasn't seen real code can fall into.

## 1. Asserts and return value checking
How many times have you seen something like:
``` c
HAL_UART_Receive(&uart1, index, 1, 5000);
index = index - '0';
array[index] = 5;
```
Now, for the STM32 uninitiated, this is the code to read bytes from UART within a certain timeout. The arduino equivalent would be something like:
``` c
Serial.setTimeout(5000);
index = Serial.read();
index = index - '0';
array[index] = 5;
```
THIS IS DANGEROUS. You do not know if the data you were expecting is in the buffer. If the mcu didn't receive anything and index wasn't defaulted to something, we could get a segfault. The issue is that the UART read can fail silently by timing out. In this case, ```HAL_UART_Receive(...)``` can timeout, proceeding to the next line after those 5 seconds are over. A better way of doing this would be:
``` cpp
auto status = HAL_UART_Receive(&uart1, &index_byte, 1, 5000); 

// for arduino, check if the number of bytes received is correct via index == 1
if (status == HAL_OK) 
{ 
    int index = index_byte - '0';
    array[index] = 5;
} 
else 
{
    // handle UART read error or timeout
}
```
Now, this still isn't perfect. You'd want to make sure ```index_byte``` is a valid index otherwise you'll get another segfault.

This is called return value checking. It's really quite important, since you'll actually get to know if functions you call succeed or fail. The only unfortunate thing is that it makes your code a lot longer, which isn't an issue per se, but it definitely can be tedious to write. A more common way of laying things out to try and mitigate this is as such:
``` c
if(HAL_UART_Receieve(&uart1, &index_byte, 1, 5000) == HAL_OK) 
{
    int index = index_byte - '0';
    array[index];
}
else 
{
    // handle UART read error or timeout
}
```
This is just a few extra characters to save a lifetime of hurt whilst debugging.

What we just did is an example of runtime error checking: your code actively tests whether the UART read succeeded and responds if it didn’t. This is great for reliability, but it comes at a cost: extra instructions, memory, and sometimes a bit of added latency. In systems where failures are rare and compute, memory, or timing budgets are tight, constantly checking every return value might be overkill. For those situations, it can be helpful to check assumptions only while testing. That’s where assert() comes in. An assert verifies that a condition you expect to be true actually is. If it isn’t, the program halts immediately, usually entering a debug trap where you go into a while(1) loop, which you can pretty easily see by halting the debugger.

```c
int index = index_byte - '0';
assert(index >= 0 && index < ARRAY_SIZE);  // sanity check only during testing

array[index] = 5;
```
During development, this immediately catches programming mistakes (like sending the wrong indices) before they cause mysterious crashes in the field. In production, you can disable asserts (```#define NDEBUG```) so that they don’t consume CPU cycles or memory, leaving only your essential runtime checks. In safety-critical systems, asserts are sometimes left enabled but redirected to a controlled reset or fault handler instead of halting forever.

The rule of thumb is:
- Runtime error checking for things that can realistically fail in normal operation (UART timeouts, sensor disconnection).
- Debug-time assertions for conditions that should never happen if the code is correct (Indexing outside an array).

## 2. Watchdog timers
Remember those ```while(1)``` loops I mentioned earlier? Well what happens if we want to do something about it.

If something gets stuck, everyone's first reaction is to turn it off and on. Embedded systems are no different. Watchdog timers are timers that you have to periodically reasure them that everything is fine. If they don't get that reassurance, they hardware reset. It keeps track of a counter, which triggers the reset if it hits a user specified amount. The reassurance is setting the count back down to zero, effectively restarting the timer.

This isn't useful only for asserts, it's useful for any place where you risk going into an infinite loop. For example:
```c
volatile bool command_received = false;

void USART_IRQHandler(void) 
{
    command_received = true;
}

int main()
{
    ...
    while (!command_received) 
    {
        // wait for command
    }
    ...
}
```
Imagine a scenario where a machine waits for some kind of command, executes it, then halts till the next command. Another computer sends the command via UART. While designing a system, we know that the host computer will send a ping every 2 seconds. If we don't get it, we know something is wrong. Maybe due to a cable issue, EMI, a UART peripheral glitch, or a bug elsewhere in the system. We don't know, but going back to a known good state is out best bet. 

Without a watchdog, someone would manually have to notice and powercycle the device. With a watchdog, we are able to automatically restart when we know (or at least strongly suspect) that we've reached a bad state.

Now you may be asking something along the lines of 'what if the task I am trying to complete just takes a while? How does it know when to reset the counter?' Well, dear reader, both of those are excellent questions and I am glad you're paying attention.

The answer to both is that *you have to reset it*. You decide what has the possibly of being an infinite loop and where to put the watchdog resets. For example: 
```cpp
for(auto& pixel: pixels)
{
    HAL_IWDG_Refresh(hiwdg);
    processPixel(pixel);
}
```
Now, the entire loop can take as long as it likes. The only way the watchdog can be triggered is in ```processPixel(...)``` takes too long. Remember, the watchdog timer is checking the heartbeat of your code. If that heartbeat stops, the program restarts.

## 3. State machines
Have you ever written logic like this?
```
wait_for_input()
do_something()
wait_for_next_input()
do_something_else()
...
```
This is all fine and dandy if you're only ever doing one thing, but what if you have multiple systems working at the same time? If the input timings are non deterministic, this pattern falls apart completely. The solution to this issue is *state machines*.

Anyone that has done a digital design course is quaking in their boots right now. Yes, it turns out that state machines are useful outside of that class. Really, they're useful in a lot of places because of how good they are at modelling behaviour. I’m not going to re-teach finite state machines here. If you’ve taken a digital design course, you already know the theory. If you haven't, well spend like an hour watching a video on youtube. What matters in firmware is not what a state machine is, but how it lets you structure non-blocking, time-aware systems.

A state machine models behaviour as a set of discrete states and well defined transitions between them, driven by events. That sounds like a lot I know, but once you understand it, it's really powerful when describing a surprising amount of things. Lets look at an example using a turnstile.

A turnstile is one of those machines at the metro which lets you pass if you have a valid ticket. Simplifying the working, we have two states:

- Closed
- Open

To go from one state to another we have these operations:
- Closed to open -> show a valid ticket
- Open to closed -> inbuilt timer elapses

We don't necessarily have to indicate the inverses of these operations, but it is good practice.  
- Closed to closed -> If an invalid ticket is presented
- Open to open -> If the timer hasn't elapsed 

It is easy to see the relation drawn out in a *state transition diagram* like this:

<p align="center">
  <img src="/assets/images/turnstile_state_transition_light.svg" alt="State machine diagram">
</p>

The state machine is something that can very easily be translated to code. It becomes even easier if we pull it out into something like a ```turnstile_control()``` function.

```c
typedef enum {
    TURNSTILE_CLOSED,
    TURNSTILE_OPEN
} turnstile_state_t;

static turnstile_state_t state = TURNSTILE_CLOSED;
void turnstile_control()
{
    switch(state)
    {
        case TURNSTILE_CLOSED:
            if(valid_ticket())
            {
                state = TURNSTILE_OPEN;
                open_gate();
            }
            break;

        case TURNSTILE_OPEN:
            if(timer_elapsed())
            {
                state = TURNSTILE_CLOSED;
                close_gate();
            }
            break;
        
        default:
            assert(false); // state corruption
            state = TURNSTILE_CLOSED; // recovery for release builds
            close_gate();
            break;
    }
}
```

Over here it's pretty easy to see how the state transitions map into the logic of the switch cases. What this allows us to do is advance the system by one small, deterministic step per tick, instead of waiting for an entire operation to finish. If we have multiple things going on, that lets us do a little bit of all of them.

The unfortunate thing however, is that this is still going to be troublesome code if we have the aforementioned multiple things going on. ```close_gate()``` and ```open_gate()``` may be functions that take a long time. This would mean that ```turnstile_control()``` can be a function that takes a long time. If we have multiple of these control loops going, like we would in this case, it makes the system far less reliable. Imagine we have turnstile control, but also need to keep track of the batteries used to power the turnstile. If the batteries discharge too much because we were stuck in the control loop, dead batteries. It would be an expensive mistake. 

This gets to the heart of the state machine pattern. It enables *non-blocking* code.

Non-blocking code is when you write a system to be able to function without stopping everything else. Over here, calling ```close/open_gate()``` can be blocking. This makes the state machine blocking, going against the *exact pattern we were going after*.

A far better approach is to use the state machine to set flags that signal to the main loop to run some code. The key idea is to separate decision-making (state transitions) from work (hardware actions). This lets the state machine itself be nice and non-blocking without inherantly having to worry about the implementation. What that looks like is:
```c
volatile bool open_gate_flag = false;
volatile bool close_gate_flag = false;

typedef enum {
    TURNSTILE_CLOSED,
    TURNSTILE_OPEN
} turnstile_state_t;

static turnstile_state_t state = TURNSTILE_CLOSED;
void turnstile_control()
{
    switch(state)
    {
        case CLOSED:
            if(valid_ticket() && !open_gate_flag)
            {
                state = OPEN;
                open_gate_flag = true;
            }
            break;

        case OPEN:
            if(timer_elapsed() && !close_gate_flag)
            {
                state = CLOSED;
                close_gate_flag = true;
            }
            break;
        
        default:
            assert(false); // state corruption
            state = CLOSED; // recovery for release builds
            close_gate_flag = true;
            break;
    }
}

int main()
{
    ...

    while(1)
    {
        turnstile_control();

        ...
        if(open_gate_flag)
        {
            open_gate();
            open_gate_flag = false;
        }

        if(close_gate_flag)
        {
            close_gate();
            close_gate_flag = false;
        }
    }

    ...
}
```

However now we run into another issue. Since ```turnstile_control()``` is still being called in the same loop as ```open/close_gate()```, this doesn't actually solve anything. We can still have ```turnstile_control()``` being called at erratic times. 

What we’re missing is control over when ```turnstile_control()``` runs. As long as everything lives in the same ```while(1)``` loop, timing is implicit, fragile, and dependent on how long other code takes to execute. To make state machines truly reliable, we need a way to run them at predictable times -- independent of the rest of the system.

NOTE: Flags should only be cleared *after the action is taken*. If done before, the state machine may run again and restart the action.

## 4. Interrupts
Almost every arduino tutorial that I've seen -- and trust me, I've seen a lot -- teaches interrupts something like this:
```c
const int buttonPin = 2;
volatile int buttonState = 0;

void buttonISR() {
    buttonState = digitalRead(buttonPin);
}

void setup() {
    pinMode(buttonPin, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(buttonPin),
        buttonISR,
        CHANGE
    );
}

void loop() {
    if (buttonState == 1) {
        // do something
    }
}
```

If it's a good tutorial, it may even explain what ```volatile``` means! They get some things right introducing ISRs and getting your head around the interrupt mental model. But the reason I don't like this is because it really paints interrupts as this ‘fancy callbacks for a button’. In reality, hardware GPIO interrupts are only one of several interrupt sources available on a microcontroller. Some other interrupt sources are: 
- Timer interrupts
- ADC conversion complete interrupts
- UART RX/TX interrupts
- DMA interrupts
- SysTick (on ARM)
- Software-triggered interrupts

Honestly, interrupts could take up a whole article by themselves. Even each of these sources could take up an article. In the interest of time though, I will give further reading as homework. Here, we will talk about timer interrupts amd  UART Rx/Tx, with DMA interrupts in their own section.

NOTE: Not all interrupts are equal, they have different priorities.

### Timer interrupts
Timer interrupts have two modes. One-shot and periodic timers. The way that I think about them is like the mono-stable and a-stable modes of an oscillator or 555 timer. One-shot timers fire an interrupt a set time after they're set. They're very useful for tasks like mechanical inputs and setting delays, but what I really want to talk about right now are the periodic timers. 

Periodic timers are the lifeblood of non-blocking code. They solve the earlier issue of not being able to control the timing of control loops. Determinism in loops like this is extremely important. A lot of algorithms require equally spaced points or samples. 

In the turnstile example we took earlier, this is the final piece we need before it's actually non-blocking.

```c

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2)   // example timer
    {
        turnstile_control();
    }
}

volatile bool open_gate_flag = false;
volatile bool close_gate_flag = false;

typedef enum {
    TURNSTILE_CLOSED,
    TURNSTILE_OPEN
} turnstile_state_t;

static turnstile_state_t state = TURNSTILE_CLOSED;
static uint32_t open_timer_ms = 0;

void turnstile_control(void)
{
    switch (state)
    {
        case TURNSTILE_CLOSED:
            if (valid_ticket())
            {
                state = TURNSTILE_OPEN;
                open_gate_flag = true;
                open_timer_ms = 0;
            }
            break;

        case TURNSTILE_OPEN:
            open_timer_ms++;

            if (open_timer_ms >= 2000)  // 2 seconds @ 1 kHz timer
            {
                state = TURNSTILE_CLOSED;
                close_gate_flag = true;
            }
            break;

        default:
            assert(false);              // state corruption
            state = TURNSTILE_CLOSED;
            close_gate_flag = true;
            break;
    }
}

int main()
{
    ...

    HAL_TIM_Base_Start_IT(&htim2);
    
    while(1)
    {
        ...
        if(open_gate_flag)
        {
            open_gate();
            open_gate_flag = false;
        }

        if(close_gate_flag)
        {
            close_gate();
            close_gate_flag = false;
        }
    }

    ...
}
```

Over here, Timer2 is set to trigger every 1ms. This allows us to bake timing into the state machine very easily, meaning that delays can now be non-blocking. Also, it lets the control parts of the code trigger even if the actual implementations are going on. This essentially makes a very rudimentary scheduler. 

An important note, timer frequency is a design parameter, not a constant. You choose it based on how precise your system needs to be versus how much CPU time you can afford to spend in interrupts. We control two parameters, the prescaler and the period.

The prescaler is a scaler for the system/secondary clock that is driving the interrupt. It lets us divide the clock frequency by an arbitrary (integer) amount. The prescaler sets the *resolution* of each tick. Then the period is how many of those prescaled ticks we need to count before triggering the interrupt. 

Let's look at an example. Consider a chip capped at 60MHz. We're first going to scale the clock down to 1MHz, then count to 1000. This has the effect of dividing by 1000, which brings 1MHz to 1KHz, or 1ms time period, which is what we wanted above.

It is important to remember that the interrupt gets called every 1ms though. If the logic inside the interrupt takes longer than 1ms, we run into timing jitter, where the time between sequential calls shifts because ISR execution time varies or exceeds the timer period, *which is exactly what we want to avoid*. This can be minimised by having good 'ISR Hygiene', which I will talk about in a minute.

### Interrupt based serial
Imagine code code like this:
```cpp
String command_str;
void loop()
{
    command_str = "";
    while(Serial.available() > 0)
    {
        char c = Serial.read();
        if(c == '\n') 
            break;
        else 
            command_str += c;
    }

    auto command = parseCommand(command_str);
    
    switch(command)
    {
        ...
    }

    otherTasks();
}
```
This looks very normal. In fact, it's adapted from the example code in the arduino documentation for UART. What this code does is fill a command buffer with a command, parse it, and then continue with other tasks. On the surface, it's completely fine, right? WRONG. There are a lot of pitfalls in it. Let's walk through them.

First, exiting the loop requires us to encounter either a new line or for the serial buffer to empty. Neither of those things are guaranteed, so the execution of the other tasks can be completely starved if the uart buffer is being fed junk data. This is one of those bugs that won't show up in the lab, but definitely will in the field where noise reigns supreme. 

Second, this code assumes that when the serial buffer is empty, the entire command is read. This is also not guaranteed. Depending on how ```parseCommand()``` is written, this could very easily crash your system. This is a bug that you'll encounter as the clock speed of your chip increases. On faster chips, the main loop can run multiple times between UART byte arrivals, making it more likely that ```parseCommand()``` sees incomplete data.

Third, if the other tasks take too long, the UART buffer on the mcu can overflow, causing you to lose data. With an arduino UNO and UART at 115200, the 64byte UART Rx buffer can fill up in about 6ms. That is not a lot of time. Unlike the second reason, this is a bug you'll encounter on slower chips. 

So, if this code has issues with faster chips and slower chips, whats the solution? Well, dear reader, considering we're in the interrupts section and the interrupt based serial subsection, I'm sure you can make an educated guess.

The answer of course, is interrupts. Specifically interrupts that fire when the UART Rx buffer receives a byte (or multiple). What this allows you do to is to is to keep your own larger buffer depending on your application, which can control behaviours like overflowing, and let you signal to your main loop when data is ready to be processed. It also makes you keep the fact that you need to *detect* when commands are complete, rather than just kinda hoping and praying that you got a full command at the moment you decided to poll the serial buffer.

A better way of writing it is below.
```c
#define CMD_BUF_SIZE 256

UART_HandleTypeDef huart2;

static uint8_t rx_byte;
static char command_buf[CMD_BUF_SIZE];
static volatile uint16_t cmd_index = 0;
static volatile bool command_ready = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        char c = (char)rx_byte;

        if (c == '\n')
        {
            command_buf[cmd_index] = '\0';   // null terminate
            command_ready = true;
            cmd_index = 0;
        }
        else
        {
            if (cmd_index < CMD_BUF_SIZE - 1)
            {
                command_buf[cmd_index++] = c;
            }
            else
            {
                // buffer overflow → reset safely
                cmd_index = 0;
            }
        }

        // Re-arm interrupt
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

int main(void)
{
    ...
    // arm interrupt
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

    while (1)
    {
        if (command_ready)
        {
            command_ready = false;

            command_t command = parseCommand(command_buf);

            switch (command)
            {
                ...
            }
        }

        otherTasks();
    }
}
```

This is like the turnstile code, where we raise a flag and get the main loop to do all the heavy lifting; if we get another interrupt while we're processing the last one, we may lose it. Futher, the advantanges of non-blocking code can be seen, with other tasks not being starved, and being more deterministic about timing. Also we have a 256 byte buffer instead of the measly 64 bytes we had earlier, which gives us about 20ms before the buffer fills up. We can make it even larger if we want though. While writing the OTA bootloader, When we were receiving the bytecode for the update, it was through a UART connection with the gsm module. The update was split into 1kb chunks and send along with some metadata like length and a checksum. So we made a 2048 byte buffer with no issue.

One limitation of this simple pattern is that it only stores one complete command at a time. If a second command arrives before the first one is processed, it will overwrite or be lost. In systems where command bursts are expected, this pattern is usually extended into a ring buffer or queue. However, I will also leave this as homework to the reader.

### ISR hygiene
The last thing I want to say about interrupts -- since this section has already become far longer than I anticipated -- is about how to actually write them? I have mentioned jitter and hygiene, but what what exactly are they? Why are they important?

Interrupt Service Routines (ISRs) are special code. They do not behave like normal functions, and treating them as such is one of the fastest ways to create bugs that are impossible to reproduce. As a rule of thumb, an ISR should do the absolute minimum required to record that an event happened. When this rule of thumb is violated we get bad ISR hygiene, which is when code works, technically, but isn't robust and is very prone to errors. 

Here are some rules and why they are important:

***
<br>
#### 1. Keep ISRs short and deterministic
An ISR should execute in a bounded and predictable amount of time, since while an ISR is running, other interrupts of equal or lower priority are blocked. Long ISRs increase interrupt latency and introduce jitter into time-critical systems.

Avoid parsing data, running control algorithms, calling slow HAL functions, anything with unbounded runtime. Almost all of these can be replaced with a flag that makes them run in the main loop.

#### 2. Never block inside an ISR
ISRs must never wait for anything since an ISR that blocks can deadlock the entire system. The thing you are waiting for may itself depend on interrupts being serviced.

Avoid the use of delays, polling for conditions, waiting on flags, spinning in loops, or calling functions that might block internally. Instead, try using state machines for more complicated logic and delays. 

#### 3. Do not allocate memory in an ISR
Avoid dynamic memory entirely. Heap allocators are typically non-reentrant, non-deterministic, and may disable interrupts internally. Even if it “works once,” it will fail eventually.

Do not use malloc/new/String/containers, resize buffers, or rely on any heap-backed abstractions. Use simple stack based objects. If more complicated structures are required, use flags and implement logic in the main loop.

#### 4. Share data with the main loop very deliberatly
Be aware of compiler optimisations that can shoot you in the foot. Be liberal with your use of ```volatile```. Use atomic features where possible, especially in multithreaded scenarios. (NOTE: ```volatile``` prevents compiler reordering, it does not make access atomic.)

Keep in mind that interrupts can be fired almost *anywhere*. In the middle of parsing a buffer, that buffer's values can change. Never do major work on data shared with an interrupt, copy it into a local buffer. 

***
<br>
In general, an ISR should record that something happened -- not decide what to do about it.

I want to share a small anecdote specifically about point 4. The way we were checking to see if the GSM module was alive and working was to poll it for the IMEI number. Early in testing, the mcu was set at 8MHz. As soon as the IMEI was sent, the mcu received it properly. We then eventually increased it to 64MHz and suddenly it took at least six or seven attempts to read the data properly. Sniffing the UART showed that the data was being sent perfectly on either side, but looking at the RX buffer on the mcu showed that there were some strange null characters ('\0') that were messing things up.

The code looked like this (pseudocode):
```c
void UART_RxCallback()
{
    rxBuffer[rxLen++] = rxByte;
    if(rxLen >= RX_BUFFER_SIZE)
        rxLen = 0;
    rearmInterrupt(&rxByte, 1); // this is stm specific, but generally common
}

void getIMEI()
{
    initialiseGsmModule();
    wait(30); //30s for gsm module to start up
    
    while(attempts < MAX_RETRIES)
    {
        clearBuffer();
        uartSend(IMEI_COMMAND);

        wait(2); //2s
        while(noTimeout)
        {
            last = rxLen;
            rxBuffer[last] = '\0';
            char* t = strstr(rxBuffer, "IMEI");
            buf = imeiDecode(t);
            if(buf == GOOD_VALUE)
            {
                IMEI = buf;
                break;
            }
        }

        attempts++;
    }

    while(1); //FAILED TO GET IMEI
}
```

This is bad code. Just wanted to get that out of the way. Try and find the error before reading further.

The cause turned out to be a classic interrupt-related race condition. The RX interrupt was appending bytes to a shared buffer, while the main code was directly modifying that same buffer to temporarily insert a null terminator so that ```strstr()``` could operate on it. The code assumed this was safe because the null would be overwritten by the next received byte anyway. However, every so often, the UART interrupt would fire between reading the current buffer length and writing the null terminator. When that happened, real incoming data was overwritten instead.

At 8 MHz, this race window was small enough that we almost never hit it. At 64 MHz, the MCU was looping much faster relative to the UART byte arrival rate, dramatically increasing the chances of interrupting ourselves at exactly the wrong moment.

Making the code copy not only the length of the buffer, but of the buffer itself solved this problem instantly. 

```c
void getIMEI()
{
    initialiseGsmModule();
    wait(30); //30s for gsm module to start up
    
    while(attempts < MAX_RETRIES)
    {
        clearBuffer();
        uartSend(IMEI_COMMAND);

        wait(2); //2s
        while(noTimeout)
        {
            last = rxLen;
            memcpy(rxSlice, rxBuffer, last);
            rxSlice[last] = '\0';
            char* t = strstr(rxSlice, "IMEI");
            buf = imeiDecode(t);
            if(buf == GOOD_VALUE)
            {
                IMEI = buf;
                break;
            }
        }

        attempts++;
    }

    while(1); //FAILED TO GET IMEI
}
```
This bug, which I spent far far too long trying to find made sure I never forgot about point 4.

## 5. DMA 
Full honesty, I have not used DMA. I have seen it be used in the firmware at my internship, and we plan on using it with our ADCs since *we kinda have to*. But I'll do my best to explain what they are and why/where you'll use them.

So far, every example we’ve looked at assumes the CPU is involved whenever data moves: a UART byte arrives, an interrupt fires, the CPU copies the byte into a buffer, and then goes back to whatever it was doing. This works fine at low data rates, but it doesn’t scale. If you are receiving hundreds of kilobytes per second, or streaming ADC samples continuously, the CPU quickly becomes a glorified data copying machine.

DMA exists to solve this exact problem. Direct Memory Access allows peripherals to transfer data directly to or from memory without CPU intervention for each byte. Once configured, the DMA controller autonomously moves blocks of data while the CPU continues executing other code or sleeps entirely.

DMA does not replace interrupts, it changes what interrupts are used for. Instead of firing an interrupt for every byte, the system typically raises an interrupt only when a buffer is full, half full, or when a transfer completes. This drastically reduces interrupt rate and jitter. A UART running at 1 Mbaud generates one million bit events per second. With interrupts per byte, that’s 100,000 interrupts per second. With DMA, that might become one interrupt every 1 ms or even less. This means that interrupts (and by extension the CPU) will be used to figure out when to do something, rather than actually doing the thing. 

This article won’t walk through DMA setup or configuration. DMA is highly mcu specific, tightly coupled to bus architecture, and poorly abstracted across vendors. At this stage, what matters is recognizing when DMA is the right tool, not memorizing how to configure it, which is whenever data rates are high, timing or power matters, or the CPU just has better things to do than move memory around. 

## Conclusion
At the end of the day, all of this really comes down to being resource aware. On a microcontroller, time, memory, and determinism are finite resources, and every design choice spends some amount of them. Polling loops burn CPU time, interrupts introduce latency and shared state, buffers consume RAM, and higher clock speeds don’t make bugs disappear, they often make them easier to trigger. Good embedded code isn’t about being clever, it’s about knowing what you’re paying for and deciding whether that cost is acceptable for your system.

This is also where the difference between Arduino-style development and lower-level firmware starts to matter. Arduino hardware is perfectly capable (if expensive), but the HAL makes a number of convenience-driven choices on your behalf: dynamic memory allocation, implicit polling, hidden buffers, and opaque timing behavior. None of these are wrong, and for many projects they’re exactly what you want. The problem only appears when those abstractions leak and you don’t know what’s happening underneath. At some point, especially as systems grow more complex or more constrained, you need to understand the costs those abstractions hide.

This is why Arduino has a mixed reputation among engineers. It’s not because the platform is bad, but because it makes it easy to chain black boxes together without ever needing to reason about timing, memory, or concurrency. Writing firmware without those abstractions can feel intimidating at first, because there’s simply more you need to keep in mind. At the same time, that extra explicitness often feels better once you’re comfortable with it. Having to think about GPIO banks in something like ```HAL_GPIO_WritePin()``` makes it harder to forget what the hardware is actually doing.

Most of the concepts discussed here can be implemented in Arduino code as well, but some of them are awkward or unintuitive. Timer interrupts are a common example, since they’re already tied up in core functionality like PWM and servo libraries. Regardless of the framework you use, the important takeaway is to think consciously about the decisions you’re making. Embedded systems are long-lived, tightly constrained, and often maintained by someone else later. Being deliberate about resource usage and structure is one of the best favors you can do for future you or the poor soul who has to read your code next.

Finally, it’s worth briefly mentioning RTOSes. An RTOS doesn’t magically solve the problems discussed here, it just gives you more structured tools to manage them. Tasks, queues, and semaphores are all ways of making resource usage, timing, and concurrency explicit rather than accidental. Used well, an RTOS can improve clarity and determinism; used poorly, it can hide the same problems behind another layer of abstraction. The same rules still apply: understand what the system is doing, know the costs, and choose the right level of complexity for the job.

Anyway, thank you for reading this far. Hope you've learnt something useful, or at least have an idea of real embedded code looks like.
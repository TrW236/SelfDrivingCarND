# PID Controller

## Usage

* Please visit the [repository](https://github.com/udacity/CarND-PID-Control-Project) from Udacity.

* Replace all the files in the folder `src`.

## Result

The video is uploaded on Youtube. [link](https://www.youtube.com/watch?v=kO8BKBIiQGM&list=PLNDTbGbATLcED0iX8K-zY3vrNbwhxV8gC&index=4)

## Model Description

I used two PID controllers. 

* The first one is used to control the throttle. The desired velocity is changed according to the cross track error.

* The second one is used to control the steering angle according to the cross track error.

```
        |---------------------------------------------------|
        |                                                   |
        v                                                   |
[PID controller 2] --> [steering angle] ----|               |
                                            |--------> [cross track error]
[PID controller 1] --> [throttle] ----------|               |
        ^                                                   |
        |                                                   |
        |-------------- [set desired velocity] <------------|
```

## PID controller

* ``Proportional`` part is the main control part. But only with P controller the car will oscillate and can also be not stable (oscillate stronger).

* ``Derivative`` part is to control the change of cross track error. It's like, when the car drives too fast towards the center, the D controller will detect this action and slow down the car towards the center, which will damp the oscillation.

* ``Integral`` part is used against bias. For example, when there is strong cross wind, only the PD controller cannot let the car drive just right on the center. But the I controller can be used to against such bias.

### Implementation of the controllers

* The controller would be implemented using the equation from [Wikipedia](https://en.wikipedia.org/wiki/PID_controller). 

* The time interval `dt` was incoperated into `k`, due to that `dt` is almost constant.

* The calculations of the integral and differential part was using simple numerical methods.

### Choose appropriate Parameters

I chose ``k_p = 0.1``, ``k_i = 0.0001`` and ``k_d = 6`` for the PID controller for the steering (The time interval `dt` was incoperated into `k`, due to that `dt` is almost constant.). 

* First I chose a propriate ``k_p`` suitable for the car velocity. (the throttle is controlled with another PID controller)

* Second I chose a propriate ``k_d`` to damp the oscillation.

* I added a very small value to ``k_i``, because we don't have large bias in this simulation.

* Finally I tried to tune the parameter using a simple genetic algorithm. But I found that the parameters I previously chose are already suitable, so the tuning didn't help me much.# PIDcontroller

## References:

1. Udacity Self-Driving Car Engineer Nanodegree

2. [PID Controller (Wikipedia)](https://en.wikipedia.org/wiki/PID_controller)

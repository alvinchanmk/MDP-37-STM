# CE4171: Multidisciplinary Project

## MDP STM SOURCE CODE

# STM

The stm, the nervous system of the robot.

It has one main purpose, which is to control the robot.

This can be split into two parts, the dc motors and servo motor.

Controlling the robot is simple, we drive two dc motors to allow for forward and backward movement. We alter the angle of the servo motor to control the turning angle of the robot.

The bigger issue is how much and how long to turn them on for a specified distance or angle.

To do this we used the encoder to tell us how much the wheel has turned. By using math, we are able to stop the motor after it has moved forward a certain distance. However, this value assumes that the wheels do not slip. To resolve this we restricted the power of the motors.

To calculate the angle of turns, we used the Inertial Motion Unit (IMU) to obtain the heading of the robot. We obtain the angular velocity and acceleration from the x,y,z axis. This allows us to calculate the Roll, Pitch and Yaw of the robot with a dandy library developed by Sebastian Madgwick.

We used this to allow our robot to maintain its heading during straight line movement. We implemented a proportional controller to the servo motor to correct the heading of the robot when it experienced a drift in its heading.

However, Gyroscopes are susceptible to drift, which we can minimise by using the Accelerometer to correct for, however it is not possible to eliminate it. This results in the drifting of the Yaw values produced.

However, we were able to minimise this effect by only taking the reference heading just before we start the turn. This allowed us to minimise the effects of the drift as the error accumulated during the duration of the turn is negligible.

This has allowed us to provide sufficiently accurate movements for completion of the task.

# Video Demo 

<a href="https://youtu.be/U28C2J3W7Tk" target="_blank">Youtube Link</a>

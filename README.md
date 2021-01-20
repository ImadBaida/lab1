# Lab 1 - Wall Follower

**Code due: Immediately before your demo(s), Tuesday 26 January at the latest**

**Report due: Thursday 28 January, 23:59 EDT (Montréal time)**

This is the repository that contains the required files for the wall following lab.
For lab objectives, demo and report requirements, and submission instructions, see
detailed instructions on MyCourses.

## Implementation details

Although you only need to implement one method, `controller()` in [`Lab1.java`](controllers/Lab1/Lab1.java),
you should still go over the other code to understand the program structure, which we
briefly outline below:

[`Lab1.java`](controllers/Lab1/Lab1.java) is the main entry point for your program.
The main method sets up the program (including running Webots physics steps) and starts the
ultrasonic controller.

The ultrasonic controller controls the robot's movements based on data from the ultrasonic sensor.
It is run in an infinite loop which continuously does the following:
  * Reads the distance from the ultrasonic sensor and calls your `controller()` method, which
    calculates and updates the motor speeds based on that distance. **This is what you need to implement.**
  * Sets the motor speeds to what you just calculated. This already done in `setMotorSpeeds()`,
    so you should **not** set them again in the `controller()` method.

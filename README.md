## Welcome!
This GitHub repository is designed to run on a standard mecanum FTC robot and allows drivers to use field-centric driving as well as a mode that prevents the robot from hitting the outside field walls.

## How to Use
First, you should set up your own Android Studio project with the most up-to-date FTC base code (this repository uses the base code from Freight Frenzy). All of the files that have been created or changed in this repository are inside the `TeamCode/src/main/java/org/firstinspires/ftc/teamcode` folder, which is then split into the different parts of this program.

You can just copy over the entire `teamcode` folder into your own project, but it's best if you also understand all of the working parts in this code. A full explanation is below.

Then, before you run the code, make sure to change the constants in the constants file to match your robot's size and the size of the field.

## How it Works
There are a few different parts of this code that help it work and make it easier to edit. The first and most noticeable from a first look is the abstraction.

### Abstraction
Abstraction is the separation of programs into a base and several different iterations of that base. If several OpModes, for example, use the same chassis code and only change some other functions, the code for the chassis and its functions can be moved to its own file, where it only needs to be written once. The OpMode files then create a chassis object that they use to run its methods like the `init` and `run` methods.

This project uses the abstraction code from [Atomic Theory's Freight Frenzy code](https://github.com/charliegarfield/AtomicTheory21-22/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode), which has several instances and sometimes several layers of abstraction. Most notably, the chassis used in all of the main OpModes is a `mecanumChassis`, which is one kind of `chassis`, which in turn is a `mechanism`, one of the several on the robot. However, `Mechanism` is an interface while `Chassis` is an abstract class. This is because each implementation of the `Mechanism` interface only shares the `init` and `run` methods by name (with different versions for each implementation), whereas each different chassis type uses the same base declaration and initialization of the four wheel motors.

It's a bit difficult to see why this is helpful in a small project like this one with only one OpMode file and only one mechanism, but with several OpModes and several mechanisms, this abstraction style speeds up the creation process quite a bit. The Atomic Theory code even uses abstraction in the autonomous files, as the red and blue files are very similar in their basic structure.

### Road Runner
This code uses Road Runner as its central information about the robot's position on the field. The two files used for this are both in the `drive` folder and are `SampleMecanumDrive.java` and `StandardTrackingWheelLocalizer.java`. Road Runner has [its own guide](https://learnroadrunner.com/) to start using it, but the main difference in this code is that it sets the localizer to the tracking wheel localizer, which uses the three dead wheels you put on the robot to determine its position and angular heading. Road Runner also has [its own guide](https://learnroadrunner.com/dead-wheels.html) to using that localizer.

Road Runner is most commonly used for driving trajectories in the autonomous period, but it can also be used in the TeleOp period, as it is used here. A `SampleMecanumDrive` object is initialized as part of the chassis (it is called `driver` in this code because there is also a `drive` variable later). In each loop during the runtime, the `run` function of the declared `MecanumChassis` is called, and at the end, we call `driver.update()`, which updates the drive's knowledge of the dead wheels and in turn updates its position estimate. This creates a live reading of the robots position at all times during TeleOp.

This positional knowledge is extremely helpful and allows us to have field-centric driving and wall avoidance.

### Field-Centric Driving
Field-centric driving is a mode where the robot's motion is executed relative to the field, as opposed to the robot. Holding up on the movement stick, for example, would normally make the robot move forward relative to itself, but now it makes the robot move away from the person driving it, which is forward relative to the driver and to the field. To achieve this motion, the code uses the heading obtained from the dead wheel localization to rotate the drive and strafe vectors from the field-centric input given by the controller to the robot-centric output the robot needs to move correctly.

To calculate these adjusted values, the code first creates an intermediate variable `headingDiff`, which is the robot's heading minus 90 degrees. We need this value because we want the robot's angular difference from the vertical, as opposed to the original value, which is the robot's angluar difference from the horizontal. With this adjusted angluar value, we can apply the rotation matrix at that angle to the drive and strafe vectors to get our new rotated values.

### Wall Avoider
The wall avoider is the most calculation-heavy part of the code. I would reccomend going through each step in the code and verifying for yourself how each step is calculated.

The wall avoider starts by calculating the motion vectors at each corner of the robot. These are all calculated slightly differently, since although drive and strafe are always forward and right, respectively, turn is always clockwise, which is in a different direction at each corner. Once these values are calculated, they are translated into vectors relative to the field by finding the magnitude with the pythagorean theorem and using inverse tangent and the robot's heading to find the correct angle.

Next, the wall avoider calculates the distance from each corner to the field walls, in both the x and y directions. The robot knows the location of the field walls from having them set in the constants file, and it can calculate the location of each corner based on its knowledge of its size (also from the constants file) and its heading. Once it calculates the distance from each corner to the walls, it can adjust its motion accordingly.

On lines 229 to 231 of `MecanumChassis.java`, there are three constants that you can adjust to change the wall avoidance. The first is the distance that the robot starts slowing down at. At this distance or smaller, the robot will slow down exponentially relative to how far the robot is from the wall. You can also adjust this exponent, which changes the intensity of the slowdown. Finally, you can set a minimum distance the robot is allowed to be from the wall. Without this minimum distance, the robot would only completely stop if it were right at the wall. With the minimum distance, the robot slows to a stop at that distance, and actively moves away from the wall if the robot gets any closer.

The robot uses these constants to adjust the x and y components of the motion vectors at each corner, and then it turns these adjusted values back into drive, strafe, and turn values. Importantly, it takes the minimum value of all the corners to determine these values, so that it does not accidentally run into a wall when one corner is too close. Since this code and the field-centric adjustment code both have inputs as and return values the robot can use, both can be run in the same loop to have field-centric driving and wall avoidance at the same time in addition to being able to run each individually.

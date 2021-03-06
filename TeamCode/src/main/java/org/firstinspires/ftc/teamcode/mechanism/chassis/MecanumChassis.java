package org.firstinspires.ftc.teamcode.mechanism.chassis;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.FIELD_LENGTH;
import static org.firstinspires.ftc.teamcode.Constants.FIELD_WIDTH;
import static org.firstinspires.ftc.teamcode.Constants.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.Constants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.Constants.SENSITIVITY;
import static org.firstinspires.ftc.teamcode.Constants.SLOW_MODE_SENSITIVITY;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class MecanumChassis extends Chassis {
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    SampleMecanumDrive driver;

    //imu:
    public BNO055IMU imu;

    Orientation angles;
    Orientation lastAngles = new Orientation();
    public double globalAngle;

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        driver = new SampleMecanumDrive(hardwareMap);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set all motors to 0 power for initialization
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //IMU Setup:
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Define and Initialize BNO055IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void run(Gamepad gamepad){
        run(gamepad,true,true, null);
    }

    @Override
    public void run(Gamepad gamepad, boolean fieldCentric, boolean wallAvoider, Telemetry telemetry){
        double drive = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        if(fieldCentric) {
            double[] rotation = rotateVectors(drive, strafe);
            drive = rotation[0];
            strafe = rotation[1];
        }

        if(wallAvoider) {
            double[] adjustment = adjustVectors(drive, strafe, turn, telemetry);
            drive = adjustment[0];
            strafe = adjustment[1];
            turn = adjustment[2];
        }

        double flPower = (drive + strafe + turn) * DRIVE_STICK_THRESHOLD;
        double frPower = (drive - strafe - turn) * DRIVE_STICK_THRESHOLD;
        double blPower = (drive - strafe + turn) * DRIVE_STICK_THRESHOLD;
        double brPower = (drive + strafe - turn) * DRIVE_STICK_THRESHOLD;

        // Get the maximum value of each power value.
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));

        // If maxPower is less than 1, make it 1. This allows for slower movements.
        maxPower = Math.max(maxPower, 1);

        // Make all of them proportional to the greatest value and factor in the sensitivity.
        flPower = (flPower / maxPower) * SENSITIVITY;
        frPower = (frPower / maxPower) * SENSITIVITY;
        blPower = (blPower / maxPower) * SENSITIVITY;
        brPower = (brPower / maxPower) * SENSITIVITY;

        // Actually set them

        if (gamepad.left_trigger > TRIGGER_THRESHOLD) {
            frontLeft.setPower(flPower * SLOW_MODE_SENSITIVITY);
            frontRight.setPower(frPower * SLOW_MODE_SENSITIVITY);
            backLeft.setPower(blPower * SLOW_MODE_SENSITIVITY);
            backRight.setPower(brPower * SLOW_MODE_SENSITIVITY);
        }
        else {
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
        }

        driver.update();
    }

    // Adjustment for Field Centric Driving
    public double[] rotateVectors(double drive, double strafe) {
        double heading = driver.getPoseEstimate().getHeading();
        double headingDiff = heading - Math.PI / 2;
        double adjustedDrive = drive * Math.cos(headingDiff) - strafe * Math.sin(headingDiff);
        double adjustedStrafe = strafe * Math.cos(headingDiff) + drive * Math.sin(headingDiff);
        return new double[] {adjustedDrive, adjustedStrafe};
    }

    // Adjustment for Wall Avoider
    public double[] adjustVectors(double drive, double strafe, double turn, Telemetry telemetry) {
        Pose2d pos = driver.getPoseEstimate();
        double x = pos.getX();
        double y = pos.getY();
        double heading = pos.getHeading();

        double turnCoeff = 1 / Math.sqrt(2);

        // 0 = front left corner, 1 = front right corner, 2 = back left corner, 3 = back right corner
        /*
            Drive's vector is forward, strafe's vector is to the right, and
            turn's is perpendicular to its path, which is at 45 degrees to
            both drive and strafe, pointed clockwise, hence the turn coefficient.
         */
        double[] forwardMotion = {
                drive + turn * turnCoeff,
                drive - turn * turnCoeff,
                drive + turn * turnCoeff,
                drive - turn * turnCoeff
        };
        double[] rightMotion = {
                strafe + turn * turnCoeff,
                strafe + turn * turnCoeff,
                strafe - turn * turnCoeff,
                strafe - turn * turnCoeff
        };

        // Translate into vectors with direction and magnitude:
        double[] cornerMagnitudes = {
                Math.sqrt(forwardMotion[0] * forwardMotion[0] + rightMotion[0] * rightMotion[0]),
                Math.sqrt(forwardMotion[1] * forwardMotion[1] + rightMotion[1] * rightMotion[1]),
                Math.sqrt(forwardMotion[2] * forwardMotion[2] + rightMotion[2] * rightMotion[2]),
                Math.sqrt(forwardMotion[3] * forwardMotion[3] + rightMotion[3] * rightMotion[3])
        };

        // Math.atan only gives a value between -pi/2 and pi/2, so we need 2 cases:
        double[] angles = {
                Math.atan(rightMotion[0] / Math.abs(forwardMotion[0])),
                Math.atan(rightMotion[1] / Math.abs(forwardMotion[1])),
                Math.atan(rightMotion[2] / Math.abs(forwardMotion[2])),
                Math.atan(rightMotion[3] / Math.abs(forwardMotion[3]))
        };
        double[] cornerHeadings = {
                forwardMotion[0] > 0 ? heading - angles[0] : heading + Math.PI + angles[0],
                forwardMotion[1] > 0 ? heading - angles[1] : heading + Math.PI + angles[1],
                forwardMotion[2] > 0 ? heading - angles[2] : heading + Math.PI + angles[2],
                forwardMotion[3] > 0 ? heading - angles[3] : heading + Math.PI + angles[3]
        };

        double wallXVal = (x > 0) ? FIELD_WIDTH / 2 : -FIELD_WIDTH / 2; // The x position cap
        double wallYVal = (y > 0) ? FIELD_LENGTH / 2 : -FIELD_LENGTH / 2; // The y position cap

        double headingCos = Math.cos(heading);
        double headingSin = Math.sin(heading);

        // x and y distances from the walls to each corner of the robot (these can be negative):
        double[] cornerDistsX = {
                wallXVal - (x + 0.5 * (ROBOT_LENGTH * headingCos - ROBOT_WIDTH * headingSin)),
                wallXVal - (x + 0.5 * (ROBOT_LENGTH * headingCos + ROBOT_WIDTH * headingSin)),
                wallXVal - (x + 0.5 * (-ROBOT_LENGTH * headingCos - ROBOT_WIDTH * headingSin)),
                wallXVal - (x + 0.5 * (-ROBOT_LENGTH * headingCos + ROBOT_WIDTH * headingSin))
        };
        telemetry.addData("X corner distance 0", cornerDistsX[0]);
        telemetry.addData("X corner distance 1", cornerDistsX[1]);
        telemetry.addData("X corner distance 2", cornerDistsX[2]);
        telemetry.addData("X corner distance 3", cornerDistsX[3]);
        double[] cornerDistsY = {
                wallYVal - (y + 0.5 * (ROBOT_LENGTH * headingSin + ROBOT_WIDTH * headingCos)),
                wallYVal - (y + 0.5 * (ROBOT_LENGTH * headingSin - ROBOT_WIDTH * headingCos)),
                wallYVal - (y + 0.5 * (-ROBOT_LENGTH * headingSin + ROBOT_WIDTH * headingCos)),
                wallYVal - (y + 0.5 * (-ROBOT_LENGTH * headingSin - ROBOT_WIDTH * headingCos))
        };
        telemetry.addData("Y corner distance 0", cornerDistsY[0]);
        telemetry.addData("Y corner distance 1", cornerDistsY[1]);
        telemetry.addData("Y corner distance 2", cornerDistsY[2]);
        telemetry.addData("Y corner distance 3", cornerDistsY[3]);

        // scaling for smooth slowdown to the wall, starting slowdown at 30 inches away, but scaled by speed
        double slowDist = 30.0;
        int intensity = 5; // the higher the intensity, the more aggressively it slows you down
        double minDist = 1; // the minimum allowed distance; the robot will be stopped past this
        double[] xScalars = {
                Math.abs(cornerDistsX[0]) < minDist ? -0.5 : Math.min(Math.pow(Math.abs((cornerDistsX[0] - minDist) / (slowDist)), intensity), 1),
                Math.abs(cornerDistsX[1]) < minDist ? -0.5 : Math.min(Math.pow(Math.abs((cornerDistsX[1] - minDist) / (slowDist)), intensity), 1),
                Math.abs(cornerDistsX[2]) < minDist ? -0.5 : Math.min(Math.pow(Math.abs((cornerDistsX[2] - minDist) / (slowDist)), intensity), 1),
                Math.abs(cornerDistsX[3]) < minDist ? -0.5 : Math.min(Math.pow(Math.abs((cornerDistsX[3] - minDist) / (slowDist)), intensity), 1)
        };
        double[] yScalars = {
                Math.abs(cornerDistsY[0]) < minDist ? -0.5 : Math.min(Math.pow(Math.abs((cornerDistsY[0] - minDist) / (slowDist)), intensity), 1),
                Math.abs(cornerDistsY[1]) < minDist ? -0.5 : Math.min(Math.pow(Math.abs((cornerDistsY[1] - minDist) / (slowDist)), intensity), 1),
                Math.abs(cornerDistsY[2]) < minDist ? -0.5 : Math.min(Math.pow(Math.abs((cornerDistsY[2] - minDist) / (slowDist)), intensity), 1),
                Math.abs(cornerDistsY[3]) < minDist ? -0.5 : Math.min(Math.pow(Math.abs((cornerDistsY[3] - minDist) / (slowDist)), intensity), 1)
        };

        // adjust motion according to the scalars
        double[] adjustedXVels = {
                Math.cos(cornerHeadings[0]) * x > 0 ? cornerMagnitudes[0] * Math.cos(cornerHeadings[0]) * xScalars[0] : cornerMagnitudes[0] * Math.cos(cornerHeadings[0]),
                Math.cos(cornerHeadings[1]) * x > 0 ? cornerMagnitudes[1] * Math.cos(cornerHeadings[1]) * xScalars[1] : cornerMagnitudes[1] * Math.cos(cornerHeadings[1]),
                Math.cos(cornerHeadings[2]) * x > 0 ? cornerMagnitudes[2] * Math.cos(cornerHeadings[2]) * xScalars[2] : cornerMagnitudes[2] * Math.cos(cornerHeadings[2]),
                Math.cos(cornerHeadings[3]) * x > 0 ? cornerMagnitudes[3] * Math.cos(cornerHeadings[3]) * xScalars[3] : cornerMagnitudes[3] * Math.cos(cornerHeadings[3]),
        };
        double[] adjustedYVels = {
                Math.sin(cornerHeadings[0]) * y > 0 ? cornerMagnitudes[0] * Math.sin(cornerHeadings[0]) * yScalars[0] : cornerMagnitudes[0] * Math.sin(cornerHeadings[0]),
                Math.sin(cornerHeadings[1]) * y > 0 ? cornerMagnitudes[1] * Math.sin(cornerHeadings[1]) * yScalars[1] : cornerMagnitudes[1] * Math.sin(cornerHeadings[1]),
                Math.sin(cornerHeadings[2]) * y > 0 ? cornerMagnitudes[2] * Math.sin(cornerHeadings[2]) * yScalars[2] : cornerMagnitudes[2] * Math.sin(cornerHeadings[2]),
                Math.sin(cornerHeadings[3]) * y > 0 ? cornerMagnitudes[3] * Math.sin(cornerHeadings[3]) * yScalars[3] : cornerMagnitudes[3] * Math.sin(cornerHeadings[3]),
        };

        // rotate to get the robot-centric motion (the same as the rotateVectors method)
        double headingDiff = heading - Math.PI / 2;
        forwardMotion[0] = adjustedYVels[0] * Math.cos(headingDiff) - adjustedXVels[0] * Math.sin(headingDiff);
        rightMotion[0] = adjustedXVels[0] * Math.cos(headingDiff) + adjustedYVels[0] * Math.sin(headingDiff);
        forwardMotion[1] = adjustedYVels[1] * Math.cos(headingDiff) - adjustedXVels[1] * Math.sin(headingDiff);
        rightMotion[1] = adjustedXVels[1] * Math.cos(headingDiff) + adjustedYVels[1] * Math.sin(headingDiff);
        forwardMotion[2] = adjustedYVels[2] * Math.cos(headingDiff) - adjustedXVels[2] * Math.sin(headingDiff);
        rightMotion[2] = adjustedXVels[2] * Math.cos(headingDiff) + adjustedYVels[2] * Math.sin(headingDiff);
        forwardMotion[3] = adjustedYVels[3] * Math.cos(headingDiff) - adjustedXVels[3] * Math.sin(headingDiff);
        rightMotion[3] = adjustedXVels[3] * Math.cos(headingDiff) + adjustedYVels[3] * Math.sin(headingDiff);

        // now adjust the actual drive, strafe, and turn values to the minimum allowed value
        double[] turnPossibilities = {
                0.5 * (forwardMotion[0] - forwardMotion[1]),
                0.5 * (forwardMotion[0] - forwardMotion[3]),
                0.5 * (forwardMotion[2] - forwardMotion[1]),
                0.5 * (forwardMotion[2] - forwardMotion[3]),
                0.5 * (rightMotion[0] - rightMotion[2]),
                0.5 * (rightMotion[0] - rightMotion[3]),
                0.5 * (rightMotion[1] - rightMotion[2]),
                0.5 * (rightMotion[1] - rightMotion[3])
        };
        int minIndex = 0;
        double minVal = Math.abs(turnPossibilities[0]);
        for(int i = 1; i < turnPossibilities.length; i++) {
            if(Math.abs(turnPossibilities[i]) < minVal) {
                minVal = Math.abs(turnPossibilities[i]);
                minIndex = i;
            }
        }
        double turnWithCoeff = turnPossibilities[minIndex];

        double[] drivePossibilities = {
                forwardMotion[0] - turnWithCoeff,
                forwardMotion[1] + turnWithCoeff,
                forwardMotion[2] - turnWithCoeff,
                forwardMotion[3] + turnWithCoeff
        };
        minIndex = 0;
        minVal = Math.abs(drivePossibilities[0]);
        for(int i = 1; i < drivePossibilities.length; i++) {
            if(Math.abs(drivePossibilities[i]) < minVal) {
                minVal = Math.abs(drivePossibilities[i]);
                minIndex = i;
            }
        }
        drive = drivePossibilities[minIndex];

        double[] strafePossibilities = {
                rightMotion[0] - turnWithCoeff,
                rightMotion[1] - turnWithCoeff,
                rightMotion[2] + turnWithCoeff,
                rightMotion[3] + turnWithCoeff
        };
        minIndex = 0;
        minVal = Math.abs(strafePossibilities[0]);
        for(int i = 1; i < strafePossibilities.length; i++) {
            if(Math.abs(strafePossibilities[i]) < minVal) {
                minVal = Math.abs(strafePossibilities[i]);
                minIndex = i;
            }
        }
        strafe = strafePossibilities[minIndex];

        return new double[] {drive, strafe, turnWithCoeff / turnCoeff};
    }

    // Position functions
    public double[] getPos(){

        Pose2d pos = driver.getPoseEstimate();

        return new double[] {pos.getX(), pos.getY(), pos.getHeading()};
    }

    public void setPos(double x, double y, double heading) {
        driver.setPoseEstimate(new Pose2d(x, y, heading));
    }

    // Movement functions
    public void delay(int time) {
        double startTime = elapsedTime.milliseconds();
        while (elapsedTime.milliseconds() - startTime < time) {
        }
    }

    //IMU Functions:

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     * @param power   Motor power during turn (should not be negative)
     */
    public void rotate(double degrees, double power) throws InterruptedException {

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            power = -power;

        } else if (degrees > 0) {   // turn left.
            //return; //power = power; dont need to change anything because assuming the power is already positive, that should make the robot turn right when put into the turn function
        } else return;

        // set power to rotate.
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(-power);
        backLeft.setPower(-power);

        double tempPower;
        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
            }

            while (getAngle() > degrees) {
                tempPower = power * (getAngle() - degrees) / degrees * -1 - 0.1;
                frontRight.setPower(tempPower);
                backRight.setPower(tempPower);
                frontLeft.setPower(-tempPower);
                backLeft.setPower(-tempPower);
            }
        } else {  // left turn.
            while (getAngle() < degrees) {
                tempPower = power * (degrees - getAngle()) / degrees + 0.1;
                frontRight.setPower(tempPower);
                backRight.setPower(tempPower);
                frontLeft.setPower(-tempPower);
                backLeft.setPower(-tempPower);
            }
        }
        // turn the motors off.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        Thread.sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Rotates to a global heading using IMU
     *
     * @param degrees
     * @param power
     * @throws InterruptedException
     */
    public void rotateToGlobalAngle(int degrees, double power) throws InterruptedException {

        // get the initial angle of rotation.
        double initialAngle = getGlobalAngle();
        double deltaDegrees = degrees - initialAngle;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (deltaDegrees < 0) {   // turn right.
            power = -power;
        }

        // set power to rotate.
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(-power);
        backLeft.setPower(-power);

        double tempPower;
        // rotate until turn is completed.
        if (deltaDegrees < 0) {
            // right turn
            while (getGlobalAngle() > degrees && degrees - getGlobalAngle() >= deltaDegrees - 10) {
                tempPower = power * (getGlobalAngle() - degrees) / deltaDegrees + 0.1;
                frontRight.setPower(-tempPower);
                backRight.setPower(-tempPower);
                frontLeft.setPower(tempPower);
                backLeft.setPower(tempPower);
            }
        } else {
            // left turn.
            while (getGlobalAngle() < degrees && degrees - getGlobalAngle() <= deltaDegrees + 10) {
                tempPower = power * (degrees - getGlobalAngle()) / deltaDegrees + 0.1;
                frontRight.setPower(tempPower);
                backRight.setPower(tempPower);
                frontLeft.setPower(-tempPower);
                backLeft.setPower(-tempPower);
            }
        }

        // turn the motors off.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        Thread.sleep(100);

        // reset angle tracking on new heading.
        resetAngle();

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Get current heading relative to intialization angle (i think)
     *
     * @return Current heading / angle from -180 to 180
     */
    public double getGlobalAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}

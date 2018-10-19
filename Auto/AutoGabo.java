package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

//Made by Gabo on Oct 15 2018

@Autonomous(name = "AutoGabo", group = "Auto")
public class AutoGabo extends LinearOpMode {
    public HardwareMap hwMap; //declare the hardware map

    public DcMotor BLM; //declare motors
    public DcMotor BRM;
    public DcMotor FRM;
    public DcMotor FLM;

    ModernRoboticsI2cRangeSensor rangeSensor; //declares range sensor

    public TouchSensor LTS; //declare touch sensors
    public TouchSensor RTS;


    BNO055IMU imu; //declare imu

    // State used for updating telemetry
    Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    double globalAngle; //the number of degrees the robot has turned

    double power = 0.30; //power of the robot
    double minPower = 0.2; //least amount of power the robot can have
    int beginAngle = -83; //the number of degrees the robot should turn. Set to -83 to turn right 90 degrees. Latency is a pain

    public void runOpMode() //when you press init
    {
        hwMap = hardwareMap; //makes the hardwaremap = to hwMap. This is for easier getting

        BLM = hwMap.dcMotor.get("BLM"); //gets all the motors
        BRM = hwMap.dcMotor.get("BRM");
        FRM = hwMap.dcMotor.get("FRM");
        FLM = hwMap.dcMotor.get("FLM");

        RTS = hwMap.touchSensor.get("RTS"); //gets the 2 touch sensors
        LTS = hwMap.touchSensor.get("LTS");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RS"); //gets range sensor

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets the imu

        telemetry.addData("Working?", " ye"); //very informative telemetry
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        telemetry.addData("Mode", "initalizing...");
        telemetry.update();

        imu.initialize(parameters); //initalizes the imu

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            telemetry.addData("Mode", " calibrating..."); //inform
            telemetry.update();
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();




        waitForStart(); //waits for the start button





        telemetry.addData("Mode", "running");
        telemetry.update();

            ///RushB(); //uses touch sensor
            RushA(); //uses distance sensor

            moveFor(minPower, -minPower, 300); // backup for 300 ms

        rotate(beginAngle, power); //rotates to the right 90 degrees

        RushA(); //moves to the next wall

        move(0,0); //turns off
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //gets the angle

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle; //deltaAngle is the first angle - the lastangle it got

        if (deltaAngle < -180) //switches it to use 0 to 360
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle; //adds the deltaAngle to the globalAngle

        lastAngles = angles; //lastAngle is the anlges

        return globalAngle; //returns the amount turned
    }
    /**
     *  THIS HAS NOT BEEN TESTED YET.
     */
public void RushA () //moves with distance sensor. Slowly getting slower and slower
{
    double endDistance = 13; //stops when distance to wall is 13cm
    double maxPowerLevel = 0.5; //can only move a max of 0.5 power
    double minPowerLevel = 0.15; //can't go slower than 0.15 power
    double startDistance = rangeSensor.getDistance(DistanceUnit.CM); //gets the distance to the wall
    double distanceTraveled = 0; //sets the distance traveled to 0
    double totalNeededToTravel = startDistance - endDistance; //gets the total distance the robot needs to move
    double lastDistance = startDistance; //last distance is first distance


    while (rangeSensor.getDistance(DistanceUnit.CM) > endDistance && opModeIsActive()) //while op mode is running and the distance to the wall is greater than the end distance
    {
        double currentDistance = rangeSensor.getDistance(DistanceUnit.CM); //gets the current distance to the wall

        double deltaDistance = lastDistance - currentDistance; //change in distance is the last distance - currentDistance

        distanceTraveled += deltaDistance; //adds the change in distance to distance traveled

        lastDistance = currentDistance; //the last distance is set to the current distance

        telemetry.addData("Range", currentDistance); //tell them the range
        telemetry.update();

        double slope = -maxPowerLevel / (Math.pow(totalNeededToTravel, 3)); //gets the slope of the graph that is needed to make y = 0 when totalNeeded to travel is x

        double power = slope * Math.pow(distanceTraveled, 3) + maxPowerLevel; // the power is the x value in that position

        if (power < minPowerLevel) power = minPowerLevel; //if the power is less than the min power level just set the power to the minpower level
        if (power == 0) power = 0; //if its 0 then set it to 0 of course
        move(-power, power); //moves the robot forward with whatever the power is
    }
    move(0,0); //once done stop

}
public void moveFor(double leftPower, double rightPower, int time)
{
    move(leftPower, rightPower); //move robot
    sleep(time); //don't do anything for set time
    move(0,0); //stop robot
}

    /** THIS HAS NOT BEEN TESTED YET
     *
     */
    public void RushB() //move to wall with touch sensors
    {
        while (!RTS.isPressed() || !LTS.isPressed()) //while either of the sensors are not pressed
        {
            double left = 0.5; //left power is 0.5
            double right = -0.5; //right power is 0.5

            if (RTS.isPressed()) //if right sensor is pressed
            {
                right = 0; //sets right to 0 to stop
                left = -0.3; //left slows down
                telemetry.addData("Right Button", RTS.isPressed()); //informs that its pressed
                telemetry.update();
            }
            if (LTS.isPressed()) //if left sensor is pressed
            {
                left = 0; //left stops
                right = -0.3; //right is stopped
                telemetry.addData("Left Button", LTS.isPressed()); //informs that its pressed
                telemetry.update();
            }

            move(left, right); //moves motors with right left power

        }


        move(0,0); //once both motors are pressed it stops
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        telemetry.addData("Rotating", true); //informs
        telemetry.update();
        double  leftPower, rightPower; //declares the powers

        resetAngle(); //sets starting angle and resets the amount turned to 0

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).

        if (degrees < 0)// turn right.
        {
            leftPower = -power; //both negative
            rightPower = -power;
        }
        else if (degrees > 0)// turn left.
        {
            leftPower = power; //both positive
            rightPower = power;
        }
        else return; //do nothing

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                move(leftPower, rightPower);
            }

            while (opModeIsActive() && getAngle() > degrees) {
                move(leftPower, rightPower);
            } //once it starts turning slightly more than it should.
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                move(leftPower, rightPower);
            }

        // turn the motors off.
        move(0,0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //sets lastAngles to current angles

        globalAngle = 0; //global angle is set to 0
    }

    public void move(double leftPower, double rightPower) //move the robot
    {
        BLM.setPower(leftPower); //sets left motors to left power
        FLM.setPower(leftPower);
        FRM.setPower(rightPower); //sets right motors to right power
        BRM.setPower(rightPower);
    }
/**
 * GRAVEYARD
 */
//    void composeTelemetry() {
//
//        // At the beginning of each telemetry update, grab a bunch of data
//        // from the IMU that we will then display in separate lines.
//        telemetry.addAction(new Runnable() { @Override public void run()
//        {
//            // Acquiring the angles is relatively expensive; we don't want
//            // to do that in each of the three items that need that info, as that's
//            // three times the necessary expense.
//            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gravity  = imu.getGravity();
//        }
//        });
//
//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel*gravity.xAccel
//                                        + gravity.yAccel*gravity.yAccel
//                                        + gravity.zAccel*gravity.zAccel));
//                    }
//                });
//    }
//
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
//
//    String formatDegrees(double degrees){
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//        double beginningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        telemetry.addData("Beginning Angle", beginningAngle);
//        telemetry.update();
//        double endAngle = 90;
//        double maxPower = 0.3;
//
//        double slope = -maxPower / (Math.pow(endAngle, 3));
//        telemetry.addData("Slope", slope);
//        telemetry.update();
//        boolean finished = false;
//        move(-0.3, -0.3);
//
//        while (!finished && opModeIsActive())
//        {
//            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - beginningAngle;
//            telemetry.addData("Current Angle", currentAngle);
//            telemetry.update();
//            if (currentAngle == endAngle) finished = true;
//            else
//            {
//                double power = slope * Math.pow(currentAngle, 3) + maxPower;
//                telemetry.addData("Power", power);
//                telemetry.update();
//                move(-power, -power);
//            }
//        }
//        move(0,0);

}

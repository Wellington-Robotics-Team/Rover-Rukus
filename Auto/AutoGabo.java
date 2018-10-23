package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
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

    public DistanceSensor rightRangeSensor; //declares range sensors
    public DistanceSensor leftRangeSensor;

    public TouchSensor LTS; //declare touch sensors
    public TouchSensor RTS;

    BNO055IMU imu; //declare imu

    Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    double globalAngle; //the number of degrees the robot has turned

    double power = 0.50; //power of the robot
    double minPower = 0.2; //least amount of power the robot can have
    int turnDegrees = -82; //the number of degrees the robot should turn. Set to -82 to turn right 90 degrees. Latency is a pain

    public void runOpMode() //when you press init
    {
        hwMap = hardwareMap; //makes the hardwaremap = to hwMap. This is for easier getting

        BLM = hwMap.dcMotor.get("BLM"); //gets all the motors
        BRM = hwMap.dcMotor.get("BRM");
        FRM = hwMap.dcMotor.get("FRM");
        FLM = hwMap.dcMotor.get("FLM");

        //RTS = hwMap.touchSensor.get("RTS"); //gets the 2 touch sensors
        //LTS = hwMap.touchSensor.get("LTS");

        rightRangeSensor = hardwareMap.get(DistanceSensor.class, "RRS"); //gets range sensors
        leftRangeSensor = hardwareMap.get(DistanceSensor.class, "LRS");

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

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
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

        RushA(20); //uses distance sensor to drive forward

        align(); //align the robot with the wall

        rotate(turnDegrees, power); //rotates to the right 90 degrees

        RushA(20); //moves to the next wall

        rotate(turnDegrees, power); //rotates to the right 90 degrees

        RushC(); //moves to the crater

        move(0, 0); //turns off
    }

    /**
     * Gets the 2 distances the sensors read
     * Will return them in an array where position 0 is the left distance and position 1 is the right distance
     * 0 = left
     * 1 = right
     * @return
     */
    public double[] getDistances ()
    {
        double leftDistance = leftRangeSensor.getDistance(DistanceUnit.CM); //left sensor distance
        double rightDistance = rightRangeSensor.getDistance(DistanceUnit.CM); //right sensor distance
        double[] distances = new double[2]; //makes an array
        distances[0] = leftDistance; //sets positions to values
        distances[1] = rightDistance;
        return distances; //returns the array
    }

    /**
     * Will return - if robot is too far right
     * will return + if robot is too far left
     * will return 0 if robot is perfect
     * @return
     */
    public double getDeltaRange ()
    {
        double[] distances = getDistances(); //gets the distances
        double deltaRange = distances[0] - distances[1]; //subtracts the 2
        return deltaRange; //returns the number
    }

    /**
     * Aligns itself with the wall
     * doesn't work yet. DUh
     */
    public void align()
    {
        double accuracy = 10; //how accurate this should be to the CM

        double deltaRange = getDeltaRange();// dets the change in distance between the 2 sensors

        while (Math.abs(deltaRange) > accuracy) //while the delta is greater than the accuracy we want
        {
            if (deltaRange < 0) //if its negative
            {
                move(minPower,minPower);//turn left
                getDeltaRange(); //regets the delta range

            }
            else if (deltaRange > 0) //if the delta range is positive
            {
                move(-minPower, -minPower);//turn right
                getDeltaRange(); //regets the delta range
            }
        }
        move(0,0); //stops
    }

    /**
     * Rush to the crater
     * Vry nice
     */
    public void RushC()
    {
        telemetry.addData("Currently: ", "Rushing A");
        telemetry.addData("Status: ", "Workin");
        telemetry.update();

        RushA(200); //get 200 cm from the crater

        telemetry.addData("Status: ", "Finished");
        telemetry.update();

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle; //gets the roll of the robot

        telemetry.addData("Current Angle: ", currentAngle);

        while (currentAngle < 75) //while the current roll is less than 75
        {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle; //regets the currentAngle
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.update();
            move(-minPower, minPower); //continue moving
        }

        move(0,0); //stop
        telemetry.addData("Finished", true);
        telemetry.update();
    }



    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
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
     * Runs until it gets close to wall
     * slowly gets slower
     */
    public void RushA(double endDistance) //moves with distance sensor. Slowly getting slower and slower
    {
    double maxPowerLevel = power; //can only move a max of 0.5 power
    double minPowerLevel = minPower; //can't go slower than 0.15 power
    double startDistance = rightRangeSensor.getDistance(DistanceUnit.CM); //gets the distance to the wall
    double distanceTraveled = 0; //sets the distance traveled to 0
    double totalNeededToTravel = startDistance - endDistance; //gets the total distance the robot needs to move
    double lastDistance = startDistance; //last distance is first distance


    while (rightRangeSensor.getDistance(DistanceUnit.CM) > endDistance && opModeIsActive()) //while op mode is running and the distance to the wall is greater than the end distance
    {
        double currentDistance = rightRangeSensor.getDistance(DistanceUnit.CM); //gets the current distance to the wall
        telemetry.addData("StartDistance", startDistance);
        telemetry.addData("Needed to travel", totalNeededToTravel);
        telemetry.addData("Current distance to wall",currentDistance);


        double deltaDistance = lastDistance - currentDistance; //change in distance is the last distance - currentDistance

        distanceTraveled += deltaDistance; //adds the change in distance to distance traveled
        telemetry.addData("Distance traveled", distanceTraveled);
        lastDistance = currentDistance; //the last distance is set to the current distance

        double slope = -maxPowerLevel / (Math.pow(totalNeededToTravel, 3)); //gets the slope of the graph that is needed to make y = 0 when totalNeeded to travel is x

        double power = slope * Math.pow(distanceTraveled, 3) + maxPowerLevel; // the power is the x value in that position

        if (power < minPowerLevel && power > 0) power = minPowerLevel; //if the power is less than the min power level just set the power to the minpower level
        if (power == 0) power = 0; //if its 0 then set it to 0 of course
        telemetry.addData("Power", power);
        move(-power, power); //moves the robot forward with whatever the power is
        telemetry.update();
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
            double left = power; //left power is 0.5
            double right = power; //right power is 0.5

            if (RTS.isPressed()) //if right sensor is pressed
            {
                right = 0; //sets right to 0 to stop
                left = -minPower; //left slows down
                telemetry.addData("Right Button", RTS.isPressed()); //informs that its pressed
                telemetry.update();
            }
            if (LTS.isPressed()) //if left sensor is pressed
            {
                left = 0; //left stops
                right = -minPower; //right is stopped
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

    /**
     * move the robot
     * @param leftPower right motor power
     * @param rightPower left motor power
     */
    public void move(double leftPower, double rightPower) //move the robot
    {
        BLM.setPower(leftPower); //sets left motors to left power
        FLM.setPower(leftPower);
        FRM.setPower(rightPower); //sets right motors to right power
        BRM.setPower(rightPower);
    }
/** warning
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

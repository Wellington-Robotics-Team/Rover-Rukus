package org.firstinspires.ftc.robotcontroller.internal;

//import android.media.MediaPlayer;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
//import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;

//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.Servo;

//This auto was made by Gabo Gang on Oct 15 2018

@Autonomous(name = "AutoGabo", group = "Auto")
public class AutoGabo extends LinearOpMode {
    // Detector object
    private SamplingOrderDetector orderDetector;

    private DcMotor BLM; //declare motors
    private DcMotor BRM;
    private DcMotor FRM;
    private DcMotor FLM;
    private DcMotor LIFT;

    private Servo GlyphServo = null;

    private DistanceSensor rightRangeSensor; //declares range sensors
    private DistanceSensor leftRangeSensor;

    // Detector object
    private GoldAlignDetector goldAlignDetector;

    private BNO055IMU imu; //declare imu

    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

//    Acceleration gravity;

    private double globalAngle; //the number of degrees the robot has turned

    private double power = 0.50; //power of the robot
    private double minPower = 0.25; //least amount of power the robot can have
    private int righturn = -81; //degrees that does an accurate 90 degree right turn
    private int leftTurn = 81; //degrees that does an accurate 90 degree left turn
   // private int degreeOffset = 8; //the number of degrees its usually off by when turning
    private int distanceOffset = 8; //how many CM the robot is off by when RushingA
    //MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.undertale);
    public void runOpMode() //when you press init
    {

        //mediaPlayer.start();

        HardwareMap hwMap = hardwareMap; //makes the hardwaremap = to hwMap. This is for easier getting

        // Setup orderDetector
        orderDetector = new SamplingOrderDetector(); // Create the orderDetector
        orderDetector.init(hwMap.appContext, CameraViewDisplay.getInstance()); // Initialize orderDetector with app context and camera
        orderDetector.useDefaults(); // Set orderDetector to use default settings
        orderDetector.downscale = 0.4; // How much to downscale the input frames
        orderDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //orderDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        orderDetector.maxAreaScorer.weight = 0.001;
        orderDetector.ratioScorer.weight = 15;
        orderDetector.ratioScorer.perfectRatio = 1.0;

        // Set up detector
        goldAlignDetector = new GoldAlignDetector(); // Create detector
        goldAlignDetector.init(hwMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        goldAlignDetector.useDefaults(); // Set detector to use default settings
        goldAlignDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldAlignDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldAlignDetector.downscale = 0.4; // How much to downscale the input frames
        goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldAlignDetector.maxAreaScorer.weight = 0.005; //
        goldAlignDetector.ratioScorer.weight = 5; //
        goldAlignDetector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        //Sets up motors
        BLM = hwMap.dcMotor.get("BLM");
        BRM = hwMap.dcMotor.get("BRM");
        FRM = hwMap.dcMotor.get("FRM");
        FLM = hwMap.dcMotor.get("FLM");
        LIFT = hwMap.dcMotor.get("LIFT");

        GlyphServo = hwMap.servo.get("Glyph");

        FLM.setDirection(DcMotor.Direction.REVERSE);
        BLM.setDirection(DcMotor.Direction.REVERSE);

        rightRangeSensor = hwMap.get(DistanceSensor.class, "RRS"); //gets range sensors
        leftRangeSensor = hwMap.get(DistanceSensor.class, "LRS");

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets the imu

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        telemetry.addData("Mode", "initalizing...");
        telemetry.update();

        imu.initialize(parameters); //initalizes the imu

        telemetry.addData("Mode", " calibrating..."); //inform
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        GlyphServo.setDirection(Servo.Direction.REVERSE);
        GlyphServo.setPosition(0.6);

        waitForStart(); //waits for the start button

        winCompetition();
    }

    /**
     * will win any competition
     */
    private void winCompetition()
    {
        telemetry.addData("Mode", "lowering...");
        telemetry.update();
        //mediaPlayer.stop();
        //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.tunak);
        //mediaPlayer.start();
        //Lower
        lower();
        //mediaPlayer.stop();
        //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.leak);
        orderDetector.enable(); // Start orderDetector
        telemetry.addData("Mode: ", "Unhooking");
        telemetry.update();
        //mediaPlayer.start();
        unHook();
        move(0,0);
        telemetry.addData("Mode: ", "Searching for cube...");
        telemetry.update();
        //mediaPlayer.stop();
        //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.skidaddle);
        //mediaPlayer.start();
        //Get cube place
        int cubePlace = findCube();

        telemetry.addData("Mode: ", "Aligning with cube");
        telemetry.update();
        alignWithCube();  //Aligns the robot with the cube
        //mediaPlayer.stop();
        //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.gas);
        //mediaPlayer.start();
        move(power,power);
        sleep(1200);

        telemetry.addData("Mode: ", "Rushing D");
        telemetry.update();
        //mediaPlayer.stop();
        //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.deja);
        //mediaPlayer.start();
        RushD();

        switch (cubePlace)
        {
            case 0: //if left
                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushb);
                //mediaPlayer.start();
                telemetry.addData("Mode: ", "Rushing A");
                telemetry.update();
                RushA(40, minPower); //uses distance sensor to drive forward

                telemetry.addData("Mode: ", "Aligning");
                telemetry.update();
                align(); //align the robot with the wall

                telemetry.addData("Mode: ", "Rotating right");
                telemetry.update();
                rotate(righturn, power); //rotates to the right 90 degrees

                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushf);
                //mediaPlayer.start();
                telemetry.addData("Mode: ", "Rushing A");
                telemetry.update();
                RushA(20, power); //moves to the next wall

                telemetry.addData("Mode: ", "Aligning");
                telemetry.update();
                align();

                //put down glyph
                dropGlyph();

                telemetry.addData("Mode: ", "Rotating");
                telemetry.update();
                rotate(righturn - 1, power); //rotates to the right 90 degrees

                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushe);
               // mediaPlayer.start();
                telemetry.addData("Mode: ", "Rushing C");
                telemetry.update();
                RushC(true); //moves to the crater
                break;
            case 1: //if center
                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushb);
                //mediaPlayer.start();
                telemetry.addData("Mode: ", "Rushing A");
                telemetry.update();
                RushA(80, minPower);

                telemetry.addData("Mode: ", "Rotating left");
                telemetry.update();
                rotate(45, minPower);

                telemetry.addData("Mode: ", "Aligning");
                telemetry.update();
                align();

                telemetry.addData("Mode: ", "Rushing A");
                telemetry.update();
                RushA(30, minPower);

                telemetry.addData("Mode: ", "Aligning");
                telemetry.update();
                align();
                dropGlyph();
                telemetry.addData("Mode: ", "Rotating right");
                telemetry.update();
                rotate(righturn,power);

                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushf);
                //mediaPlayer.start();
                telemetry.addData("Mode: ", "Rushing A");
                telemetry.update();
                RushA(20, power);


                telemetry.addData("Mode: ", "Aligning");
                telemetry.update();
                align();

                telemetry.addData("Mode: ", "Rotating right");
                telemetry.update();
                rotate(righturn - 1, power);

                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushe);
                //mediaPlayer.start();
                telemetry.addData("Mode: ", "Rushing C");
                telemetry.update();
                RushC(true); //moves to the crater
                break;
            case 2: //if right
                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushb);
                //mediaPlayer.start();
                telemetry.addData("Mode: ", "Rushing A");
                telemetry.update();
                RushA(20,minPower);

                align();
                telemetry.addData("Mode: ", "Turning left");
                telemetry.update();
                rotate(leftTurn, minPower);

                telemetry.addData("Mode: ", "Rushing A");
                telemetry.update();

                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushf);
                //mediaPlayer.start();
                RushA(20, minPower);

                telemetry.addData("Mode: ", "Aligning");
                telemetry.update();
                align();

                dropGlyph();
                //mediaPlayer.stop();
                //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.rushe);
                //mediaPlayer.start();
                telemetry.addData("Mode: ", "Rushing C");
                telemetry.update();
                RushC(false); //moves to the crater
                break;

        }
        move(0, 0); //turns off
        //mediaPlayer.stop();
        //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.tracer);
        //mediaPlayer.start();

        telemetry.addData("Mode: ", "Stopped");
        telemetry.update();



        stop();
    }

    /**
     * rotates a servo so that it drops the glyph.
     */
    private void dropGlyph ()
    {
        GlyphServo.setPosition(0.85);
    }
    /**
     * Lower the robot at the start
     */
    private void lower ()
    {
        LIFT.setPower(power);
        sleep(900);
        LIFT.setPower(0);
    }

    /**
     * Will unhook the robot
     */
    private void unHook()
    {
        rotate(20, minPower);

        move(minPower,minPower);
        sleep(500);
        move(0,0);
        rotate(-20, minPower);

        //LIFT.setPower(0);
        move(-minPower,-minPower);
        sleep(500);
        move(0,0);
    }
    /**
     * if x < 280 turn left
     * if x > 280 turn right
     */
    private void alignWithCube ()
    {
        move(0,0);
        goldAlignDetector.enable();
        sleep(2500);
        while (!isStopRequested() && !goldAlignDetector.getAligned())
        {
            double xPos = goldAlignDetector.getXPosition();
            telemetry.addData("IsAligned" , goldAlignDetector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos" , xPos); // Gold X position.
            telemetry.update();

            if (xPos < 300) move(-minPower, minPower);
            else move(minPower, -minPower);
        }
        move(0,0);
        goldAlignDetector.disable();

    }

    /**
     * @return 0 = left, 1 = center, 2 = right
     * Will get the position of the cube
     */
    private int findCube ()
    {
        move(0,0);
        sleep(2500);
        int place; //using an int to store the place instead of just using a string because ints are easier
        while (orderDetector.getCurrentOrder().toString().equals("UNKNOWN") && !isStopRequested()) //not do anything while the order is unknown
        {
            move(minPower,-minPower);
        }
        move(0,0);
        orderDetector.disable();
        switch (orderDetector.getCurrentOrder().toString())
        {
            case "LEFT":
                place = 0;
                break;
            case "CENTER":
                place = 1;
                break;
            default:
                place = 2;
                break;
        }

        return place; //returns the place
    }

    /**
     * Move the robot close enough to the wall so that at least one of the sensors is working
     */
    private void RushD() {
        move(minPower, minPower);
        while (leftRangeSensor.getDistance(DistanceUnit.CM) > 800 && rightRangeSensor.getDistance(DistanceUnit.CM) > 800)
        {
            telemetry.addData("Left distance", leftRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Right distance", rightRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        move(0, 0); // Insert why you use 800 here...
    }


    /**
     * Gets the 2 distances the sensors read
     * Will return them in an array where position 0 is the left distance and position 1 is the right distance
     * 0 = left
     * 1 = right
     *
     * @return an array with the distances read from both distance sensors
     */
    private double[] getDistances() {
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
     *
     * @return the number of degrees its turned since the last degree reset
     */
    private double getDeltaRange() {
        double[] distances = getDistances(); //gets the distances
        return distances[0] - distances[1]; //subtracts the 2
    }

    /**
     * Aligns itself with the wall
     */
    private void align() {

        double accuracy = 1; //how accurate this should be to the CM

        double deltaRange = getDeltaRange();// gets the change in distance between the 2 sensors      +1 because inaccurate


        while (Math.ceil(Math.abs(deltaRange)) > accuracy && !isStopRequested()) //while the delta is greater than the accuracy we want
        {

            telemetry.addData("Status: ", "Aligning");
            deltaRange = getDeltaRange();
            telemetry.addData("Delta Range", deltaRange);
            telemetry.update();
            if (deltaRange < 0) //if its negative
            {
                move(-minPower, minPower);//turn left

            } else if (deltaRange > 0) //if the delta range is positive
            {
                move(minPower, -minPower);//turn right
            }
        }
        move(0, 0); //stops
    }

    /**
     * Rush to the crater
     * Vry nice
     */
    private void RushC(boolean forward) {
        telemetry.addData("Status: ", "Workin");
        telemetry.update();

        telemetry.addData("Status: ", "Finished");
        telemetry.update();

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle; //gets the roll of the robot

        telemetry.addData("Current Angle: ", currentAngle);

        while ((currentAngle < 4 && currentAngle > -1.5) && !isStopRequested()) //while the current roll is less than 72
        {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle; //regets the currentAngle
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.update();
            if (forward)
            {
                move(power, power); //continue moving
            } else
            {
                move(-power, -power); //backwards
            }

        }

        move(0, 0); //stop
        telemetry.addData("Finished", true);
        telemetry.update();
    }


    /**
     * Get current cumulative angle rotation from last reset.
     *
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
     * Runs until it gets close to wall
     * slowly gets slower
     */
    private void RushA(double endDistance, double maxPowerLevel) //moves with distance sensor. Slowly getting slower and slower
    {
        endDistance = endDistance + distanceOffset;
        double minPowerLevel = minPower; //can't go slower than 0.15 power
        boolean leftSensor = false;
        DistanceSensor closerSensor;

        double[] distances = getDistances();
        if (distances[0] < distances[1]) {
            closerSensor = leftRangeSensor;
            leftSensor = true;
        } else {
            closerSensor = rightRangeSensor;
        }
        telemetry.addData("Distance to right sensor: ", distances[1]);
        telemetry.addData("Distance to left sensor: ", distances[0]);
        telemetry.addData("Using the left sensor? ", leftSensor);
        telemetry.update();


        double startDistance = closerSensor.getDistance(DistanceUnit.CM); //gets the distance to the wall
        double distanceTraveled = 0; //sets the distance traveled to 0
        double totalNeededToTravel = startDistance - endDistance; //gets the total distance the robot needs to move
        double lastDistance = startDistance; //last distance is first distance
        double thingy = totalNeededToTravel * totalNeededToTravel * totalNeededToTravel;
        double slope = -maxPowerLevel / thingy; //gets the slope of the graph that is needed to make y = 0 when totalNeeded to travel is x

        while (closerSensor.getDistance(DistanceUnit.CM) > endDistance && !isStopRequested()) //while op mode is running and the distance to the wall is greater than the end distance
        {
            telemetry.addData("Status: ", "Rushing A");
            double currentDistance = closerSensor.getDistance(DistanceUnit.CM); //gets the current distance to the wall
            telemetry.addData("Current distance to wall: ", currentDistance);
            telemetry.addData("Using left sensor? ", leftSensor);

            double deltaDistance = lastDistance - currentDistance; //change in distance is the last distance - currentDistance

            distanceTraveled += deltaDistance; //adds the change in distance to distance traveled
            lastDistance = currentDistance; //the last distance is set to the current distance
            double distanceCubed = distanceTraveled * distanceTraveled * distanceTraveled;
            double power = (   slope * distanceCubed) + maxPowerLevel; // the power is the x value in that position
            if (power > maxPowerLevel) power = minPowerLevel;
            if (power < minPowerLevel && power > 0) power = minPowerLevel; //if the power is less than the min power level just set the power to the minpower level
            if (power == 0) power = 0; //if its 0 then set it to 0 of course
            telemetry.addData("Power", power);
            move(power, power); //moves the robot forward with whatever the power is
            telemetry.update();
        }
        move(0, 0); //once done stop

    }

     /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * This will slowly increase speed as the robot gets closer to the specified degrees
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        telemetry.addData("Rotating", true); //informs
        telemetry.update();

        resetAngle(); //sets starting angle and resets the amount turned to 0

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).
        double rotateCubed = degrees * degrees * degrees; //a new variable to make things easier for me to read
        double slope = (power - minPower) / rotateCubed;

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (!isStopRequested() && getAngle() == 0) {
                double currentAngle = getAngle();
                double CurrentAngleCubed = currentAngle * currentAngle * currentAngle;
                double newPower = slope * CurrentAngleCubed + minPower;
                if (newPower < minPower) newPower = minPower;
                else if (newPower > power) newPower = power;
                if (newPower < 0) newPower = 0;
                telemetry.addData("Power: ", -newPower);
                telemetry.update();
                move(newPower, -newPower);
            }

            while (!isStopRequested() && getAngle() > degrees) {
                double currentAngle = getAngle();
                double CurrentAngleCubed = currentAngle * currentAngle * currentAngle;
                double newPower = slope * CurrentAngleCubed + minPower;
                if (newPower < minPower) newPower = minPower;
                else if (newPower > power) newPower = power;
                if (newPower < 0) newPower = 0;
                telemetry.addData("Power: ", -newPower);
                telemetry.update();
                move(newPower, -newPower);
            } //once it starts turning slightly more than it should.
        } else {
            // left turn.
            while (!isStopRequested() && getAngle() < degrees) {
                double currentAngle = getAngle();
                double CurrentAngleCubed = currentAngle * currentAngle * currentAngle;
                double newPower = slope * CurrentAngleCubed + minPower;
                if (newPower < minPower) newPower = minPower;
                else if (newPower > power) newPower = power;
                if (newPower < 0) newPower = 0;
                telemetry.addData("Power: ", newPower);
                telemetry.update();
                move(-newPower, newPower);
            }
        }


        // turn the motors off.
        move(0, 0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
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
}

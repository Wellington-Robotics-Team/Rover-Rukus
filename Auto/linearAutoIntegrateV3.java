/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "linearautointegrateV3",group = "auto")
public class linearAutoIntegrateV3 extends SeanLinearOpMode {

    // Declare OpMode members.
    //infrastructure vars
    private ElapsedTime runtime = new ElapsedTime();
    Robot testbot;
    //Servo markerServo;
    DcMotor liftMotor;
    BNO055IMU imu; //declare imu
    private DistanceSensor rightRangeSensor;//declare range sensor
    private DistanceSensor leftRangeSensor;
    private DistanceSensor frontRangeSensor;

    //DogeCV
    private GoldAlignDetector goldAlignDetector;
    //Enum with positions for the gold mineral
    enum GoldPosition{
        RIGHT,
        LEFT,
        CENTER
    }
    //Variables used to do encoder driving
    double radius = 3;//inches
    //convert inches to ticks
    public final double inchesToTicks = (560/(2*3.14159*radius));//560 ticks per rev for neverest 20s



    //non infrastructure vars (easily editable, will be edited)
    GoldPosition goldLoc = GoldPosition.CENTER;

    //position at which the servo holds the idol/marker
    double servoHoldPos;//unknown now

    //position at which the servo drops the idol/marker
    double servoDropPos;//unknown now

    //distance in from the center of robot
    final double forwardOffset =0.5;//measure on bot

    //distance from front of bot to wall
    double wallDist;

    //distance to leave in order to turn and not catch on wall
    double turnDist = 2;

    //ticks to lower the robot.
    int liftTicksLower = 10070;//what it was on jankbot

    //inches to drive forward initially just to get away from the lander
    double driveOffLift = 6;

    //degree position before the align with the cube
    double rotBeforeAlign = 0;
    //degree position after the align with the cube
    double rotAfterAlign;
    //theta for sample field -- this is used in our path
    double thetaToSample;

    //use pythag and such to find length of path to the left and right minerals.
    //finding a and b in right tri
    double inchesToCenterMineral = 35;
    double inchesBetweenMinerals = 14.5;
    //finding C in right tri
    double wiggleRoom = 0;
    double inchesToSideMinerals = Math.sqrt(inchesToCenterMineral*inchesToCenterMineral+inchesBetweenMinerals*inchesBetweenMinerals)+wiggleRoom;

    //we want to drive a little more than we really need for the minerals.
    //in inches
    double overshootSides = 2;
    double overshootCenter;//this is unknown as of now because you need theta

    //drive after hitting the center mineral
    double afterCenter = 10;
    double afterSideMineralDist;
    double afterMineralTurn;
    double backUpinches = -20;

    private double maxPower = 0.30; //power of the robot
    private double minPower = 0.09; //least amount of power the robot can have
    double turnPowerNormal = 0.3;
    double turnPowerMin = 0.25;

    //important vars for pid align
    double timePrev = 0;
    double timeCurr = 0;
    double timeDelta = 0;
    double error;
    double pControl = 0;
    double pGain = 0.068;//should be tuned

    double iControl;
    double iGain = 0.0005;//should be tuned

    double controllerOut;
    int successes = 0;
    double[] distances;

    @Override
    public void runOpMode() {
        //Init button pressed.

        telemetry.addData("Status", "Started Init");
        telemetry.update();
        //Init sensors first because they take less battery and are most important

        //Init distance
        rightRangeSensor = hardwareMap.get(DistanceSensor.class, "rightRangeSensor");
        leftRangeSensor = hardwareMap.get(DistanceSensor. class,"leftRangeSensor");
        frontRangeSensor = hardwareMap.get(DistanceSensor.class,"frontRangeSensor");

        //Init DogeCV
        goldAlignDetector = new GoldAlignDetector(); // Create detector
        goldAlignDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        goldAlignDetector.useDefaults(); // Set detector to use default settings
        goldAlignDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldAlignDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldAlignDetector.downscale = 0.4; // How much to downscale the input frames
        goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldAlignDetector.maxAreaScorer.weight = 0.005; //
        goldAlignDetector.ratioScorer.weight = 5; //
        goldAlignDetector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        //Init imu
        //gets the imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //set params
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //initalize the imu
        imu.initialize(parameters);
        //check if the imu is calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Status", " calibrating..."); //inform
            telemetry.update();
            sleep(10);
            idle();
        }

        /*//Init Servo
        markerServo = hardwareMap.get(Servo.class, "idol");
        markerServo.setPosition(servoHoldPos);//set to straight up.*/

        //Init robot
        testbot = new Robot(hardwareMap);
        //Init wheels
        testbot.InitializeMotors();
        //braking,
        testbot.enableBraking();

        //Init Lift
        liftMotor = hardwareMap.dcMotor.get("LIFT");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Set lift:
        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //All done!
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Lowering");
        telemetry.update();
        //Lower the robot/unhook
        liftMotor.setTargetPosition(liftTicksLower);
        while(liftMotor.isBusy()) {
            liftMotor.setPower(1);
        }
        liftMotor.setPower(0);
        rotate(-30,turnPowerNormal);
        //lower lift
        //lower lift
        liftMotor.setTargetPosition(0);
        while(liftMotor.isBusy()) {
            liftMotor.setPower(1);
        }
        liftMotor.setPower(0);

        rotate(30,turnPowerNormal);

        //Rotate back to roughly 45 deg - how it was hanging
        rotateBackToHanging();

        //drive forward so that the lift doesn't hit the lander.
        testbot.TargetDist(Robot.RIGHTMOTORS, ticksint(inchesToTicks*driveOffLift));
        testbot.TargetDist(Robot.LEFTMOTORS, ticksint(inchesToTicks*driveOffLift));
        encoderDrive();

        //lower lift
        liftMotor.setTargetPosition(0);
        while(liftMotor.isBusy()) {
            liftMotor.setPower(1);
        }
        liftMotor.setPower(0);

        //Align
        rotBeforeAlign = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rotate(-40,turnPowerNormal);
        alignWithCube();
        rotAfterAlign = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //find the degree turned to find the gold mineral (will be negative if its on right)
        thetaToSample = rotAfterAlign-rotBeforeAlign;
        //yes, I know this takes more time than doing it myself. It's more readable, and
        //the robot isn't moving right now so it won't throw the path off.

        //decide which side gold is on
        if (thetaToSample >10){
            goldLoc = GoldPosition.LEFT;
        }
        if (thetaToSample <-10){
            goldLoc = GoldPosition.RIGHT;
        }
        //default is center

        //set the overshoot for the center
        overshootCenter = overshootSides*Math.cos(degToRad(Math.abs(thetaToSample)));
        //again, using Math. but it isn't in motion
        //set afterSide, using trig
        afterSideMineralDist = Math.hypot(inchesBetweenMinerals+overshootSides*Math.sin(degToRad(Math.abs(thetaToSample))),afterCenter);
        afterMineralTurn = 90-Math.atan(Math.abs((overshootCenter+afterCenter)/(inchesBetweenMinerals+overshootSides*Math.sin(degToRad(Math.abs(thetaToSample))))));



        //drive up to the mineral, distance varies depending on position
        switch (goldLoc){

            case CENTER:

                telemetry.addData("Status: ", "CENTER");
                telemetry.update();
                testbot.TargetDist(Robot.RIGHTMOTORS,ticksint(inchesToTicks*(inchesToCenterMineral+overshootCenter+afterCenter)));
                testbot.TargetDist(Robot.LEFTMOTORS,ticksint(inchesToTicks*(inchesToCenterMineral+overshootCenter+afterCenter)));
                encoderDrive();
                //the middle of the path
                rotate(45,turnPowerNormal);

                break;
            case RIGHT:
                telemetry.addData("Status: ", "RIGHT");
                telemetry.addData("Theta: ", thetaToSample);
                telemetry.update();
            case LEFT:
                if (goldLoc == GoldPosition.LEFT){
                    telemetry.addData("Status: ", "LEFT");
                    telemetry.addData("Theta: ", thetaToSample);
                    telemetry.update();
                }
                //should drive far enough that the center of the bot is on the dot, then drive a bit farther.
                testbot.TargetDist(Robot.RIGHTMOTORS,ticksint(inchesToTicks*(inchesToSideMinerals+overshootSides)));
                testbot.TargetDist(Robot.LEFTMOTORS,ticksint(inchesToTicks*(inchesToSideMinerals+overshootSides)));
                encoderDrive();

                //rotate(-thetaToSample,turnPowerNormal);
                telemetry.addData("Status: ", "LEFT 45,perp next");
                telemetry.update();
                //sleep(1000);
                //rotate(-afterMineralTurn*thetaToSample/Math.abs(thetaToSample),maxPower);
                rotate(45-thetaToSample,turnPowerNormal);
                telemetry.addData("Status: ", "LEFT Perp, drive next");
                telemetry.update();
                //sleep(1000);
                /*testbot.TargetDist(Robot.RIGHTMOTORS,ticksint(inchesToTicks*afterSideMineralDist));
                testbot.TargetDist(Robot.LEFTMOTORS,ticksint(inchesToTicks*afterSideMineralDist));
                encoderDrive();
                rotate(afterMineralTurn*thetaToSample/Math.abs(thetaToSample),turnPowerNormal);*/


                break;
        }
        telemetry.addData("Status: ", "Post-Switch");
        telemetry.update();
        //rotate(-45,turnPowerNormal);
        telemetry.addData("Status: ", "DRIVE to wall, 90 next");
        telemetry.update();
        //sleep(500);
        if (goldLoc ==GoldPosition.RIGHT)LoopAlignPid(1,1);
        wallDist = frontRangeSensor.getDistance(DistanceUnit.INCH)-forwardOffset-turnDist;
        telemetry.addData("Wall: ", wallDist);
        telemetry.update();
        testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*wallDist));
        testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*wallDist));
        encoderDrive();
        //sleep(1000);
        rotate(90,turnPowerNormal);
        if(goldLoc==GoldPosition.RIGHT){
            testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*12));
            testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*12));
            encoderDrive();
        }
        telemetry.addData("Status: ", "90, backup next" );
        telemetry.update();
        //sleep(1000);
        //align();
        LoopAlignPid(0.6,4);//normal values are 0.4 and 4
        //sleep(1000);
        if(goldLoc == GoldPosition.LEFT){
            testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*backUpinches));
            testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*backUpinches));
            encoderDrive();
        }

        //markerServo.setPosition(servoDropPos);
        RushC(true);


        while (opModeIsActive()) {

            telemetry.addData("Status", "Finished. Time: " + runtime.toString());
            telemetry.update();
        }
    }

    private double degToRad(double Degrees){
        //quick little meth
        return Degrees*180/3.14159;
    }

    //initial rotation, from hanging to sitting on the field.
    double initialRotOff = 0;//zero so errors won't be thrown if the gyro doesn't return.

    private void rotateBackToHanging(){
        initialRotOff = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //rotate
        rotate(-(int) //negative so it rotates back rather than further off.
                Math.floor(initialRotOff),turnPowerMin); //floor rounds down. Being conservative rather than aggressive.

    }
    double startAngleAlign;
    double endAnglAlign;
    double deltaAngleAlign,angleBetweensamples;
    boolean isDetectorEnabled = false;
    private void enableDetector(){
        goldAlignDetector.enable();
        isDetectorEnabled = true;
    }
    private void disableDetector(){
        goldAlignDetector.disable();
        isDetectorEnabled = false;
    }
    private void alignWithCube(){
        enableDetector();
        sleep(1000);
        while (opModeIsActive() && !goldAlignDetector.getAligned())
        {
            double xPos = goldAlignDetector.getXPosition();
            telemetry.addData("IsAligned" , goldAlignDetector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos" , xPos); // Gold X position.
            telemetry.update();

            if (xPos > 300) testbot.TankDrive(-turnPowerMin, turnPowerMin);
            else testbot.TankDrive(turnPowerMin, -turnPowerMin);
        }
        testbot.TankDrive(0,0);
        disableDetector();
    }
    private int ticksint(double ticks){
        return (int)Math.round(ticks);
    }
    double encSpeed = 0;
    private void encoderDrive(){
        int targetTicks = testbot.BRM.getTargetPosition();
        while(opModeIsActive()&&
                testbot.BRM.isBusy()&&
                testbot.BLM.isBusy()){
            encSpeed = (maxPower-minPower)/targetTicks*
                    //take the absolute value of the average ticks between back motors minus the target divided by two.
                    Math.abs((testbot.BRM.getCurrentPosition()+testbot.BLM.getCurrentPosition())/2-(targetTicks/2))
                    +maxPower;
            testbot.TankDrive(encSpeed,encSpeed);
        }
        testbot.TankDrive(0,0);
        testbot.disableEncoders();

    }

    private void LoopAlignPid(double accuracyCM, int timesCorrect) {
        testbot.enableEncoders();
        runtime.reset();
        do
        {
            alignPid();
            telemetry.update();
        }while(opModeIsActive() && !wasSuccessful(error,accuracyCM, timesCorrect));
        telemetry.update();
        testbot.TankDrive(0,0);
    }
    private void alignPid(){

        error = getDeltaRange();//returns left minus right

        timePrev = timeCurr;
        timeCurr = runtime.time(TimeUnit.SECONDS);
        //this and timeprev implementation are new
        timeDelta = timeCurr-timeDelta;

        //this is new
        if (distances[0]<100&&distances[1]<100) {
            pControl = error*pGain;
            iControl = error * iGain * timeDelta + iControl;// add to integrate
            controllerOut = pControl+iControl;

        }
        else
        {
            controllerOut = pControl;
        }
        if(Math.abs(controllerOut) > maxPower) controllerOut = maxPower*getSign(controllerOut);
        testbot.TankDrive(controllerOut,-controllerOut);

        telemetry.addData("delta",error);
        telemetry.addData("leftsensor",distances[0]);
        telemetry.addData("rightsensor",distances[1]);
        telemetry.addData("pGain", pGain);
        telemetry.addData("iGain", iGain);
        telemetry.addData("icontrol", iControl);
        telemetry.addData("controllerOut", controllerOut);
    }
    private boolean wasSuccessful(double error, double accuracyCM, int timesCorrect) {
        //repititions
        if (error <= accuracyCM&&
                error>=-accuracyCM&&
                distances[0]<300)
        {
            successes++;
        }
        else{
            successes = 0;
        }
        telemetry.addData("success",successes);
        telemetry.addData("%success",successes/timesCorrect*100);
        if (successes>=timesCorrect) return true;
        else return false;
    }
    private double getSign(double s){
        return Math.abs(s)/s;
    }

    @Override
    public void stop(){
        if(imu!=null) imu.close();
        if (isDetectorEnabled) disableDetector();
        if(testbot.FRM!=null) testbot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        super.stop();
    }

    //Methods from AutoGabo
    //align
    private void align() {

        double accuracy = 1; //how accurate this should be to the CM

        double deltaRange = -getDeltaRange();// gets the cha  nge in distance between the 2 sensors      +1 because inaccurate

        while (Math.ceil(Math.abs(deltaRange)) > accuracy && !isStopRequested()) //while the delta is greater than the accuracy we want
        {

            telemetry.addData("Status: ", "Aligning");
            deltaRange = -getDeltaRange();
            telemetry.addData("Delta Range", deltaRange);
            telemetry.update();
            if (deltaRange < 0) //if its negative
            {
                testbot.TankDrive(turnPowerMin, -turnPowerMin);//turn left

            } else if (deltaRange > 0) //if the delta range is positive
            {
                testbot.TankDrive(-turnPowerMin, turnPowerMin);//turn right
            }
        }
        testbot.TankDrive(0,0); //stops
    }
    private double getDeltaRange() {
        distances = getDistances(); //gets the distances
        return distances[0] - distances[1]; //subtracts the 2
    }
    private double[] getDistances() {
        double leftDistance = leftRangeSensor.getDistance(DistanceUnit.CM); //left sensor distance
        double rightDistance = rightRangeSensor.getDistance(DistanceUnit.CM); //right sensor distance
        double[] distances = new double[2]; //makes an array
        distances[0] = leftDistance; //sets positions to values
        distances[1] = rightDistance;
        return distances; //returns the array
    }
    private void RushC(boolean forward) {
        telemetry.addData("Status: ", "Workin");
        telemetry.update();

        telemetry.addData("Status: ", "Finished");
        telemetry.update();

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle; //gets the roll of the robot

        telemetry.addData("Current Angle: ", currentAngle);
        telemetry.update();
        //sleep(3000);
        while ((currentAngle < -80 && currentAngle > -100) && !isStopRequested()) //while the current roll is less than 72
        {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle; //regets the currentAngle
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.update();
            if (forward)
            {
                testbot.TankDrive(maxPower, maxPower); //continue moving
            } else
            {
                testbot.TankDrive(-maxPower, -maxPower); //backwards
            }

        }
        //give it enough time to get over crater
        sleep(500);
        testbot.TankDrive(0, 0); //stop
        telemetry.addData("Finished", true);
        telemetry.update();
    }
    //Variables from AutoGabo
    private double globalAngle; //the number of degrees the robot has turned
    Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors
        //MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.undertale);

    private void rotate(double degrees, double maxPower) {
        //I changed degrees to a double because that's what the imu gives and
        //I didn't see a reason not to.
        // Sean 12/11/18

        telemetry.addData("Rotating", true); //informs
        telemetry.update();

        resetAngle(); //sets starting angle and resets the amount turned to 0

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).
        double thingy = degrees * degrees * degrees;
        double slope = -maxPower / thingy; //gets the slope of the graph that is needed to make y = 0 when totalNeeded to travel is x

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (!isStopRequested() && getAngle() == 0) {
                double currentAngle = getAngle();
                double thingy1 = currentAngle * currentAngle * currentAngle;
                double newPower = slope * thingy1 + maxPower; // the power is the x value in that position
                if (newPower < turnPowerMin) newPower = turnPowerMin;
                if (newPower <= 0) newPower = 0;
                telemetry.addData("Power: ", -newPower);
                telemetry.update();
                testbot.TankDrive(-newPower, newPower);
            }

            while (!isStopRequested() && getAngle() > degrees) {
                double currentAngle = getAngle();
                double thingy3 = currentAngle * currentAngle * currentAngle;
                double newPower = slope * thingy3 + maxPower; // the power is the x value in that position
                if (newPower < turnPowerMin) newPower = turnPowerMin;
                if (newPower <= 0) newPower = 0;
                telemetry.addData("Power: ", -newPower);
                telemetry.update();
                testbot.TankDrive(-newPower, newPower);
            } //once it starts turning slightly more than it should.
        } else {
            // left turn.
            while (!isStopRequested() && getAngle() < degrees) {
                double currentAngle = getAngle();
                double thingy2 = currentAngle * currentAngle * currentAngle;
                double newPower = slope * thingy2 + maxPower; // the power is the x value in that position
                if (newPower < turnPowerMin) newPower = turnPowerMin;
                if (newPower <= 0) newPower = 0;
                telemetry.addData("Power: ", newPower);
                telemetry.update();
                testbot.TankDrive(newPower, -newPower);
            }
        }

        // turn the motors off.
        testbot.TankDrive(0, 0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //sets lastAngles to current angles

        globalAngle = 0; //global angle is set to 0
    }
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
}

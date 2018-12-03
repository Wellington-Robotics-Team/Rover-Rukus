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
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "SamplerV4", group = "Auto")
public class SamplerV4 extends LinearOpMode {
//0.17,0.4
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Robot testbot;
    Servo markerServo;
    DcMotor liftMotor;

    int liftTicks = -13532;
    private SamplingOrderDetector detector;


    BNO055IMU imu; //declare imu

    Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    double globalAngle; //the number of degrees the robot has turned

    double power = 0.2; //power of the robot
    double turningPower = power +0.1;
    double minPower = 0.1; //least amount of power the robot can have
    //8 degrees of latency
    int turnDegrees; //the number of degrees the robot should turn. Set to -82 to turn right 90 degrees. Latency is a pain

    double radius = 3;//inches
    public final double inchesToTicks = (560/(2*3.14159*radius));//560 ticks per rev for neverest 20s

    double driveOffLift = 9;
    double encSpeed = 0;
    double floorpadinches = 23.5;
    double inchesDepotToCrater = floorpadinches*4;//lengh of one floor pad times 5 for five floor pads.
    //use pythag to find length of path to the left and right minerals.
    double inchesToCenterMineral = 24;
    //using pythag
    double inchesBetweenMinerals = 14.5;
    double wiggleRoom = -1;
    double inchesToSideMinerals = Math.sqrt(inchesToCenterMineral*inchesToCenterMineral+inchesBetweenMinerals*inchesBetweenMinerals)+wiggleRoom;
    int angleBetweenSamples = (int)Math.round(180/3.14*Math.atan(inchesBetweenMinerals/inchesToCenterMineral))-5;//minus 5 for latency
    //int cornerTurn = 135-135*5/90;

    double inchesIntoDepot = 6;//7.5
    double servorestpos = 0.4;
    double servoHoldpos = 0.15;
    @Override
    public void runOpMode() {
        //Init button pressed.
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        testbot = new Robot(hardwareMap);
        testbot.InitializeMotors();
        testbot.enableBraking();

        liftMotor = hardwareMap.dcMotor.get("LIFT");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //servo init
        markerServo = hardwareMap.get(Servo.class, "left_hand");
        markerServo.setPosition(servoHoldpos);//set to straight up.
        //Doge CV
        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*//Testing rotate
        rotate(90,turningPower);
        sleep(2000);
        rotate(100,turningPower);
        sleep(2000);
        rotate(2*angleBetweenSamples,turningPower);
        */


        sleep(4000);
        // Initialize things here.
        //determine which position the gold is in
        SamplingOrderDetector.GoldLocation Location  = detector.getLastOrder();
        /*
        liftMotor.setTargetPosition(liftTicks);
        liftMotor.setPower(1);
        sleep(7000);
        rotate(5,turningPower);
        sleep(1000);
        rotate(-5,turningPower);
        liftMotor.setTargetPosition(liftTicks-500);
        liftMotor.setPower(1);
        */
        telemetry.addData("Gyro ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
        testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*driveOffLift));
        testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*driveOffLift));
        encoderDrive();

        detector.disable();
        switch (Location) {
            //if the gold is on the left, turn left the right amount (with gyro)
            case LEFT:
                //leaving movement normal to test switch and angles, should change to encoders.
                telemetry.addData("Case?", " LEFT"); //very informative telemetry
                telemetry.update();
                rotate(angleBetweenSamples, turningPower);
                runtime.reset();
                testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*inchesToSideMinerals+2.5));
                testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*inchesToSideMinerals+2.5));
                encoderDrive();
                //turn back to center, then past center to look at the depot
                rotate(-2*angleBetweenSamples, turningPower);
                //drive into depot
                testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*inchesToSideMinerals));
                testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*inchesToSideMinerals));
                encoderDrive();
                testbot.TankDrive(0, 0);
                //turn so that all cases end pointing the same direction
                rotate(angleBetweenSamples,turningPower);
                break;

            case RIGHT:
                telemetry.addData("Case?", " Right"); //very informative telemetry
                telemetry.update();
                rotate(-angleBetweenSamples, turningPower);
                runtime.reset();
                testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*inchesToSideMinerals+2.5));
                testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*inchesToSideMinerals+2.5));
                encoderDrive();
                //it gets here
                telemetry.addData("Case?", " Right, Mid-run"); //very informative telemetry
                telemetry.update();

                rotate(2*angleBetweenSamples, turningPower);//this line has issues.
                telemetry.addData("Case?", " Right, Past Rotate"); //very informative telemetry
                telemetry.update();
                //sleep(1000);
                testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*inchesToSideMinerals));
                testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*inchesToSideMinerals));
                encoderDrive();
                testbot.TankDrive(0, 0);
                //end turn
                rotate(-angleBetweenSamples,turningPower);
                break;
            default:
            case CENTER:
                telemetry.addData("Case?", " CENTER"); //very informative telemetry
                telemetry.update();
                runtime.reset();
                testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*inchesToCenterMineral+2));
                testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*inchesToCenterMineral+2));
                encoderDrive();
                testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*inchesToCenterMineral));
                testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*inchesToCenterMineral));
                encoderDrive();
                testbot.TankDrive(0, 0);
                break;


        }

        //markerServo.setPosition(0);//drop marker
        rotate(-45+5,turningPower);

        testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*inchesIntoDepot-2));
        testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*inchesIntoDepot-2));
        encoderDrive();

        rotate(-90+4,turningPower);//5
        markerServo.setPosition(0.6);
        sleep(500);
        markerServo.setPosition(0.6);
        testbot.TargetDist(Robot.RIGHTMOTORS, (int)Math.round(inchesToTicks*inchesDepotToCrater));
        testbot.TargetDist(Robot.LEFTMOTORS, (int)Math.round(inchesToTicks*inchesDepotToCrater));
        encoderDrive();


        runtime.reset();
        //testbot.TankDrive(0.3,0.3);
       //sleep(timetodrive);
        // run until the end of the match (driver presses STOP)
        testbot.TankDrive(0,0);
        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

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
    private void encoderDrive(){
        int targetTicks = testbot.BRM.getTargetPosition();
        while(opModeIsActive()&&
                testbot.BRM.isBusy()&&
                testbot.BLM.isBusy()){
            encSpeed = (power-minPower)/targetTicks*
                    //take the absolute value of the average ticks between back motors minus the target divide by two.
                    Math.abs((testbot.BRM.getCurrentPosition()+testbot.BLM.getCurrentPosition())/2-(targetTicks/2))
                    +power;
            testbot.TankDrive(encSpeed,encSpeed);
        }
        testbot.TankDrive(0,0);
        testbot.disableEncoders();
        ;
    }
    private void rotate(int degrees, double power)
    {
        //telemetry.addData("Rotating", true); //informs
        //telemetry.update();
        double  leftPower, rightPower; //declares the powers

        resetAngle(); //sets starting angle and resets the amount turned to 0

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).

        if (degrees < 0)// turn right.
        {
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)// turn left.
        {
            leftPower = -power;
            rightPower = power;
        }
        else return; //do nothing

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                testbot.TankDrive(rightPower, leftPower);
            }

            while (opModeIsActive() && getAngle() > degrees) {
                testbot.TankDrive(rightPower, leftPower);
            } //once it starts turning slightly more than it should.
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                testbot.TankDrive(rightPower, leftPower);
            }

        // turn the motors off.
        testbot.TankDrive(0,0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //sets lastAngles to current angles

        globalAngle = 0; //global angle is set to 0
    }
}

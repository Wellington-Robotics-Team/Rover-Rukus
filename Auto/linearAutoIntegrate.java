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
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Disabled
public class linearAutoIntegrate extends LinearOpMode {

    // Declare OpMode members.
    //infrastructure vars
    private ElapsedTime runtime = new ElapsedTime();
    Robot testbot;
    Servo markerServo;
    DcMotor liftMotor;
    BNO055IMU imu; //declare imu
    private GoldAlignDetector goldAlignDetector;

    //non infrastructure vars
    //position at which the servo holds the idol/marker
    double servoHoldpos;//unknown now
    //ticks to lower the robot.
    int liftTicksLower = -13532;//what it was on jankbot


    @Override
    public void runOpMode() {
        //Init button pressed.

        telemetry.addData("Status", "Started Init");
        telemetry.update();
        //Init Servo
        markerServo = hardwareMap.get(Servo.class, "idol");
        markerServo.setPosition(servoHoldpos);//set to straight up.

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
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            sleep(50);
            idle();
        }

        //All done!
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Lower the robot/unhook
        liftMotor.setTargetPosition(liftTicksLower);
        liftMotor.setPower(1);

        //Rotate back to roughly 45 deg - how it was hanging
        rotateBackToHanging();
        //Align

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        //clean up

    }

    //initial rotation, from hanging to sitting on the field.
    double initialRotOff = 0;//zero so errors won't be thrown if the gyro doesn't return.

    private void rotateBackToHanging(){
        initialRotOff = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //rotate
        rotate(-(int) //negative so it rotates back rather than further off.
                Math.floor(initialRotOff),maxPower); //floor rounds down. Being conservative rather than aggressive.

    }
    double startAngleAlign;
    double endAnglAlign;
    double deltaAngleAlign,angleBetweensamples;

    private void alignWithCube(){
        while (opModeIsActive() && !goldAlignDetector.getAligned())
        {
            double xPos = goldAlignDetector.getXPosition();
            telemetry.addData("IsAligned" , goldAlignDetector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos" , xPos); // Gold X position.
            telemetry.update();

            if (xPos < 300) testbot.TankDrive(minPower, -minPower);
            else testbot.TankDrive(-minPower, minPower);
        }
        testbot.TankDrive(0,0);
        goldAlignDetector.disable();

    }

    //Methods from AutoGabo
    //Variables from AutoGabo
    private double globalAngle; //the number of degrees the robot has turned
    Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors
    private double maxPower = 0.50; //power of the robot
    private double minPower = 0.25; //least amount of power the robot can have
    //MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.undertale);

    private void rotate(int degrees, double maxPower) {
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
                if (newPower < minPower) newPower = minPower;
                if (newPower <= 0) newPower = 0;
                telemetry.addData("Power: ", -newPower);
                telemetry.update();
                testbot.TankDrive(-newPower, newPower);
            }

            while (!isStopRequested() && getAngle() > degrees) {
                double currentAngle = getAngle();
                double thingy3 = currentAngle * currentAngle * currentAngle;
                double newPower = slope * thingy3 + maxPower; // the power is the x value in that position
                if (newPower < minPower) newPower = minPower;
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
                if (newPower < minPower) newPower = minPower;
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

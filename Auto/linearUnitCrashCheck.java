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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;


@TeleOp(name = "linearUnitCrashCheck", group = "Unit")
public class linearUnitCrashCheck extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    Robot testbot;

    @Override
    public void runOpMode() {
        //Init button pressed.

        //set up imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //init imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //NOTE: imu sample code doesn't calibrate, Idk why. I am not going to either.

        //set up 'bot
        testbot = new Robot(hardwareMap);
        testbot.InitializeMotors();

        //all set up.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //simple teleop with scaling so we don't go too fast.
            testbot.TankDrive(gamepad1.right_stick_y*4/10,gamepad1.left_stick_y*4/10);
            //if we've crashed
            if(hasCrashed()){
                //do the thing that we do when crashed
                //currently stop and back up.
                crashOperation();
            }
        }
    }
    //need a bool to describe whether or not the robot has crashed
    private boolean crashed = false;
    //acceleration from imu
    Acceleration accel;
    /*
        These are 1d Kinematics values to find the accel threshold. Values are negative
    because negative is forward using rev imu. Units here are m/s and m. This isn't
    standard for us because field dimensions are imperial, but they are SI units and
    more familiar for physics.
    */
    //Vo
    private double Vo = -1.524;
    //V
    private double V = 0;
    //Xo
    private double Xo = 0;
    //X
    private double X = -0.1;
    //acceleration threshold - if it is larger than this, we say there has been a crash.
    //determined with 1d kinematics, using above vars.
    private double aThreshold = (Vo*Vo-V*V)/(-2*X-Xo);//11.61 m/s/s

    private boolean hasCrashed(){
        //find out if the bot has crashed

        //positive x accel should be backwards according to rev lettering
        accel = imu.getLinearAcceleration();
        //if the acceleration is larger than the threshold return true
        return (accel.xAccel>=-aThreshold);
    }
    //slow speed to back away at
    private double slowSpeed = 0.13;
    //time (ms) to back up for
    private long time = 400;
    private void crashOperation(){
        //first, stop bot
        testbot.TankDrive(0,0);
        //reverse away from whatever you just hit, but slowly.
        testbot.TankDrive(-slowSpeed,-slowSpeed);
        sleep(time);
        testbot.TankDrive(0,0);
        //could also perform some kind of systems check, i.e. try to spin each motor.
        //not going to worry about that now.
    }
}

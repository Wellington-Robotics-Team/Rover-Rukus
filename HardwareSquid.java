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

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareSquid
{
    /* Public OpMode members. */
    public DcMotor  FLM   = null;
    public DcMotor  FRM  = null;
    public DcMotor  RLM  = null;
    public DcMotor  RRM  = null;
    public DcMotor  LiftM = null;
    //public DcMotor  LiftClawM = null;
    public Servo    LiftClaw   = null;
    public Servo    JewelArm  = null;
    public Servo    ArmShoulder = null;
    public Servo    ArmElbow = null;
    public Servo    ArmWrist = null;
    public Servo    ArmClaw = null;
    //public Servo    Extender = null;
    // Interface Devices


    // Public Values
    double power = 0.5;
    double servoDelta = 0.001;
    ElapsedTime timer;
    // Arm Lengths
    public double   Lu=38;
    public double   Ll=38;
    public double   MagMax;
    // Digital Switches for lift stops
    public DigitalChannel DTop;
    public DigitalChannel DBottom;

    public double ElbowScale = 1.0;
    public double ShoulderScale = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSquid(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        MagMax=Math.pow(Ll+Lu,2);
        // Define and Initialize Motors
        FLM  = hwMap.get(DcMotor.class, "FLM"); // Correct
        FRM = hwMap.get(DcMotor.class, "FRM");  // Correct
        RLM  = hwMap.get(DcMotor.class, "BLM"); // Correct
        RRM = hwMap.get(DcMotor.class, "BRM");  // m4
        LiftM  = hwMap.get(DcMotor.class, "Lift"); // Correct
        //LiftClawM = hwMap.get(DcMotor.class,"LiftClawM");
        //DTop = hwMap.get(DigitalChannel.class,"switch1");
        //DBottom = hwMap.get(DigitalChannel.class,"switch2");
        FLM.setDirection(DcMotor.Direction.FORWARD); // et to REVERSE if using AndyMark motors
        FRM.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        RLM.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RRM.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        FLM.setPower(0);
        FRM.setPower(0);
        RLM.setPower(0);
        RRM.setPower(0);
        LiftM.setPower(0);
        //LiftClawM.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //LiftClawM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LiftClawM.setMode(DcMotor.RunMode.RUN_TO_POSITION); // THE LIFT NEEDS THIS IN ORDER TO OPERATE PROPERLY!!!
        //LiftClawM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //LiftClaw  = hwMap.get(Servo.class, "LiftClaw");
        JewelArm = hwMap.get(Servo.class, "JewelArm");
        ArmElbow = hwMap.get(Servo.class, "ArmServo2");
        ArmShoulder = hwMap.get(Servo.class, "ArmServo1");
        ArmWrist = hwMap.get(Servo.class,"Wrist");
        //Extender = hwMap.get(Servo.class,"Extender");
        ArmClaw  = hwMap.get(Servo.class,"ArmClaw");

        ArmShoulder.setDirection(Servo.Direction.REVERSE);
        ArmElbow.setDirection(Servo.Direction.FORWARD );
        //Extender.setDirection(Servo.Direction.REVERSE);
        ArmClaw.setDirection(Servo.Direction.FORWARD);
        //Extender.scaleRange(0,1);
        ArmWrist.scaleRange(0,1);
        ArmShoulder.scaleRange(0.05,0.57);
        ArmElbow.scaleRange(0,0.3);

        timer = new ElapsedTime();
    }

    public void LiftControl(double up, double down){
        //if (!DBottom.getState()){

            LiftM.setPower((up-down)*0.25);

        //} else LiftM.setPower(-down*0.25);


    }

    public void LiftControl(double Pow){

        LiftM.setPower(-Pow);
    }

    /*
    public void LiftClawControl(double open, double close){

        LiftClawM.setPower(close-open);

    }
    */

    public void LiftClawControl(double CurrPosLiftClaw, boolean OpenButton, boolean CloseButton){
        if (OpenButton) {
            CurrPosLiftClaw += servoDelta;
        }

        if (CloseButton){
            CurrPosLiftClaw -= servoDelta;
        }

    }

    public void OmniDrive(double y, double x, double rotation){

        FRM.setPower((y + x+rotation) * power);
        FLM.setPower((y - x-rotation) * power);
        RRM.setPower((y - x+rotation) * power);     // This is the code I made to make the bot go in all directions...
        RLM.setPower((y + x-rotation) * power);

    }

    public boolean SetShoulder(double deg)
    {
        // Set Servo Position based on 180 degree allowed range
        double pos = (deg*ShoulderScale)/180;
        boolean error = true;
        if (pos >= 0 && pos<= 1.0) {
            ArmShoulder.setPosition((deg * ShoulderScale) / 180);
            error = false;
        }
        return error;
    }

    public boolean SetElbow(double deg)
    {
        // Set Servo Position based on 180 degree allowed range
        double pos = ((180-deg)*ElbowScale)/180;
        boolean error = true;
        if (pos >= 0 && pos<= 1.0) {
            ArmElbow.setPosition(((180-deg) * ElbowScale) / 180);
            error = false;
        }
        return error;
    }
}
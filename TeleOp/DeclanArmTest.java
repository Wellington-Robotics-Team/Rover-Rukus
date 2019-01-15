package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DeclanArmTest extends OpMode {

    public HardwareMap hwMap;

    public DcMotor BLM;
    public DcMotor BRM;
    public DcMotor FRM;
    public DcMotor FLM;

    public DcMotor Arm1;
    public DcMotor Arm2;

    //public CRServo Bucket;
    //public CRServo Roller;

    public double power = 0.5;
    public double armpower = 0.05;

    public boolean RollersON = false;

    public void init()
    {
        hwMap = hardwareMap; // Gets Hardware Map

        BLM = hwMap.dcMotor.get("BLM"); //gets the motors
        BRM = hwMap.dcMotor.get("BRM");
        FRM = hwMap.dcMotor.get("FRM");
        FLM = hwMap.dcMotor.get("FLM");

        //BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Arm1 = hwMap.dcMotor.get("Arm1");
        Arm2 = hwMap.dcMotor.get("Arm2"); // Gets Arm Motors

        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Bucket = hwMap.crservo.get("Bucket");
        //Roller = hwMap.crservo.get("Roller"); //Gets Servos for Bucket

        //Bucket.setPosition(0.5);
        //Roller.setPosition(0.5);

    }

    public void loop()
    {
        Arm1.setPower(gamepad1.right_stick_x * armpower);
        Arm2.setPower(gamepad1.right_stick_x * armpower); // Movement for the arm is controlled by the horizontal motion of the right thumbstick at the main power level


        if (gamepad1.right_trigger == 1) {
            while (gamepad1.right_trigger == 1) ;
            armpower += 0.025;
            if (armpower >= 0.3) armpower = 0.3;
        }
        if (gamepad1.left_trigger == 1) {
            while (gamepad1.left_trigger == 1) ;
            armpower -= 0.025;
            if (armpower <= 0) armpower = 0;
        }

        if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper) ;
            power += 0.1;
            if (power >= 1) power = 1;
        }
        if (gamepad1.left_bumper) {
            while (gamepad1.left_bumper) ;
            power -= 0.1;
            if (power <= 0) power = 0;
        }

        //   !!! SERVO CODE FOR THE BUCKET AND ROLLER UNITS !!!

        //Bucket.setPower(gamepad1.right_stick_y * power); //Control the Bucket with vertical motion of the right thumbstick at the main power level

        //if (gamepad1.a) RollersON = true; // Press the A button to turn the Rollers on
        //if (gamepad2.b) RollersON = false; // Press the B button to turn the Rollers off

        //if (RollersON) Roller.setPower(power); //This uses the bool to adjust the roller's status
        //if (!RollersON) Roller.setPower(0);

        double turn = gamepad1.left_stick_x;
        double drive = gamepad1.left_stick_y;
        double leftPower = (-drive + turn)* -power; //left power is negative drive plus turn * negative power
        //straight is left positive and right negative
        double rightPower = (-drive - turn)* power; //same but positive power

        move(leftPower, rightPower); //moves robot with the power

        //Credit to TeleGabo for this quick fix POV Drive code... Thx Gabo for uploading to GitHub!!!

        telemetry.addData("Drive Power: ", power);
        telemetry.addData("Arm Power: ", armpower);
        telemetry.addData("Arm1: ", Arm1.getPower());
        telemetry.addData("Arm2: ", Arm2.getPower());
        telemetry.addData("BLM: ", BLM.getPower());
        telemetry.addData("BRM: ", BRM.getPower());
        telemetry.addData("FRM: ", FRM.getPower());
        telemetry.addData("FLM: ", FLM.getPower());
        telemetry.update();

        /**
         * Summary of Controls:
         *
         * Left Stick Vertical - Back and Forth
         * Left Stick Horizontal - Turn Right and Left
         *
         * Right Stick Vertical - Pitch the Bucket
         * Right Stick Horizontal - Pitch the Arm
         *
         * A Button - Turn on The Rollers
         * B Button - Turn off the Rollers
         *
         * Right Bump - Drive Power Up
         * Left Bump - Drive Power Down
         * Right Trig - Arm Power Up
         * Left Trig - Arm Power Down
         **/
    }

    public void move(double leftPower, double rightPower) //move function for ez use
    {
        BLM.setPower(leftPower); //left motors are leftPower
        FLM.setPower(leftPower);
        FRM.setPower(rightPower); //right motors are rightPower
        BRM.setPower(rightPower);
    }


}

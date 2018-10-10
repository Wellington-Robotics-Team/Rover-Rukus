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
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by David on 10/30/2017.
 */

public class ServoTest extends OpMode {
    /*
    private HardwareMap hwMap = null;
    private double ServoTime = 1.14;
    private Servo ArmServo1 = null;
    private Servo ArmServo2 = null;
    private double CurrPos1 = 0.18; // Rest Position: 0.18, Top Position: 0.05,
    private double CurrPos2 = 0.25; // Rest Position: 0.25, Top Position: 0
    private int sleepErrors = 0;
    private double servoDelta = 0.001;
    */

    OmniBot Arm = new OmniBot();

    double RestPos1 = 0.55;
    double RestPos2 = 0.15;
    double J1Offset;
    double J2Offset;
    double J1Angle;
    double J2Angle;
    double J1x;
    double J1y;
    double J2x;
    double J2y;



    public void init() {
        Arm.hwMap = hardwareMap;
        Arm.InitializeArmServos(38.5, 35, RestPos1, RestPos2);
        Arm.ArmServo1.setDirection(Servo.Direction.REVERSE);
        Arm.ArmServo2.setDirection(Servo.Direction.REVERSE);
        //Arm.InitializeArmClaw();

        Arm.CurrPosWrist = 0.5;
        Arm.CurrPosArmClaw = 0.5;


        //Arm.CurrPosWrist = 1;

    }

    public void loop() {



        if (gamepad1.dpad_down) {
            Arm.CurrPos1 += Arm.servoDelta*2;
        }                                           //Just in case...
        if (gamepad1.dpad_up) {
            Arm.CurrPos1 -= Arm.servoDelta*2;
        }

        if (gamepad1.dpad_left) {
            Arm.CurrPos2 += Arm.servoDelta*2;
        }
        if (gamepad1.dpad_right) {
            Arm.CurrPos2 -= Arm.servoDelta*2;
        }

        if (gamepad1.a) Arm.CurrPosWrist = 1;
        if (gamepad1.y) Arm.CurrPosWrist = 0.5;
        if (gamepad1.x) Arm.CurrPosArmClaw = 1;
        if (gamepad1.b) Arm.CurrPosArmClaw = 0.5;
        //if (gamepad1.y) Arm.CurrPosWrist = 1;

        /*
        if (gamepad1.a) Arm.CurrPosWrist = 0;
        if (gamepad1.b) Arm.CurrPosWrist = 0.25;
        if (gamepad1.y) Arm.CurrPosWrist = 0.5;
        if (gamepad1.x) Arm.CurrPosWrist = 0.75;
        if (gamepad1.start) Arm.CurrPosWrist = 1;
        */


        // Alright, time to figure out the X's and Y's of this bot... FORWARD KINEMATICS

        /*

        J1Angle = Arm.PositionToAngle(Arm.CurrPos1);
        J2Angle = Arm.PositionToAngle(Arm.CurrPos2);

        J1x = Math.cos(((J1Angle*Math.PI)/180))*Arm.L1;
        J1y = Math.sin(((J1Angle*Math.PI)/180))*Arm.L1;
        J2x = Math.cos((((J2Angle+J1Angle)*Math.PI)/180))*(Arm.L1+J1x);
        J2y = Math.sin((((J2Angle+J1Angle)*Math.PI)/180))*(Arm.L1+J1y);

        */




        // Alright, this is where it all ends... INVERSE KINEMATICS BOIIIIII!!!


        /*



        Arm.c2 = (Math.pow(J2x,2) + Math.pow(J2y,2) - Math.pow(Arm.L1,2) - Math.pow(Arm.L2,2)) / (2*Arm.L1*Arm.L2);
        Arm.s2 = Math.sqrt(1-Math.pow(Arm.c2,2));
        Arm.K1 = Arm.L1+Arm.L2*Arm.c2;
        Arm.K2 = Arm.L2*Arm.s2;

        Arm.theta = Math.atan2(J2x,J2y) - Math.atan2(Arm.K1,Arm.K2);
        Arm.psi = Math.atan2(Arm.c2,Arm.s2);

        J1Angle = (Arm.theta*180)/Math.PI;
        J2Angle = (Arm.psi*180)/Math.PI;

        Arm.CurrPos1 = Arm.AngleToPosition(J1Angle);
        Arm.CurrPos2 = Arm.AngleToPosition(J2Angle);

        */




        // write position values to the servo
        Arm.ArmClaw.setPosition(Arm.CurrPosArmClaw);
        Arm.Wrist.setPosition(Arm.CurrPosWrist);
        Arm.ArmServo1.setPosition(Arm.CurrPos1);
        Arm.ArmServo2.setPosition(Arm.CurrPos2);

        telemetry.addData("Arm Servo 1 Position: ", Arm.ArmServo1.getPosition());
        telemetry.addData("Arm Servo 2 Position: ", Arm.ArmServo2.getPosition());
        //telemetry.addData("Wrist Position: ", Arm.Wrist.getPosition());
        telemetry.addData("J1 X: ", J1x);
        telemetry.addData("J1 Y: ", J1y);
        telemetry.addData("J2 X: ", J2x);
        telemetry.addData("J2 Y: ", J2y);

    }

    public void Sleep(int time){
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            ++Arm.sleepErrors;
        }
    }
}

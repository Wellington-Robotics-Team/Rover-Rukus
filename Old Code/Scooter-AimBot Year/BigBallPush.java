package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by David on 1/26/2017.
 */

public class BigBallPush extends LinearOpMode {
    private HardwareMap hwMap = null;
    public double power = 0.5;
    private double ServoTime = 1.14;
    private Servo PushServo;
    private ColorSensor colorSensorRight;
    private ColorSensor colorSensorLeft;
    private ColorSensor colorSensorWhite;
    public int sleepErrors = 0;
    private double ServoMiddle = 0.495;
    private DcMotor LeftMotor  = null;
    private DcMotor RightMotor = null;
    private UltrasonicSensor USLong;
    private UltrasonicSensor USShort;
    //private ModernRoboticsI2cRangeSensor USFront;
    private float WhitehsvValues[] = {0F,0F,0F};
    private int TooClose = 6;
    private int TooFar = 12;
    private int Ideal = 18;
    private int GoodDistance = 25;

    public void runOpMode() throws InterruptedException {
        hwMap = hardwareMap;
        LeftMotor = hwMap.dcMotor.get("left");
        RightMotor = hwMap.dcMotor.get("right");
        PushServo = hwMap.servo.get("Button Pusher");
        colorSensorRight = hardwareMap.colorSensor.get("ccRight");
        colorSensorLeft = hardwareMap.colorSensor.get("ccLeft");
        colorSensorWhite = hardwareMap.colorSensor.get("ccWhite");
        USLong = hardwareMap.ultrasonicSensor.get("Distance Long");
        USShort = hardwareMap.ultrasonicSensor.get("Distance Short");

        waitForStart();

        Color.RGBToHSV(colorSensorWhite.red(), colorSensorWhite.green(), colorSensorWhite.blue(), WhitehsvValues);

        GoScooter(0.4,-0.4);

        Thread.sleep(2000);

        StopScooter();

        GoScooter(-0.5,-0.5);

        Thread.sleep(3200);

        StopScooter();


    }

    public void StopScooter(){
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }

    public void GoScooter(double LeftPower, double RightPower){
        LeftMotor.setPower(LeftPower);
        RightMotor.setPower(RightPower * 1.15);
    }


}


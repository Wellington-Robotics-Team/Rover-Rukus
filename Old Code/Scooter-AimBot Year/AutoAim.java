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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by David on 12/12/2016.
 */

public class AutoAim extends LinearOpMode {
    private HardwareMap hwMap = null;
    public double power = 0.5;
    private double ServoTime = 1.14;
    private Servo PushServo;
    private ColorSensor colorSensor;
    private ColorSensor colorSensorLeft;
    private ColorSensor colorSensorWhite;
    private TouchSensor touchSensor;
    public int sleepErrors = 0;
    private double ServoMiddle = 0.495;
    private DcMotor LeftMotor  = null;
    private DcMotor RightMotor = null;
    private UltrasonicSensor USLong;
    private UltrasonicSensor USShort;
    //private ModernRoboticsI2cRangeSensor USFront;
    private float WhitehsvValues[] = {0F,0F,0F};
    private float hsvValues[] = {0F,0F,0F};
    private int TooClose = 6;
    private int TooFar = 12;
    private int Ideal = 18;
    private int GoodDistance = 25;
    public boolean line = false;


    public void runOpMode() throws InterruptedException {
        hwMap = hardwareMap;
        LeftMotor = hwMap.dcMotor.get("DriveMotorsLeft");
        RightMotor = hwMap.dcMotor.get("DriveMotorsRight");
        //PushServo = hwMap.servo.get("Button Pusher");
        colorSensor = hardwareMap.colorSensor.get("Beacon Sensor");
        //colorSensorLeft = hardwareMap.colorSensor.get("ccLeft");
        colorSensorWhite = hardwareMap.colorSensor.get("Line Sensor");
        //touchSensor = hardwareMap.touchSensor.get("Stop");
        //USLong = hardwareMap.ultrasonicSensor.get("Distance Long");
        //USShort = hardwareMap.ultrasonicSensor.get("Distance Short");
        //USFront = hardwareMap.get (ModernRoboticsI2cRangeSensor.class,"Distance Front");
        //LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        Color.RGBToHSV(colorSensorWhite.red(), colorSensorWhite.green(), colorSensorWhite.blue(), WhitehsvValues);
        Color.RGBToHSV(colorSensor.red(),colorSensor.green(), colorSensor.blue(), hsvValues);

        /*while (colorSensorWhite.red() < 80) { // Get color value change when on the white line...
            GoScooter(0.45, 0.45);

        }

        StopScooter();

        Thread.sleep(1000);

        GoScooter(0.4,-0.4); //Turn

        Thread.sleep(1000); // Turn

        StopScooter(); //pointed at beacon

        Thread.sleep(250); // pointed at beacon

        GoScooter(0.5,0.5);  // CHARGE AT THE BEACON!!!

        Thread.sleep(500); // GO GO GO!!!

        StopScooter(); // stop

        if(colorSensor.red() > 3){
            GoScooter(-0.5,-0.5); // Back away from the beacon

            Thread.sleep(500);  // Back away from the beacon

            StopScooter(); // prep for the next charge

            GoScooter(0.5,0.5); // CHARGE!!!

            Thread.sleep(500); // NEVER STOP!!!

            StopScooter(); // stop...
        }*/

        while (colorSensorWhite.red() < 80) { // Get color value change when on the white line...
            GoScooter(0.45, 0.45);
        }
            double runtime = getRuntime();
            double time = getRuntime();
            while (runtime < (time+5)){
                while (colorSensorWhite.red() > 80 && runtime < (time+5)) {
                    GoScooter(0.1,0.1);
                    time = getRuntime();
                }

                while (colorSensorWhite.red() < 80 && runtime < (time+5)){
                    GoScooter(0.1,-0.05);
                    time = getRuntime();
                }

                time = getRuntime();
            }

            StopScooter();



        //while ((USLong.getUltrasonicLevel() - USShort.getUltrasonicLevel()) > Ideal){
        //    GoScooter(0.3,0.3);
        //}

        //while (USFront.getDistance(DistanceUnit.CM) > GoodDistance){
        //    GoScooter(0.3,-0.3);
        //}

        //if ((USLong.getUltrasonicLevel() - USShort.getUltrasonicLevel()) == "Ideal" && USShort.getUltrasonicLevel() > "Too Close" && USShort.getUltrasonicLevel() < "Too Far"){
        //    while (colorSensorLeft.red() < 200) {
        //        LeftMotor.setPower(0.3);
        //        RightMotor.setPower(-0.3);
        //    }
        //}

        telemetry.addData("Distance Short(cm): ", USShort.getUltrasonicLevel());
        telemetry.addData("Distance Long(cm): ", USLong.getUltrasonicLevel());
        telemetry.update();

        //for (int i = 0; i < 10; i++){
        //    Thread.sleep(50);
        //if (USShort.getUltrasonicLevel() > TooClose && USShort.getUltrasonicLevel() < TooFar) {
        //    LeftMotor.setPower(-0.4);
        //    continue;
        //}
        //    if (USShort.getUltrasonicLevel() > TooClose && USShort.getUltrasonicLevel() > TooFar) {
        //        while((USLong.getUltrasonicLevel() - USShort.getUltrasonicLevel()) > Ideal) {
        //            LeftMotor.setPower(-0.3);
        //RightMotor.setPower(0.2);
        //            telemetry.update();
        //        }
        //        continue;
        //    }
        //    if (USShort.getUltrasonicLevel() < TooClose && USShort.getUltrasonicLevel() < TooFar) {
        //        RightMotor.setPower(0.4);
        //        telemetry.update();
        //    }
        //    else{
        //        PushServo.setPosition(1);
        //        try {
        //            Thread.sleep((long) (ServoTime * 1000));
        //        } catch (InterruptedException ex) {
        //            Thread.currentThread().interrupt();
        //            ++sleepErrors;
        //        }
        //        PushServo.setPosition(0.495);
        //    }
        //}

        telemetry.update();


        //GoScooter(0.3,-0.3);

        Thread.sleep(1000);

        //while (colorSensorLeft.red() < 200) {
        //    LeftMotor.setPower(0.3);
        //    RightMotor.setPower(-0.3);
        //}

        StopScooter();


        //for (int i = 0; i < 5; i++)
        //{
        //    while (colorSensorWhite.red() < 20) {
        //        LeftMotor.setPower(0.3);
        //        RightMotor.setPower(0.3);

        //    }
        //    LeftMotor.setPower(0.3);
        //    RightMotor.setPower(-0.3);
        //    Thread.sleep(100);
        //    LeftMotor.setPower(0);
        //    RightMotor.setPower(0);
        //}

        //do
        //{
        //    PrevDistance = ultrasonicSensor.getUltrasonicLevel();
        //    Thread.sleep(100);
        //    CurrentDistance = ultrasonicSensor.getUltrasonicLevel();
        //} while ( CurrentDistance <  PrevDistance );


        StopScooter();





    }

    public void StopScooter(){
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }

    public void GoScooter(double LeftPower, double RightPower){
        LeftMotor.setPower(-LeftPower);
        RightMotor.setPower(-RightPower);
    }
}
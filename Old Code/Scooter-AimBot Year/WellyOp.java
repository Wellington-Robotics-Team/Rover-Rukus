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
 * Created by David on 10/3/2016.
 */

public class WellyOp extends OpMode {
    private HardwareMap hwMap = null;
    private double prevvalleft = 0;
    private double prevvalright = 0;
    private DcMotor frontLeftMotor  = null;
    //public DcMotor backLeftMotor   = null;
    private DcMotor frontRightMotor = null;
    //public DcMotor backRightMotor  = null;
    public double power = 0.5;
    private double diffleft = 0;
    private double diffright = 0;
    private double ServoMiddle = 0.495;
    private boolean slowsdownfast = false;
    public int sleepErrors = 0;
    private double ServoTime = 1.14;
    private Servo PushServo;
    public ColorSensor colorSensor1;
    public ColorSensor colorSensor2;
    public ColorSensor colorSensorWhite;
    public UltrasonicSensor USLong;
    public UltrasonicSensor USShort;
    public ModernRoboticsI2cRangeSensor USFront;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    // values is a reference to the hsvValues array.
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    float hsvValues1[] = {0F,0F,0F};
    float hsvValues2[] = {0F,0F,0F};
    float WhitehsvValues[] = {0F,0F,0F};
    //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
    //final float values[] = hsvValues;
    public void init() {
        hwMap = hardwareMap;
        frontLeftMotor = hwMap.dcMotor.get("left");
        //backLeftMotor = hwMap.dcMotor.get("BLM");
        frontRightMotor = hwMap.dcMotor.get("right");
        //backRightMotor = hwMap.dcMotor.get("BRM");
        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PushServo = hwMap.servo.get("Button Pusher");
        colorSensor1 = hardwareMap.colorSensor.get("ccRight");
        colorSensor2 = hardwareMap.colorSensor.get("ccLeft");
        colorSensorWhite = hardwareMap.colorSensor.get("ccWhite");
        USLong = hardwareMap.ultrasonicSensor.get("Distance Long");
        USShort = hardwareMap.ultrasonicSensor.get("Distance Short");
        USFront = hardwareMap.get (ModernRoboticsI2cRangeSensor.class,"Distance Front");
        colorSensor1.enableLed(false);
        colorSensor2.enableLed(false);
        colorSensorWhite.enableLed(true);


    }

    public void loop() {


        frontLeftMotor.setPower(gamepad1.left_stick_y * -1 * power);
        frontRightMotor.setPower(gamepad1.right_stick_y * power);
        double currentpowerright = (gamepad1.right_stick_y * power);
        double currentpowerleft = (gamepad1.left_stick_y * power);
        while (slowsdownfast) {
            double currentpowerright1 = (gamepad1.right_stick_y * power);
            double currentpowerleft1 = (gamepad1.left_stick_y * power);
            if (currentpowerleft >= currentpowerleft1) {
                currentpowerleft = currentpowerleft - .01;
            }
            if (currentpowerright >= currentpowerright1) {
                currentpowerright = currentpowerright - .01;
            }
            if (currentpowerright == currentpowerright1 && currentpowerleft == currentpowerleft1) {
                slowsdownfast = false;
            }
            frontLeftMotor.setPower(currentpowerleft);
            frontRightMotor.setPower(currentpowerright);
        }
        if (gamepad1.right_trigger == 1) {
            while (gamepad1.right_trigger == 1) ;
            power = power + 0.1;
            if (power > 1) power = 1;
        }

        if (gamepad1.left_trigger == 1) {
            while (gamepad1.left_trigger == 1) ;
            power = power - 0.1;
            if (power < 0) power = 0;
        }

        if (gamepad1.x) {
            while (gamepad1.x) ;
            power = 0;
        }

        if (gamepad1.y) {
            while (gamepad1.y) ;
            power = 0.7;
        }

        if (gamepad1.a) {
            while (gamepad1.a) ;
            power = 0.4;
        }

        while (gamepad1.right_bumper) {
            frontLeftMotor.setPower(0.4);
            frontRightMotor.setPower(0.4);
        }

        while (gamepad1.left_bumper) {
            frontLeftMotor.setPower(-0.4);
            frontRightMotor.setPower(-0.4);
        }
        diffleft = (gamepad1.left_stick_y * -1 * power) - prevvalleft;
        diffright = (gamepad1.right_stick_y * power) - prevvalright;
        prevvalleft = gamepad1.left_stick_y * -1 * power;
        prevvalright = gamepad1.right_stick_y * power;
        if (Math.abs(diffright) <= -0.1 && Math.abs(diffleft) <= -0.1) {
            slowsdownfast = true;
        } else {
            slowsdownfast = false;
        }

        //if (gamepad1.dpad_right) {
        //    PushServo.setPosition(1);
        //    try {
        //        Thread.sleep((long) (ServoTime * 1000));
        //    } catch (InterruptedException ex) {
        //        Thread.currentThread().interrupt();
        //        ++sleepErrors;
        //    }
        //    PushServo.setPosition(0.495);

        //}

        //if (gamepad1.dpad_left) {
        //    PushServo.setPosition(1);
        //    try {
        //        Thread.sleep((long) (ServoTime * 1000));
        //    } catch (InterruptedException ex) {
        //        Thread.currentThread().interrupt();
        //        ++sleepErrors;
        //    }
        //    PushServo.setPosition(0.495);
        //}

        // Colors are converted from RGB values to HSV values
        Color.RGBToHSV(colorSensor1.red(), colorSensor1.green(), colorSensor1.blue(), hsvValues1);
        Color.RGBToHSV(colorSensor2.red(), colorSensor2.green(), colorSensor2.blue(), hsvValues2);
        Color.RGBToHSV(colorSensorWhite.red(), colorSensorWhite.green(), colorSensorWhite.blue(), WhitehsvValues);

        // if (colorSensor1.blue() > 200 && colorSensor1.red() < colorSensor1.blue() ) {
        //     PushServo.setPosition(1);
        //     try {
        //         Thread.sleep((long) (ServoTime * 1000));
        //     } catch (InterruptedException ex) {
        //         Thread.currentThread().interrupt();
        //         ++sleepErrors;
        //     }
        //     PushServo.setPosition(0.495);

    //  }
        telemetry.addData("Clear (Right): ", colorSensor1.alpha());
        telemetry.addData("Red (Right): ", colorSensor1.red());
        telemetry.addData("Green (Right): ", colorSensor1.green());    // Data for right Sensor...
        telemetry.addData("Blue (Right): ", colorSensor1.blue());
        telemetry.addData("Hue (Right): ", hsvValues1[0]);

        telemetry.addData("Clear (Left): ", colorSensor2.alpha());
        telemetry.addData("Red (Left): ", colorSensor2.red());        // Data for left Sensor...
        telemetry.addData("Green (Left): ", colorSensor2.green());
        telemetry.addData("Blue (Left): ", colorSensor2.blue());
        telemetry.addData("Hue (Left): ", hsvValues2[0]);

        telemetry.addData("Clear (Under): ", colorSensorWhite.alpha());
        telemetry.addData("Red (Under): ", colorSensorWhite.red());
        telemetry.addData("Green (Under): ", colorSensorWhite.green());   // Data for white line Sensor...
        telemetry.addData("Blue (Under): ", colorSensorWhite.blue());
        telemetry.addData("Hue (Under): ", WhitehsvValues[0]);

        //telemetry.addData("Distance(cm): ", ultrasonicSensor.getUltrasonicLevel());
        telemetry.addData("Distance Long(cm): ", USLong.getUltrasonicLevel());
        telemetry.addData("Distance Short(cm): ", USShort.getUltrasonicLevel());
        telemetry.addData("Distance Front(cm): ", USFront.getDistance(DistanceUnit.CM));

        //relativeLayout.post(new Runnable() {
        //    public void run() {
        //        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        //    }
        //});
        telemetry.addData("Sleep Errors: ", sleepErrors);
        telemetry.addData("Power: ", power);
        telemetry.update();
        // Full Stop, will slow to a stop...
        //if (frontLeftMotor.getPower() == 100 && frontRightMotor.getPower() == 100){
        //if (gamepad1.a == true) {
        //for (int i = 100; gamepad1.left_stick_y <= 0 && gamepad1.right_stick_y <= 0;i--) {
        //frontLeftMotor.setPower(i-1);
        //frontRightMotor.setPower(i-1);
        //try {
        //    wait(10);
        //} catch (InterruptedException e) {
        //    e.printStackTrace();
        //}
        //}
        //}
    }
}


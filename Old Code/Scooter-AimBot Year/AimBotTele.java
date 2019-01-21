package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

//import javafx.scene.media.Media;
//import javafx.scene.media.MediaPlayer;


import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by David on 2/2/2017.
 */

public class AimBotTele extends OpMode {
    HardwareMap hwMap = null;
    public double power = 0.5;
    public DcMotor Left = null;
    public DcMotor Right = null;
    public ColorSensor colorSensor = null;
    public ColorSensor colorSensorWhite = null;
    private float WhitehsvValues[] = {0F,0F,0F};
    private float hsvValues[] = {0F,0F,0F};

    public void init(){
        hwMap = hardwareMap;
        colorSensor = hwMap.colorSensor.get("Beacon Sensor");
        colorSensorWhite = hwMap.colorSensor.get("Line Sensor");
        Left = hwMap.dcMotor.get("DriveMotorsLeft");
        //backLeftMotor = hwMap.dcMotor.get("BLM");
        Right = hwMap.dcMotor.get("DriveMotorsRight");
        //backRightMotor = hwMap.dcMotor.get("BRM");

        colorSensor.enableLed(false);
    }

    public void loop(){
        Left.setPower(gamepad1.left_stick_y*power);
        Right.setPower(gamepad1.right_stick_y*power);

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

        while (gamepad1.right_bumper) {
            Left.setPower(power);
            Right.setPower(-power);
        }

        while (gamepad1.left_bumper) {
            Left.setPower(-power);
            Right.setPower(power);
        }

        telemetry.addData("Power: ", power);

        telemetry.addData("Clear: ", colorSensor.alpha());
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());    // Data for main Sensor...
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Hue: ", hsvValues[0]);

        telemetry.addData("Clear (Under): ", colorSensorWhite.alpha());
        telemetry.addData("Red (Under): ", colorSensorWhite.red());
        telemetry.addData("Green (Under): ", colorSensorWhite.green());   // Data for white line Sensor...
        telemetry.addData("Blue (Under): ", colorSensorWhite.blue());
        telemetry.addData("Hue (Under): ", WhitehsvValues[0]);

        telemetry.update();

    }


}

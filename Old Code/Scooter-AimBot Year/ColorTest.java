package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;



/**
 * Created by David on 12/14/2016.
 */

public class ColorTest extends OpMode {
    public HardwareMap hwMap = null;
    public ColorSensor colorSensor;
    public UltrasonicSensor ultrasonicSensor;
    public ColorSensor colorSensorWhite;

    float hsvValues[] = {0F,0F,0F};
    float WhitehsvValues[] = {0F,0F,0F};

    public void init() {
        hwMap = hardwareMap;
        colorSensor = hwMap.colorSensor.get("cc");
        ultrasonicSensor = hwMap.ultrasonicSensor.get("Distance");
        colorSensorWhite = hardwareMap.colorSensor.get("ccWhite");
        colorSensor.enableLed(false);
    }

    public void loop() {
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
        Color.RGBToHSV(colorSensorWhite.red(), colorSensorWhite.green(), colorSensorWhite.blue(), WhitehsvValues);

        telemetry.addData("Clear: ", colorSensor.alpha());
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Hue: ", hsvValues[0]);
        telemetry.addData("Ultra Sonic Value: ", ultrasonicSensor.getUltrasonicLevel());
        telemetry.addData("Clear (Under): ", colorSensorWhite.alpha());
        telemetry.addData("Red (Under): ", colorSensorWhite.red());
        telemetry.addData("Green (Under): ", colorSensorWhite.green());
        telemetry.addData("Blue (Under): ", colorSensorWhite.blue());
        telemetry.addData("Hue (Under): ", WhitehsvValues[0]);

        telemetry.update();

    }
}

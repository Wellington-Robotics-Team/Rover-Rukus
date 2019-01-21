package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by David on 12/14/2016.
 */

public class TouchTest extends OpMode {
    private HardwareMap hwMap = null;
    private TouchSensor touchSensor = null;
    private TouchSensor touchSensor2 = null;
    private boolean Touched = false;
    private boolean Touched2 = false;


    public void init() {
        hwMap = hardwareMap;
        touchSensor = hwMap.touchSensor.get("touchy");
        touchSensor2 = hwMap.touchSensor.get("touchy2");
    }

    public void loop() {
        telemetry.addData("Touch 1: ", touchSensor.isPressed());
        telemetry.addData("Touch 2: ", touchSensor2.isPressed());

        telemetry.update();
    }
}

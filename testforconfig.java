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
 * Created by David on 9/29/2017.
 */

public class testforconfig extends OpMode {
    private HardwareMap hwMap = null;
    private DcMotor FRM = null;
    private DcMotor FLM = null;
    private DcMotor BRM = null;
    private DcMotor BLM = null;
    public double power = 0.5;

    public void init() {
        hwMap = hardwareMap;
        FLM = hwMap.dcMotor.get("FLM");
        FRM = hwMap.dcMotor.get("FRM");
        BRM = hwMap.dcMotor.get("BRM");
        BLM = hwMap.dcMotor.get("BLM");
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(){

        FRM.setPower(gamepad1.right_stick_y);
        FLM.setPower(gamepad1.left_stick_y);
        BRM.setPower(gamepad1.right_stick_x);
        BLM.setPower(gamepad1.left_stick_x);

    }

}

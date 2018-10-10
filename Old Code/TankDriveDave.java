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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by David on 1/18/2018.
 */

public class TankDriveDave extends OpMode{
    private HardwareMap hwMap;
    private DcMotor Left;
    private DcMotor Right;
    private double power = 0.5;

    public void init(){
        hwMap = hardwareMap;

        Left = hwMap.dcMotor.get("left");
        Right = hwMap.dcMotor.get("right");

    }

    public void loop(){

        Left.setPower(power*gamepad1.left_stick_y);
        Right.setPower(-power*gamepad1.right_stick_y);


        if (gamepad1.right_trigger == 1) {
            while (gamepad1.right_trigger == 1) ;
            power = power + 0.1;
            if (power >= 1) power = 1;
        }
        if (gamepad1.left_trigger == 1) {
            while (gamepad1.left_trigger == 1) ;
            power = power - 0.1;
            if (power <= 0) power = 0;
        }

        telemetry.addData("Power: ", power);


    }

}

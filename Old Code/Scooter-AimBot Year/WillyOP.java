package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by David on 10/3/2016.
 */

public class WillyOP extends OpMode {
    HardwareMap hwMap = null;

    public DcMotor frontLeftMotor  = null;
    //public DcMotor backLeftMotor   = null;
    public DcMotor frontRightMotor = null;
    //public DcMotor backRightMotor  = null;
    public double power = 0.5;

    public void init() {
        hwMap = hardwareMap;
        frontLeftMotor = hwMap.dcMotor.get("left");
        //backLeftMotor = hwMap.dcMotor.get("BLM");
        frontRightMotor = hwMap.dcMotor.get("right");
        //backRightMotor = hwMap.dcMotor.get("BRM");


    }
    public void loop() {

        frontLeftMotor.setPower(gamepad1.left_stick_y*power);
        frontRightMotor.setPower(gamepad1.right_stick_y*-1*power);

        //if (gamepad1.right_trigger == 1){
        //    while (gamepad1.right_trigger == 1);
        //    power = power + 0.05;
        //    if (power > 1) power = 1;
        //}

        //if (gamepad1.left_trigger == 1){
        //    while (gamepad1.left_trigger == 1);
        //    power = power - 0.05;
        //    if (power < 0) power = 0;
        //}
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

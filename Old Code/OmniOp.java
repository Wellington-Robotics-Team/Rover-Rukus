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
 * Created by David on 9/29/2017.
 */

public class OmniOp extends OpMode {

    private HardwareMap hwMap = null;   // Making HardwareMap...
    private DcMotor FRM = null;
    private DcMotor FLM = null;
    private DcMotor BRM = null;      // Making Motors...
    private DcMotor BLM = null;
    private double ServoTime = 1.14;
    private Servo ArmServo1 = null;
    private Servo ArmServo2 = null;
    private double power = 0.5;
    private int sleepErrors = 0;
    private double GyroCaliTime = 0;           // These are Values used by the Motors and the Gyro...
    private int rawX;
    private int rawY;
    private int rawZ;
    private int heading;
    private int integratedZ;
    private AngularVelocity rates;
    private float zAngle;
    private int zAxisOffset;
    private int zAxisScalingCoefficient;

    private  ElapsedTime timer = new ElapsedTime();   // Instantiating Timer...

    // Making Gyro...
    private ModernRoboticsI2cGyro gyro;

    //public boolean DpadMode = false;

    public void init() {
        hwMap = hardwareMap;
        FLM = hwMap.dcMotor.get("FLM");
        FRM = hwMap.dcMotor.get("FRM");
        BRM = hwMap.dcMotor.get("BRM");
        BLM = hwMap.dcMotor.get("BLM");
        //ArmServo1 = hwMap.servo.get("ArmServo1");
        //ArmServo2 = hwMap.servo.get("ArmServo2");
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);            // Instantiation of Motors on Robot...

        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        // This Forces the Motors to run with Encoders...
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");   // Instantiating Gyro...

        timer.reset();
        telemetry.log().add("Gyro Calibrating. Do Not Move!");    // Gyro Calibration...
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            telemetry.addData("Gyro", "Calibrating");
            telemetry.update();
        }

        GyroCaliTime = timer.seconds();   // This is so we can keep track of the time it takes for Gyro Calibration to complete...

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();
    }

    public void start() {
        telemetry.log().clear();


        timer.reset();      // This is so we can keep track of how long the loop() to complete...
    }

    public void loop() {

        //if (gamepad1.start && !DpadMode) DpadMode = true;

        //if (gamepad1.start && DpadMode) DpadMode = false;

        rawX = gyro.rawX();
        rawY = gyro.rawY();
        rawZ = gyro.rawZ();
        heading = gyro.getHeading();
        integratedZ = gyro.getIntegratedZValue();

        rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        zAxisOffset = gyro.getZAxisOffset();
        zAxisScalingCoefficient = gyro.getZAxisScalingCoefficient();

        if (gamepad1.a){
            Turn(90);
        }


        FRM.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) * power);
        FLM.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) * power);
        BRM.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) * power);     // This is the code I made to make the bot go in all directions...
        BLM.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) * power);

        while (gamepad1.right_bumper) {

            FLM.setPower(power);
            BLM.setPower(power);   // This makes the bot turn right...
            FRM.setPower(-power);
            BRM.setPower(-power);

        }

        while (gamepad1.left_bumper) {

            FLM.setPower(-power);
            BLM.setPower(-power);    // This makes the bot turn left
            FRM.setPower(power);
            BRM.setPower(power);

        }

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

        //if (DpadMode) {

            while (gamepad1.dpad_up) {

                FRM.setPower(power);
                FLM.setPower(power);
                BLM.setPower(power);
                BRM.setPower(power);
            }

            while (gamepad1.dpad_down) {

                FRM.setPower(-power);
                FLM.setPower(-power);
                BLM.setPower(-power);
                BRM.setPower(-power);
            }

            while (gamepad1.dpad_right) {

                FRM.setPower(-power);
                FLM.setPower(power);
                BLM.setPower(-power);
                BRM.setPower(power);
            }

            while (gamepad1.dpad_left) {

                FRM.setPower(power);
                FLM.setPower(-power);
                BLM.setPower(power);
                BRM.setPower(-power);
            }
        //}



        telemetry.addData("Power: ",power);
        //telemetry.addData("Dpad Mode: ",DpadMode);

        telemetry.addData("Y Axis: ", gamepad1.left_stick_y);
        telemetry.addData("X axis: ",gamepad1.left_stick_x);

        telemetry.addData("FRM: ", FRM.getPower());
        telemetry.addData("FLM: ", FLM.getPower());
        telemetry.addData("BRM: ", BRM.getPower());
        telemetry.addData("BLM: ", BLM.getPower());

        telemetry.addData("Calibration Time: ", GyroCaliTime);
        telemetry.addData("Time: ", timer.seconds());

        telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
        telemetry.addData("angle", "%s deg", formatFloat(zAngle));
        telemetry.addData("heading", "%3d deg", heading);
        telemetry.addData("integrated Z", "%3d", integratedZ);
        telemetry.addLine()
                .addData("rawX", formatRaw(rawX))
                .addData("rawY", formatRaw(rawY))
                .addData("rawZ", formatRaw(rawZ));
        telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);

        telemetry.update();



    }

    public void Sleep(int time){
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            ++sleepErrors;
        }
    }

    public int GetSignedHeading(){
        int CurrentHeading = gyro.getHeading();

        if (CurrentHeading > 180){
            CurrentHeading -= 360;
        }

        return CurrentHeading;
    }

    public void Turn(int TargetHeading){
        int CurrentHeading = GetSignedHeading();

        while(CurrentHeading < TargetHeading){
            FLM.setPower(power);
            BLM.setPower(power);   // This makes the bot turn right...
            FRM.setPower(-power);
            BRM.setPower(-power);

            CurrentHeading = GetSignedHeading();
        }

    }

    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }

}

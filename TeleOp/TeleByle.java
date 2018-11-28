package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleByle", group = "TeleOp")

public class TeleByle extends OpMode {

    DcMotor FLM = null;
    DcMotor FRM = null;
    DcMotor BLM = null;
    DcMotor BRM = null;
    DcMotor LIFT = null;
    double power = 0.4;
    double liftpowermodifier = 0.5;
    @Override
    public void init() {

        FLM = hardwareMap.dcMotor.get ("FLM");
        FRM = hardwareMap.dcMotor.get ("FRM");
        BLM = hardwareMap.dcMotor.get ("BLM");
        BRM = hardwareMap.dcMotor.get ("BRM");
        LIFT = hardwareMap.dcMotor.get ("LIFT");

        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized") ;

        telemetry.update();
    }

    @Override
    public void loop() {

        double powerLeft; // declares variables for the left and right side
        double powerRight;

        double drive = -gamepad1.left_stick_y; // sets drive to the y axis on the left stick
        double turn = gamepad1.left_stick_x; // sets turn to the x axis on the left stick

        powerLeft = (drive + turn)*powerModifier(power); //powerLeft are both equal to the sum of drive and turn and is multiplied by powerModifier
        powerRight = (drive - turn)*powerModifier(power); //powerRight becomes the difference of drive and turn and is multiplied by powerModifier

        go(powerRight,powerLeft); //go (function that sets the power for each motor) uses powerRight and powerLeft as the inputs

        //if statements are controlling power for ARM

        LIFT.setPower(-gamepad1.right_stick_y * 0.5);
    }

    public void go(double right, double left) {

        FLM.setPower(left); //sets power for front left motor
        BLM.setPower(left); //sets power for back left motor
        FRM.setPower(right); //sets power for front right motor
        BRM.setPower(right); //sets power for back right motor
    }

    public double powerModifier(double power) {

        if (gamepad1.right_trigger > 0) { //if the input from the right trigger is more than 0
            power = power + (0.5 * gamepad1.right_trigger); // power is added to the product of 0.5 (the default power value) and input from the right trigger

            return power; //returns the new value of power
        }

        else if (gamepad1.left_trigger > 0) { //if the input from the right trigger is more than 0
            power = power - (0.5 * gamepad1.left_trigger); // power is added to the product of 0.5 (the default power value) and input from the right trigger

            return power; //returns the new value of power
        }

        else {
            return power; //returns the default value of power (0.5)
        }
    }

}


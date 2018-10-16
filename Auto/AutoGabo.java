package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

//Made by Gabo on Oct 15 2018

public class AutoGabo extends LinearOpMode {
    public HardwareMap hwMap;
    public DcMotor BLM;
    public DcMotor BRM;
    public DcMotor FRM;
    public DcMotor FLM;
    public TouchSensor LTS;
    public TouchSensor RTS;

    public void runOpMode()
    {
        hwMap = hardwareMap;

        BLM = hwMap.dcMotor.get("BLM"); //gets all the motors
        BRM = hwMap.dcMotor.get("BRM");
        FRM = hwMap.dcMotor.get("FRM");
        FLM = hwMap.dcMotor.get("FLM");
        RTS = hwMap.touchSensor.get("RTS"); //gets the 2 touch sensors
        LTS = hwMap.touchSensor.get("LTS");

        telemetry.addData("Working?", " ye"); //very informative telemetry
        telemetry.update();

        waitForStart(); //waits for the start button

        RushB();
    }

    public void RushB()
    {
        while (!RTS.isPressed() || !LTS.isPressed()) //while either of the sensors are not pressed
        {
            double left = 0.5; //left power is 0.5
            double right = 0.5; //right power is 0.5

            if (RTS.isPressed()) //if right sensor is pressed
            {
                right = 0; //sets right to 0 to stop
                left = 0.3; //left slows down
                telemetry.addData("Right Button", RTS.isPressed()); //informs that its pressed
                telemetry.update();
            }
            if (LTS.isPressed()) //if left sensor is pressed
            {
                left = 0; //left stops
                right = 0.3; //right is stopped
                telemetry.addData("Left Button", LTS.isPressed()); //informs that its pressed
                telemetry.update();
            }

            move(left, right); //moves motors with right left power

        }


        move(0,0); //once both motors are pressed it stops
    }

    public void move(double leftPower, double rightPower)
    {
        BLM.setPower(leftPower);
        FLM.setPower(leftPower);
        FRM.setPower(rightPower);
        BRM.setPower(rightPower);
    }
}

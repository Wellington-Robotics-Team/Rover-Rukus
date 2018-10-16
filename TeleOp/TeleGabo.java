package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Created by Gabo on 10/9/18

public class TeleGabo extends OpMode {
    //Declare variables
    public HardwareMap hwMap;
    public DcMotor BLM;
    public DcMotor BRM;
    public DcMotor FRM;
    public DcMotor FLM;
    //Set power
    public double power = 0.5;
    public double oldPower;
    public boolean fullPower = false;
    public void init() //runs when you press init on phone
    {
        hwMap = hardwareMap;

        BLM = hwMap.dcMotor.get("BLM"); //gets the motors
        BRM = hwMap.dcMotor.get("BRM");
        FRM = hwMap.dcMotor.get("FRM");
        FLM = hwMap.dcMotor.get("FLM");


    }
    public void loop() //runs after you press play
    {
    if (gamepad1.right_bumper) //if the right bumber is pressed
    {
        while (gamepad1.right_bumper && power <= 1) ; //power goes up

        power += 0.1;

        if (power >= 1) power = 1; //doesn't let you go above 1
    }

        if (gamepad1.left_bumper) //if left bumber is pressed
        {
            while (gamepad1.left_bumper && power >= -1) ; //power goes down

                power -= 0.1;
            if (power <= 0) power = 0; //doesn't let it go below 0
        }

        if (gamepad1.left_trigger == 1) { //if the left trigger is fully down
            oldPower = power; //keeps track of old power
            while (gamepad1.left_trigger == 1) //while its pressed
            {
                move(0, 0); //stop robot
            }
            power = oldPower; //once its let go resumes previous power
        }

        if (gamepad1.right_trigger == 1) //if right trigger is down
        {
            if (!fullPower) oldPower = power; //if fullpower is false sets the oldpower to power
            fullPower = true; //sets full power to true
        } else if (fullPower){ //if fullpower is true and trigger is not down
            power = oldPower; //sets power to back to normal
            fullPower = false; //full power is false
        }


        double drive = gamepad1.left_stick_y; //drive is y value
        double turn = gamepad1.left_stick_x; //turn is x value
        if (fullPower) power = 1; //if fullPower is true sets power to 1
        double leftPower = (-drive + turn)* -power; //left power is negative drive plus turn * negative power
        double rightPower = (-drive - turn)* power; //same but positive power

        move(leftPower, rightPower); //moves robot with the power


        telemetry.addData("Power to left motor ", leftPower); //informs leftPower
        telemetry.addData("Power to right motor ", rightPower); //informs right Power
        telemetry.addData("Power ", power); //talks about power
        telemetry.addData("FullPower? ", fullPower); //tells if its fullpower

        telemetry.update(); //updates telemetry
    }
    public void move(double leftPower, double rightPower) //move function for ez
    {
        BLM.setPower(leftPower); //left motors are leftPower
        FLM.setPower(leftPower);
        FRM.setPower(rightPower); //right motors are rightPower
        BRM.setPower(rightPower);
    }

}

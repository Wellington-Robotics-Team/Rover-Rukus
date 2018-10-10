package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by David on 11/28/2017.
 */


public class LiftTest extends OpMode{

    public OmniBot Lifty = new OmniBot();

    public void init(){

        Lifty.hwMap = hardwareMap;
        Lifty.InitializeLift();
        Lifty.InitializeArmServos();
        Lifty.InitializeArmClaw();
        Lifty.ArmServo2.setDirection(Servo.Direction.REVERSE);

        Lifty.CurrPos2 = 0;
        Lifty.CurrPosLiftClaw = 0.3;
        Lifty.CurrPosWrist = 1;
        Lifty.CurrPosArmClaw = 0.5;

        int CurrPosLift = 1;
        int TarPosLift = 1;

    }

    public void loop(){

        Lifty.LiftControl(gamepad1.right_trigger, gamepad1.left_trigger);

        if (gamepad1.dpad_left) {
            Lifty.CurrPos2 += Lifty.servoDelta*2;
        }
        if (gamepad1.dpad_right) {
            Lifty.CurrPos2 -= Lifty.servoDelta*2;
        }



        if (gamepad1.a) Lifty.CurrPosLiftClaw -= Lifty.servoDelta*5;
        if (gamepad1.y) Lifty.CurrPosLiftClaw += Lifty.servoDelta*5;


        Lifty.ArmServo2.setPosition(Lifty.CurrPos2);
        //Lifty.LiftClaw.setPosition(Lifty.CurrPosLiftClaw);

        //telemetry.addData("Lift Claw Position: ", Lifty.LiftClaw.getPosition());
        telemetry.addData("Lift Claw Pos Num: ", Lifty.CurrPosLiftClaw);
        //telemetry.addData("Arm Servo 1 Position: ", Lifty.ArmServo1.getPosition());
        telemetry.addData("Arm Servo 2 Position: ", Lifty.ArmServo2.getPosition());
        telemetry.addData("Arm Servo 2 Pos Num: ", Lifty.CurrPos2);


        telemetry.addData("Time: ", Lifty.timer.seconds());


        telemetry.update();


    }
}

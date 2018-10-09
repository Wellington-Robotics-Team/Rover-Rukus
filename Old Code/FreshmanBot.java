package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * Created by David on 11/13/2017.
 */

public class FreshmanBot extends OpMode {

    private OmniBot FreshmanBot = new OmniBot();
    private boolean DavidDone = true;

    public void init(){

        FreshmanBot.hwMap = hardwareMap;

        FreshmanBot.Initialize();

        FreshmanBot.InitializeArmClaw();

    }

    public void loop(){

        FreshmanBot.TankDrive(gamepad1.right_stick_y, gamepad1.left_stick_y);

        FreshmanBot.TriggerTurn(gamepad1.right_trigger, gamepad1.left_trigger);

        //FreshmanBot.DpadDrive(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left);

        FreshmanBot.PowerChange(gamepad1.right_bumper, gamepad1.left_bumper);

        if (gamepad1.a) FreshmanBot.ArmClaw.setPosition(0);
        if (gamepad1.b) FreshmanBot.ArmClaw.setPosition(0.25);
        if (gamepad1.y) FreshmanBot.ArmClaw.setPosition(0.5);
        if (gamepad1.x) FreshmanBot.ArmClaw.setPosition(0.75);
        if (gamepad1.start) FreshmanBot.ArmClaw.setPosition(1);

        telemetry.addData("Power", FreshmanBot.power);

        telemetry.addData("Claw Position", FreshmanBot.ArmClaw.getPosition());

        telemetry.addData("FRM: ", FreshmanBot.FRM.getPower());
        telemetry.addData("FLM: ", FreshmanBot.FLM.getPower());
        telemetry.addData("BRM: ", FreshmanBot.BRM.getPower());
        telemetry.addData("BLM: ", FreshmanBot.BLM.getPower());

        telemetry.addData("Time: ", FreshmanBot.timer.seconds());

        telemetry.addLine();telemetry.addLine();telemetry.addLine();

        telemetry.addData("Is David Done with Freshman? ",DavidDone);
    }
}

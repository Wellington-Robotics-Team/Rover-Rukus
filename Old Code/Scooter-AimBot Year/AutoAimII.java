package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Template: Linear OpMode", group="Auto")  // @Autonomous(...) is the other common choice

public class AutoAimII extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;
    Robot Aimbot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Aimbot = new Robot(hardwareMap,telemetry, false, false, false,true, false);
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        Aimbot.LeftM.setPower(0);
        Aimbot.RightM.setPower(0);
        waitForStart();

        runtime.reset();
        do{
            Aimbot.LeftM.setPower(-0.1);//the negative shouldn't be needed on aimbot.
            Aimbot.RightM.setPower(0.1);
            telemetry.addData("Status", Aimbot.Sensors.LSense.red());
            telemetry.update();
        }while(!Aimbot.Sensors.Line());
        Aimbot.LeftM.setPower(0.1);
        Aimbot.RightM.setPower(-0.1);
        /*try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } */
        Aimbot.LeftM.setPower(0);
        Aimbot.RightM.setPower(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", Aimbot.Sensors.LSense.red());
            telemetry.update();
            if (Aimbot.Sensors.Line()){
                Aimbot.RightM.setPower(0);//the motors are flipped around for some reason
                Aimbot.LeftM.setPower(-0.1);//flip them back when working on aimbot not redbot
            }
            else{
                Aimbot.LeftM.setPower(-0.05);
                Aimbot.RightM.setPower(0.1);
            }

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
        Aimbot.LeftM.setPower(-0.1);
        Aimbot.RightM.setPower(0);
    }
}

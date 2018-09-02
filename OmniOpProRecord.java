package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 11/2/2017.
 */

public class OmniOpProRecord extends OpMode{

    private OmniBot OmniBoi = new OmniBot();


    public void init(){

        OmniBoi.hwMap = hardwareMap;

        OmniBoi.recorder = new Recorder(0.1, "OmniBoiRecord",gamepad1,gamepad2);

        OmniBoi.Initialize();

        OmniBoi.InitializeOmniWheels();

        //OmniBoi.InitializeGyro();


    }

    public void loop(){

        //OmniBoi.recorder.record(getRuntime(),telemetry);
        OmniBoi.recorder.setGamepads(getRuntime());
        gamepad1 = OmniBoi.recorder.getG1();

        OmniBoi.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //OmniBoi.GyroUpdate();

        //OmniBoi.DpadDriveOmni(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left);

        OmniBoi.ButtonTurn(gamepad1.right_bumper, gamepad1.left_bumper);

        OmniBoi.PowerChange(gamepad1.right_trigger, gamepad1.left_trigger);


        telemetry.addData("Power: ", OmniBoi.power);

        telemetry.addData("Y Axis: ", gamepad1.left_stick_y);
        telemetry.addData("X axis: ",gamepad1.left_stick_x);

        telemetry.addData("FRM: ", OmniBoi.FRM.getPower());
        telemetry.addData("FLM: ", OmniBoi.FLM.getPower());
        telemetry.addData("BRM: ", OmniBoi.BRM.getPower());
        telemetry.addData("BLM: ", OmniBoi.BLM.getPower());

        //telemetry.addData("Calibration Time: ", OmniBoi.GyroCaliTime);
        telemetry.addData("Time: ", OmniBoi.timer.seconds());

        /*telemetry.addLine()
                .addData("dx", OmniBoi.formatRate(OmniBoi.rates.xRotationRate))
                .addData("dy", OmniBoi.formatRate(OmniBoi.rates.yRotationRate))
                .addData("dz", "%s deg/s", OmniBoi.formatRate(OmniBoi.rates.zRotationRate));
        telemetry.addData("angle", "%s deg", OmniBoi.formatFloat(OmniBoi.zAngle));
        telemetry.addData("heading", "%3d deg", OmniBoi.heading);
        telemetry.addData("integrated Z", "%3d", OmniBoi.integratedZ);
        telemetry.addLine()
                .addData("rawX", OmniBoi.formatRaw(OmniBoi.rawX))
                .addData("rawY", OmniBoi.formatRaw(OmniBoi.rawY))
                .addData("rawZ", OmniBoi.formatRaw(OmniBoi.rawZ));
        telemetry.addLine().addData("z offset", OmniBoi.zAxisOffset).addData("z coeff", OmniBoi.zAxisScalingCoefficient);
        */

        telemetry.update();


    }


}

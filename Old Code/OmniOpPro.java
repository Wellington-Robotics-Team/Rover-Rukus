package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by David on 11/2/2017.
 */

public class OmniOpPro extends OpMode{

    private OmniBot OmniBoi = new OmniBot();

    double RestPos1 = 0; //0.53
    double RestPos2 = 0; //0.25
    double J1Offset;
    double J2Offset;
    double J1Angle;
    double J2Angle;
    double J1x;
    double J1y;
    double J2x;
    double J2y;
    private boolean stickcontrol = false;



    private boolean CheckRange(double x, double y) {
        boolean t = true;
        if (x < 5)
        {
            telemetry.addLine("Range Check x < 5");
            t = false;
        }
        if (y > 0.9*(OmniBoi.L1+OmniBoi.L2)) {
            telemetry.addLine("Range Check y > 90% L1+L2");
            t = false;
        }
        if (x > 0.98 * (OmniBoi.L1+OmniBoi.L2))
        {
            telemetry.addLine("Range Check x > 98% L1+L2");
            t = false;
        }
        if (y < -10)
        {
            telemetry.addLine("Range Check y < -10");
            t = false;
        }
        if ((x*x + y*y) > (OmniBoi.MagMax)) {
            telemetry.addLine("Range Check x2+y2 > MagMax");
            t = false;
        }
        return t;
    }





    public void init(){

        OmniBoi.hwMap = hardwareMap;

        //OmniBoi.recorder = new Recorder(0.1, "OmniBoiRecord");

        OmniBoi.Initialize();

        OmniBoi.InitializeOmniWheels();

        OmniBoi.InitializeLift();

        OmniBoi.InitializeArmServos(38, 42);
        //OmniBoi.ArmServo1.scaleRange(0,1.0);
        //OmniBoi.ArmServo2.scaleRange(0,1.0);
        OmniBoi.ArmServo1.setDirection(Servo.Direction.REVERSE);
        OmniBoi.ArmServo2.setDirection(Servo.Direction.REVERSE);
        OmniBoi.InitializeArmClaw();
        OmniBoi.Extender = OmniBoi.hwMap.servo.get("Extender");

        OmniBoi.CurrPos1 = 0; //0.53
        OmniBoi.CurrPos2 = 0; //0.25
        OmniBoi.CurrPosWrist = 0.05;
        OmniBoi.CurrPosArmClaw = 0.5;
        OmniBoi.CurrPosExtender = 0;

        //J2x = 10;
        //J2y = 10;


        //OmniBoi.InitializeGyro();

        OmniBoi.SetShoulder(170);
        OmniBoi.SetElbow(10);


    }

    public void loop(){

        //OmniBoi.recorder.record(getRuntime(),telemetry);
        //OmniBoi.recorder.setGamepads(getRuntime());
        //gamepad1 = OmniBoi.recorder.getG1();

        OmniBoi.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //OmniBoi.TriggerTurn(gamepad1.right_trigger, gamepad1.left_trigger);

        //OmniBoi.GyroUpdate();

        //OmniBoi.DpadDriveOmni(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left);

        //OmniBoi.ButtonTurn(gamepad1.right_bumper, gamepad1.left_bumper);

        //OmniBoi.PowerChange(gamepad1.right_bumper, gamepad1.left_bumper);

        if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper);
            OmniBoi.power = OmniBoi.power + 0.1;
            if (OmniBoi.power >= 1) OmniBoi.power = 1;
        }
        if (gamepad1.left_bumper) {
            while (gamepad1.left_bumper) ;
            OmniBoi.power = OmniBoi.power - 0.1;
            if (OmniBoi.power <= 0) OmniBoi.power = 0;
        }

        OmniBoi.LiftControl(gamepad1.right_trigger, gamepad1.left_trigger);

        //OmniBoi.LiftClawControl(gamepad1.y, gamepad1.a, 1, 0.7);
        /*
        if (gamepad1.a) OmniBoi.CurrPosLiftClaw = 0;
        if (gamepad1.b) OmniBoi.CurrPosLiftClaw = 0.25;
        if (gamepad1.y) OmniBoi.CurrPosLiftClaw = 0.5;
        if (gamepad1.x) OmniBoi.CurrPosLiftClaw = 0.75;
        if (gamepad1.start) OmniBoi.CurrPosLiftClaw = 1;
        */

        //if (gamepad1.a) OmniBoi.CurrPosLiftClaw -= OmniBoi.servoDelta*5;
        //if (gamepad1.y) OmniBoi.CurrPosLiftClaw += OmniBoi.servoDelta*5;

        /*
        if (gamepad1.a) {
            while (gamepad1.a);
            OmniBoi.LiftClaw.setPower(0.05);
        }
        if (gamepad1.y) {
            while (gamepad1.y);
            OmniBoi.LiftClaw.setPower(-0.05);
        }
        */



        if (gamepad1.x) OmniBoi.CurrPosExtender = 0;
        if (gamepad1.b) OmniBoi.CurrPosExtender = 1;



        /*
        if (gamepad1.dpad_down) {
            OmniBoi.CurrPos1 += OmniBoi.servoDelta*2;
        }                                           //Just in case...
        if (gamepad1.dpad_up) {
            OmniBoi.CurrPos1 -= OmniBoi.servoDelta*2;
        }

        if (gamepad1.dpad_left) {
            OmniBoi.CurrPos2 += OmniBoi.servoDelta*2;
        }
        if (gamepad1.dpad_right) {
            OmniBoi.CurrPos2 -= OmniBoi.servoDelta*2;
        }

        */


        //OmniBoi.ArmControl(gamepad2.left_stick_x*2, gamepad2.left_stick_y*2, gamepad2.right_stick_y*3, gamepad2.right_trigger*3, gamepad2.left_trigger*3);





        /*
        if (gamepad2.a) OmniBoi.CurrPosWrist = 1;
        if (gamepad2.y) OmniBoi.CurrPosWrist = 0.5;
        if (gamepad2.x) OmniBoi.CurrPosArmClaw = 1;
        if (gamepad2.b) OmniBoi.CurrPosArmClaw = 0.5;
        */


        /*
        if (gamepad2.x){
            J2x = 10;  // Start Position...
            J2y = 10;
        }

        if (gamepad2.b){
            OmniBoi.SetServo1((Math.PI*17)/18); // Rest Position...
            OmniBoi.SetServo2(Math.PI/18);
        }

        if (gamepad2.y){
            J2x = OmniBoi.L1 + OmniBoi.L2;  // Deploy Idol to "3" Zone...
            J2y = 0;
        }
        */


        /*

        if (gamepad2.a){
            while (gamepad2.a);
            OmniBoi.CurrPos1 = OmniBoi.CurrPos1 + (0.25/3);
        }
        if (gamepad2.y) {
            while (gamepad2.y);
            OmniBoi.CurrPos1 = OmniBoi.CurrPos1 - (0.25/3);
        }
        if (gamepad2.x) {
            while (gamepad2.x);
            OmniBoi.CurrPos2 = OmniBoi.CurrPos2 + (0.25/3);
        }
        if (gamepad2.b) {
            while (gamepad2.b);
            OmniBoi.CurrPos2 = OmniBoi.CurrPos2 - (0.25/3);
        }

        */




        if (OmniBoi.CurrPos1 > 0.53) OmniBoi.CurrPos1 = 0.53;
        if (OmniBoi.CurrPos2 > 0.25) OmniBoi.CurrPos2 = 0.25;
        if (OmniBoi.CurrPos1 < 0) OmniBoi.CurrPos1 = 0;        // Clipping the ranges of the joints on the Arm...
        if (OmniBoi.CurrPos2 < 0) OmniBoi.CurrPos2 = 0;

        //Range.clip(OmniBoi.CurrPosWrist, 0.5, 1);
        //Range.clip(OmniBoi.CurrPosArmClaw, 0.5, 1);

        /*
        if (OmniBoi.CurrPos2 < 0.5) OmniBoi.CurrPosArmClaw = 0.5;
        if (OmniBoi.CurrPos2 > 1) OmniBoi.CurrPosArmClaw = 1;
        if (OmniBoi.CurrPos2 < 0.5) OmniBoi.CurrPosWrist = 0.5;
        if (OmniBoi.CurrPos2 > 1) OmniBoi.CurrPosWrist = 1;
        */


        OmniBoi.Wrist.setPosition(OmniBoi.CurrPosWrist);
        OmniBoi.ArmClaw.setPosition(OmniBoi.CurrPosArmClaw);
        OmniBoi.Extender.setPosition(OmniBoi.CurrPosExtender);
        //OmniBoi.ArmServo1.setPosition(OmniBoi.CurrPos1);
        //OmniBoi.ArmServo2.setPosition(OmniBoi.CurrPos2); // 0.6 is the scalar unit of the servo...


        /*

        // Alright, time to figure out the X's and Y's of this bot... Forward Kinematics (NOT OPERABLE)
        J1Angle = PositionToAngle(OmniBoi.CurrPos1, J1Offset);
        J2Angle = PositionToAngle(OmniBoi.CurrPos2, J2Offset);

        J1x = Math.cos(Math.toRadians(((J1Angle*Math.PI)/180)))*OmniBoi.L1;
        J1y = Math.sin(Math.toRadians(((J1Angle*Math.PI)/180)))*OmniBoi.L1;
        J2x = Math.cos(Math.toRadians((((J2Angle+J1Angle)*Math.PI)/180)))*(OmniBoi.L1+J1x);
        J2y = Math.sin(Math.toRadians((((J2Angle+J1Angle)*Math.PI)/180)))*(OmniBoi.L1+J1y);

        //OmniBoi.ArmServo1.setPosition(OmniBoi.CurrPos1);
        //OmniBoi.ArmServo2.setPosition(OmniBoi.CurrPos2);

        //telemetry.addData("Arm Servo 1 Position: ", OmniBoi.ArmServo1.getPositio n());
        //telemetry.addData("Arm Servo 2 Position: ", OmniBoi.ArmServo2.getPosition());

        */



        /*
        // Version 2 of the Forward Kinematics Operation...
        J1Angle = OmniBoi.CurrPos1*(Math.PI*2);
        J2Angle = -OmniBoi.CurrPos2*(Math.PI*2);

        J1x = Math.cos(J1Angle)*OmniBoi.L1;
        J1y = Math.sin(J1Angle)*OmniBoi.L1;
        J2x = Math.cos(J2Angle+J1Angle)*(OmniBoi.L2)+J1x;
        J2y = Math.sin(J2Angle+J1Angle)*(OmniBoi.L2)+J1y;
        */






        /*
        if (gamepad2.dpad_down) J2y = J2y-OmniBoi.DeltaY;
        if (gamepad2.dpad_up) J2y = J2y+OmniBoi.DeltaY;
        if (gamepad2.dpad_right) J2x = J2x-OmniBoi.DeltaX;
        if (gamepad2.dpad_left) J2x = J2x+OmniBoi.DeltaX;
        */

        /*


        if (CheckRange(J2x,J2y+(OmniBoi.DeltaY*gamepad2.left_stick_y))){
            J2y =+ OmniBoi.DeltaY*gamepad2.left_stick_y;
        }
        if (CheckRange(J2x+(OmniBoi.DeltaX*gamepad2.left_stick_x),J2y)){
            J2x = OmniBoi.DeltaX*gamepad2.left_stick_x;
        }

        */





        /*
        // A test of Inverse Kinematics... Maybe it will work???

        OmniBoi.c2 = (Math.pow(J2x,2) + Math.pow(J2y,2) - Math.pow(OmniBoi.L1,2) - Math.pow(OmniBoi.L2,2)) / (2*OmniBoi.L1*OmniBoi.L2);
        OmniBoi.s2 = Math.sqrt(1-Math.pow(OmniBoi.c2,2));

        //OmniBoi.K1 = OmniBoi.L1+OmniBoi.L2*OmniBoi.c2;
        //OmniBoi.K2 = OmniBoi.L2*OmniBoi.s2;
    // Old code for inverse kinematics...
        //OmniBoi.theta = Math.atan2(J2x,J2y) - Math.atan2(OmniBoi.K1,OmniBoi.K2);
        //OmniBoi.psi = Math.atan2(OmniBoi.c2,OmniBoi.s2);
    // Old code for inverse kinematics...
        //J1Angle = (OmniBoi.theta*180)/Math.PI;
        //J2Angle = (OmniBoi.psi*180)/Math.PI;

        OmniBoi.theta = Math.asin((J2y*(OmniBoi.L1+OmniBoi.L2*OmniBoi.c2)-J2x*OmniBoi.L2*OmniBoi.s2)/(Math.pow(J2x,2)+Math.pow(J2y,2)));
        OmniBoi.psi = Math.acos(OmniBoi.c2);

        OmniBoi.CurrPos1 = OmniBoi.theta/Math.PI*2;
        OmniBoi.CurrPos2 = -OmniBoi.psi/Math.PI*2;

        */


        // Inverse Kinematics V3...




        //angles A;

        //A = Angles(J2x, J2y);
        //OmniBoi.theta = A.A1;
        //OmniBoi.psi = A.A2;
        //OmniBoi.SetShoulder(A.A1);
        //OmniBoi.SetElbow(A.A2);
        //OmniBoi.CurrPos1 = OmniBoi.theta/Math.PI*2;
        //OmniBoi.CurrPos2 = -OmniBoi.psi/Math.PI*2;

        if (gamepad2.x) {  // Begin Inverse Kinematic Control
            stickcontrol = true;
            J2x = 10;
            J2y = 10;
        }
        if (gamepad2.y) { // Deploy Idol
            J2x = OmniBoi.L2 + OmniBoi.L1;
            J2y = 0;
            // Extend Forearm
            // Realease claw
        }
        if (gamepad2.b) {  // Return Home
            stickcontrol = false;
            OmniBoi.SetShoulder(170);
            OmniBoi.SetElbow(10);
        }

        if (gamepad2.left_stick_y > 0.1) {
            double yn = J2y - OmniBoi.DeltaY * Math.abs(gamepad2.left_stick_y);
            if (CheckRange(J2x, yn)) {
                J2y = yn;
            }
        }

        if (gamepad2.left_stick_y < -0.1) {
            double yn = J2y + OmniBoi.DeltaY * Math.abs(gamepad2.left_stick_y);
            if (CheckRange(J2x, yn)) {
                J2y = yn;
            }
        }

        if (gamepad2.left_stick_x > 0.1) {

            double xn = J2x - OmniBoi.DeltaX * Math.abs(gamepad2.left_stick_x);
            if (CheckRange(xn, J2y)) {
                J2x = xn;
            }

        }

        if (gamepad2.left_stick_x < -0.1) {
            double xn = J2x + OmniBoi.DeltaX * Math.abs(gamepad2.left_stick_x);
            if (CheckRange(xn, J2y)) {
                J2x = xn;
            }
        }


        boolean SS,SE;
        if (stickcontrol)
        {
            angles Angle = Angles(J2x,J2y);
            SS = OmniBoi.SetShoulder(Angle.A1);
            SE = OmniBoi.SetElbow(Angle.A2);
        }


        // If this code doesn't work I will actually just leave...







        telemetry.addData("Power: ", OmniBoi.power);

        telemetry.addData("Y Axis: ", gamepad1.left_stick_y);
        telemetry.addData("X axis: ",gamepad1.left_stick_x);

        telemetry.addData("Arm Servo 1 Position: ", OmniBoi.ArmServo1.getPosition());
        telemetry.addData("Arm Servo 2 Position: ", OmniBoi.ArmServo2.getPosition());
        telemetry.addData("Extender Position: ", OmniBoi.Extender.getPosition());
        telemetry.addData("J1 X: ", J1x);
        telemetry.addData("J1 Y: ", J1y);
        telemetry.addData("J2 X: ", J2x);
        telemetry.addData("J2 Y: ", J2y);
        telemetry.addData("Lift Encoder Value: ", OmniBoi.Lift.getCurrentPosition());
        telemetry.addData("Wrist Position:  ", OmniBoi.Wrist.getPosition());
        telemetry.addData("Arm Claw Position:  ", OmniBoi.ArmClaw.getPosition());
        telemetry.addData("Arm Claw Pos Num:  ", OmniBoi.CurrPosArmClaw);
        telemetry.addData("Wrist Pos Num::  ", OmniBoi.CurrPosWrist);

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

    public double PositionToAngle(double Position, double Offset){
        double Angle; // Declare what we are going to return later

        Angle = (Position - Offset); // Even if the Offset is larger, it should be the same position

        if (Angle < 0) Angle = (Angle*-1); // It must be positive if it is larger, of course!

        Angle = (Angle*360); // Values on Servo are 0 to 1, multiply it by 360 and it's a wonderful circle!

        return Angle;
    }

    private double lawOfCosines(double a, double b, double c) {

        return Math.acos((a * a + b * b - c * c) / (2 * a * b));

    }



    private double distance(double x, double y) {

        return Math.sqrt(x * x + y * y);

    }

    class angles {
        public double A1 ;

        public double A2 ;
    }



    private angles Angles(double x, double y)

    {

        double dist = distance(x, y);

        double D1 = Math.atan2(y, x);

        double D2 = lawOfCosines(dist, OmniBoi.L1, OmniBoi.L2);

        double A1 = D1 + D2;

        double A2 = lawOfCosines(OmniBoi.L1, OmniBoi.L2, dist);

        angles A = new angles();

        A.A1 = A1;
        A.A2 = A2;

        return A;

    }


}

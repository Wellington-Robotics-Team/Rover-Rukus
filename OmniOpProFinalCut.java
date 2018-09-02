package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 1/24/2018.
 */

public class OmniOpProFinalCut extends OpMode {
    HardwareKraken robot = new HardwareKraken();
    double x=-5;
    double y=15;
    double deltax = 1.0;
    double deltay = 1.0;
    public double testangle = 0;
    public double flipy = 1;
    public double flipx = 1;
    public double shoulderangle = 178;
    public double elbowangle = 0;
    int count = 0;
    boolean stickcontrol = false;
    boolean bx,by,ba,bb;
    double extposition = 0.95;
    double wristposition = 0.01;
    double wristoffset = 0;

    double clawposition = 0.01;
    double liftclawposition = 0.81;

    double EndTime  = 0;
    double CurTime  = 0;

    @Override
    public void init() {


    }
    public void start(){
        robot.init(hardwareMap);
        // Put the arm in a safe position
        robot.SetShoulder(178);
        robot.SetElbow(10);

        robot.Extender.setPosition(0.8);
        robot.ArmClaw.setPosition(0.5);
        robot.ArmWrist.setPosition(0.1);
        //robot.LiftClaw.setPosition(0.81);
        robot.JewelArm.setPosition(0);
        bx = false;
        by = false;
        bb = false;
        ba = false;
    }

    private boolean CheckRange(double x, double y) {
        boolean t = true;
        if (x < 5)
        {
            telemetry.addLine("Range Check x < 5");
            t = false;
        }
        if (y > 0.9*(robot.Ll+robot.Lu)) {
            telemetry.addLine("Range Check y > 90% Ll+Lu");
            t = false;
        }
        if (x > 0.98 * (robot.Ll+robot.Lu))
        {
            telemetry.addLine("Range Check x > 98% Ll+Lu");
            t = false;
        }
        if (y < -10)
        {
            telemetry.addLine("Range Check y < -10");
            t = false;
        }
        if ((x*x + y*y) > (robot.MagMax)) {
            telemetry.addLine("Range Check x2+y2 > MagMax");
            t = false;
        }
        return t;
    }

    private double lawOfCosines(double a, double b, double c) {
        double val = (a * a + b * b - c * c) / (2 * a * b);
        return Math.acos(val);
    }

    private double distance(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
    class angles
    {
        public double A1;
        public double A2;
    }

    private angles Angles(double x, double y)
    {
        double dist = distance(x, y);
        double D1 = Math.atan2(y, x);
        double D2 = lawOfCosines(dist, robot.Ll, robot.Lu);
        double A1 = D1 + D2;
        double A2 = lawOfCosines(robot.Ll, robot.Lu, dist);

        angles A = new angles();

        A.A1 = A1*(180/Math.PI);
        A.A2 = A2*(180/Math.PI);

        return A;

    }

    @Override
    public void loop() {
        
        robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        //robot.GyroDriver.GyroDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        robot.LiftControl(gamepad1.right_stick_y);

        robot.LiftClawControl(gamepad1.right_trigger, gamepad1.left_trigger);

        if (gamepad1.a){
            liftclawposition = 0.83;
        }

        if (gamepad1.y){
            liftclawposition = 0.85;
        }

        if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper);
            robot.power = robot.power + 0.1;
            if (robot.power >= 1) robot.power = 1;
        }                                               // Power Change code
        if (gamepad1.left_bumper) {
            while (gamepad1.left_bumper) ;
            robot.power = robot.power - 0.1;
            if (robot.power <= 0) robot.power = 0;
        }

        /*
        if (gamepad1.a){
            robot.LiftClaw.setTargetPosition(500);
            robot.LiftClaw.setPower(1);
        }
        if (gamepad1.b){
            robot.LiftClaw.setTargetPosition(220);
            robot.LiftClaw.setPower(1);
        }
        */

        count += 1;
        /*
        if (gamepad2.dpad_left) {
            if (count > 30) {
                shoulderangle -= 5;
                count = 0;
                String values = "sa:" + shoulderangle + " ea:" + elbowangle;
                telemetry.addData("Val", values);
                robot.SetShoulder(shoulderangle);
            }
        }
        if (gamepad2.dpad_right) {
            if (count > 30) {
                shoulderangle += 5;
                count = 0;
                String values = "sa:" + shoulderangle + " ea:" + elbowangle;
                telemetry.addData("Val", values);
                robot.SetShoulder(shoulderangle);
            }
        }
        if (gamepad2.dpad_down) {
            if (count > 30) {
                elbowangle -= 5;
                count = 0;
                String values = "sa:" + shoulderangle + " ea:" + elbowangle;
                telemetry.addData("Val", values);
                robot.SetElbow(elbowangle);
            }
        }
        if (gamepad2.dpad_up) {
            if (count > 30) {
                elbowangle += 5;
                count = 0;
                String values = "sa:" + shoulderangle + " ea:" + elbowangle;
                telemetry.addData("Val", values);
                robot.SetElbow(elbowangle);
            }
        }

        */



        robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);



        if (gamepad2.x) {  // Begin Inverse Kinematic Control
            stickcontrol = true;
            x = 10;
            y = 10;
        }
        if (gamepad2.left_trigger == 1){
            stickcontrol = true;
            extposition = 0.8;
            robot.Lu = 38;
            x = -24;
            y = 42;
            wristposition = 0;
        }
        // Set arm to maximum extension
        if (gamepad2.y) {
            stickcontrol = false;
            extposition = 0.1;
            clawposition = 0;
            robot.SetElbow(180);
            double EndTime = getRuntime() + 0.5;
            double CurTime = getRuntime();
            while ( CurTime<EndTime)
            {
                CurTime=getRuntime();
            }
            robot.SetShoulder(30);
            // Extend Forearm
            // Realease claw
        }
        // Retract Forearm
        if (gamepad2.a)
        {
            extposition=0.8;
            robot.Lu = 38;
            clawposition = 0.1;
        }
        // Return Home
        if (gamepad2.b) {
            stickcontrol = false;
            robot.Extender.setPosition(0.8);
            robot.Lu = 38;
            robot.SetShoulder(179);

            /*
            double EndTime = getRuntime() + 2;
            double CurTime = getRuntime();
            if ( CurTime>EndTime)
            {
                robot.SetElbow(10);
            }
            */

            EndTime = getRuntime() + 2;
            CurTime = getRuntime();
        }
        if (EndTime != 0)
        {
            CurTime = getRuntime();
            if (CurTime > EndTime){
                robot.SetElbow(10);
                EndTime = 0;
            }
        }






        robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);



        // Right Stick Y opens and closes the claw
        if (gamepad2.right_stick_y > 0.1)
        {

            if (clawposition<1)
                clawposition += 0.007;
            else clawposition=0.999;

            telemetry.addData("+Claw:",clawposition);
        }
        if (gamepad2.right_stick_y < -0.1)
        {

            if (clawposition>0)
                clawposition-=0.007;
            else clawposition=0.001;
            telemetry.addData("-Claw:",clawposition);
        }
        // Right Stick X is wrist angle
        if (gamepad2.right_stick_x > 0.1)
        {
            if (wristposition<1)
                wristposition += 0.005;
            else wristposition= 0.999;

        }
        if (gamepad2.right_stick_x < -0.1)
        {
            if (wristposition>0)
                wristposition -= 0.005;
            else wristposition = 0.001;

        }

        // Left Stick x & y moves inverse kinematics location
        if (gamepad2.left_stick_y > 0.1) {
            double yn = y - deltay * Math.abs(gamepad2.left_stick_y);
            if (CheckRange(x, yn)) {
                y = yn;
            }
        } robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.left_stick_y < -0.1) {
            double yn = y + deltay * Math.abs(gamepad2.left_stick_y);
            if (CheckRange(x, yn)) {
                y = yn;
            }
        } robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.left_stick_x > 0.1) {

            double xn = x - deltax * Math.abs(gamepad2.left_stick_x);
            if (CheckRange(xn, y)) {
                x = xn;
            }
        } robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.left_stick_x < -0.1) {
            double xn = x + deltax * Math.abs(gamepad2.left_stick_x);
            if (CheckRange(xn, y)) {
                x = xn;
            }

        } robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.right_bumper)
        {
            extposition = 0.8;
            robot.Lu = 38;

        }
        if (gamepad2.left_bumper)
        {
            extposition = 0.1;
        }

/*
        if (gamepad1.x) {
            if (bx)
                robot.FLM.setPower(0.1);
            else
                robot.FLM.setPower(0);
            bx = !bx;
        }
        if (gamepad1.y) {
            if (by)
                robot.FRM.setPower(0.1);
            else
                robot.FRM.setPower(0.0);
            by = !by;
        }
        if (gamepad1.b) {
            if (bb)
            robot.RLM.setPower(0.1);
            else
                robot.RLM.setPower(0.0);
            bb = !bb;
        }
        if (gamepad1.a) {
            if (ba)
            robot.RRM.setPower(0.1);
            else
                robot.RRM.setPower(0.0);
            ba = !ba;
        }
        if (gamepad1.right_trigger >0.3 && !robot.DTop.getState())
        {
            robot.LiftM.setPower(0.5);
        //    telemetry.addData("Right Trigger:",gamepad1.right_trigger);
        }
        if (gamepad1.right_trigger <=0.3 && gamepad1.left_trigger <=0.3 )
        {
            robot.LiftM.setPower(0);
          //  telemetry.addLine("Both Triggers Neutral");
        }
        if (gamepad1.left_trigger > 0.3 && !robot.DBottom.getState())
        {
            robot.LiftM.setPower(-0.5);

           // telemetry.addLine("Left Trigger");
        }
       // if (robot.DTop.getState() || robot.DBottom.getState())
      //  {
       //     robot.LiftM.setPower(0);
       //     String val = robot.DTop.getState() + "," + robot.DBottom.getState();
      //      telemetry.addData("Dtop/DBottm",val);
      //  }
*/
        boolean SS,SE;
        if (stickcontrol)
        {
            angles A = Angles(x,y);
            robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            SS = robot.SetShoulder(A.A1);

            robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            SE = robot.SetElbow(A.A2);

            robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            robot.Extender.setPosition(extposition);
            // Wrist Correction Code ! ! !
            wristoffset = (180 - A.A1 - A.A2)/180;
            robot.ArmWrist.setPosition(wristposition + wristoffset);
            //robot.LiftClaw.setPosition(liftclawposition);
            robot.ArmClaw.setPosition(clawposition);

            if (count > 100)
            {
                String values = "x:"+x+" y:"+y+" A1:"+A.A1+" A2:"+A.A2+" SS:"+SS+" SE:"+SE;
                telemetry.addData("Val",values);
                count = 0;
            }
        }
        else {
            robot.Extender.setPosition(extposition);
            robot.ArmWrist.setPosition(wristposition);
            robot.ArmClaw.setPosition(clawposition);
            //robot.LiftClaw.setPosition(liftclawposition);
        }

        telemetry.addData("Power: ", robot.power);

        telemetry.addData("Arm Shoulder Position: ", robot.ArmShoulder.getPosition());
        telemetry.addData("Arm Elbow Position: ", robot.ArmElbow.getPosition());
        telemetry.addData("Extender Position: ", robot.Extender.getPosition());
        telemetry.addData("Length of Arm", robot.Lu);
        telemetry.addData("Claw X: ", x);
        telemetry.addData("Claw Y: ", y);
        telemetry.addData("Wrist Position:  ", robot.ArmWrist.getPosition());
        telemetry.addData("Arm Claw Position:  ", robot.ArmClaw.getPosition());
        telemetry.addData("Lift Claw Power: ", robot.LiftClawM.getPower());
        //telemetry.addData("Lift Claw Position: ", robot.LiftClaw.getPosition());

        telemetry.addData("FRM: ", robot.FRM.getPower());
        telemetry.addData("FLM: ", robot.FLM.getPower());
        telemetry.addData("BRM: ", robot.RRM.getPower());
        telemetry.addData("BLM: ", robot.RLM.getPower());

        //telemetry.addData("Calibration Time: ", robot.GyroCaliTime);
        telemetry.addData("Time: ", robot.timer.seconds());

        /*telemetry.addLine()
                .addData("dx", robot.formatRate(robot.rates.xRotationRate))
                .addData("dy", robot.formatRate(robot.rates.yRotationRate))
                .addData("dz", "%s deg/s", robot.formatRate(robot.rates.zRotationRate));
        telemetry.addData("angle", "%s deg", robot.formatFloat(robot.zAngle));
        telemetry.addData("heading", "%3d deg", robot.heading);
        telemetry.addData("integrated Z", "%3d", robot.integratedZ);
        telemetry.addLine()
                .addData("rawX", robot.formatRaw(robot.rawX))
                .addData("rawY", robot.formatRaw(robot.rawY))
                .addData("rawZ", robot.formatRaw(robot.rawZ));
        telemetry.addLine().addData("z offset", robot.zAxisOffset).addData("z coeff", robot.zAxisScalingCoefficient);
        */

        telemetry.update();

    }

}

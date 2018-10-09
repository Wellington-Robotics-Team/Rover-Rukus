package org.firstinspires.ftc.robotcontroller.internal;

/**
 * Created by kruzan on 1/12/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

public class ArmTest extends OpMode {
    HardwareSquid robot = new HardwareSquid();
    double x=0;
    double y=10;
    double deltax = 1.0;
    double deltay = 1.0;
    public double testangle = 0;
    public double flipy = 1;
    public double flipx = 1;
    public double shoulderangle = 0;
    public double elbowangle = 0;
    int count = 0;
    boolean stickcontrol = false;
    boolean bx,by,ba,bb;
    public ArmTest() {
    }

    @Override
    public void init() {

        robot.init(hardwareMap);
        robot.SetShoulder(170);
        robot.SetElbow(10);
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

        count += 1;
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

        if (gamepad2.x) {  // Begin Inverse Kinematic Control
            stickcontrol = true;
            x = 10;
            y = 10;
        }
        if (gamepad2.y) { // Deploy Idol
            x = robot.Lu + robot.Ll;
            y = 0;
            // Extend Forearm
            // Realease claw
        }
        if (gamepad2.b) {  // Return Home
            stickcontrol = false;
            robot.SetShoulder(170);
            robot.SetElbow(10);
        }

        if (gamepad2.left_stick_y > 0.1) {
            double yn = y - deltay * Math.abs(gamepad2.left_stick_y);
            if (CheckRange(x, yn)) {
                y = yn;
            }
        }

        if (gamepad2.left_stick_y < -0.1) {
            double yn = y + deltay * Math.abs(gamepad2.left_stick_y);
            if (CheckRange(x, yn)) {
                y = yn;
            }
        }
        if (gamepad2.left_stick_x > 0.1) {

            double xn = x - deltax * Math.abs(gamepad2.left_stick_x);
            if (CheckRange(xn, y)) {
                x = xn;
            }
        }
        if (gamepad2.left_stick_x < -0.1) {
            double xn = x + deltax * Math.abs(gamepad2.left_stick_x);
            if (CheckRange(xn, y)) {
                x = xn;
            }

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
            telemetry.addData("Right Trigger:",gamepad1.right_trigger);
        }
        if (gamepad1.right_trigger <=0.3 && gamepad1.left_trigger <=0.3 )
        {
            robot.LiftM.setPower(0);
            telemetry.addLine("Both Triggers Neutral");
        }
        if (gamepad1.left_trigger > 0.3 && !robot.DBottom.getState())
        {
            robot.LiftM.setPower(-0.5);

            telemetry.addLine("Left Trigger");
        }
        if (robot.DTop.getState() || robot.DBottom.getState())
        {
            robot.LiftM.setPower(0);
            String val = robot.DTop.getState() + "," + robot.DBottom.getState();
            telemetry.addData("Dtop/DBottm",val);
        }

        */


        boolean SS,SE;
        if (stickcontrol)
        {
            angles A = Angles(x,y);
            SS = robot.SetShoulder(A.A1);
            SE = robot.SetElbow(A.A2);
            if (count > 100)
            {
                String values = "x:"+x+" y:"+y+" A1:"+A.A1+" A2:"+A.A2+" SS:"+SS+" SE:"+SE;
                telemetry.addData("Val",values);
                count = 0;
            }
        }
    }


}

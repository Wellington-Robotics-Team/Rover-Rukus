package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sean on 12/5/2016.
 */

public class Robot{
    Deflector Deflect;
    ShooterClass Shooter;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor LeftM;//left front motor
    public DcMotor  RightM;//right front motor
    public DcMotor Collector;
    public SensorClass Sensors;
    private Lift Lift;
    public double ServoTime = 1.14;
    //HardwareMap hwmap;
    //gitlab code link: gitlab.com/wsierz/FTC2016
    public Robot(HardwareMap hwmap, Telemetry telemetry, boolean deflector, boolean collector, boolean shooter, boolean sensors, boolean lift){
        LeftM = hwmap.dcMotor.get("DriveMotorsLeft");
        RightM = hwmap.dcMotor.get("DriveMotorsRight");
        RightM.setDirection(DcMotorSimple.Direction.REVERSE);

        if (deflector) Deflect = new Deflector(hwmap,telemetry);
        if (collector) Collector = hwmap.dcMotor.get("CMotor");
        if (shooter) Shooter = new ShooterClass(hwmap, telemetry);
        if (sensors) Sensors = new SensorClass(hwmap);
        if (lift) Lift = new Lift(hwmap);
    }
    public Robot(HardwareMap hwmap, Telemetry telemetry){
        //telemetry.addData("Status", "Initialized");
        LeftM = hwmap.dcMotor.get("DriveMotorsLeft");
        RightM = hwmap.dcMotor.get("DriveMotorsRight");
        RightM.setDirection(DcMotorSimple.Direction.REVERSE);
        Collector = hwmap.dcMotor.get("CMotor");
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        Deflect = new Deflector(hwmap,telemetry);
        Shooter = new ShooterClass(hwmap, telemetry);
        Sensors = new SensorClass(hwmap);
        Lift = new Lift(hwmap);
        // Define and Initialize Motors
        //LeftFrontM   = hwmap.dcMotor.get("lfm");
        //RightFrontM  = hwmap.dcMotor.get("lrm");
        //LeftBackM    = hwmap.dcMotor.get("lbm");
        //RightBackM   = hwmap.dcMotor.get("rbm");

        //LeftFrontM.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //RightFrontM.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //LeftBackM.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //RightBackM.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        //LeftFrontM.setPower(0);
        //RightFrontM.setPower(0);
        //LeftBackM.setPower(0);
        //RightBackM.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //LeftFrontM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //RightFrontM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //LeftBackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //RightBackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");

    }


}
class Deflector{
    //motor with encoder has 1,480.59 ticks per 360 degrees;
    public Servo DirectionM;//direction motor, has encoder
    public Servo AngleServo;
    private double ServoTime = 1.14;
    private double MotorTicks = 1120;// amount of ticks per motor rotation, next is ticks per turret rotation
    private double TurretTicks = MotorTicks*3.222973;//ratio between turret rotation ticks and motor ticks
    private double ServoMax = 180;// maximum degrees the thing can actually rotate.
    private Telemetry telemetry;
    private double Servo90 = 1; //not actually 1, just set to that until i test
    final double NormalPower = 0.6;

    public Deflector(HardwareMap hwmap, Telemetry gettelemetry){
        DirectionM = hwmap.servo.get("directionmotor");
        AngleServo = hwmap.servo.get("angleservo");
        this.telemetry = gettelemetry;
        //DirectionM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DirectionM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //public void SetDir(double DegreeAngle){
        /*if (DegreeAngle >= 260){
            telemetry.addData("Error","Would run into motor");
            return;
        }
        if (DegreeAngle < 0 )//&& DegreeAngle >= -100)//right now it should be like this, but if it's ever just given a degree we should change it
        {
            telemetry.addData("Error","Would run into motor");
            return;
        }*/
        /*if (DegreeAngle > 360) DegreeAngle = DegreeAngle-360;//new motor is 1120 ticks for one motor rotation 3609.7(hopefully) for turret
        int TargetPosition = (int)Math.round((DegreeAngle/360*TurretTicks));//1480 is one motor rotation, 4770 is one turret rotation
        DirectionM.setTargetPosition(TargetPosition);
        DirectionM.setPower(NormalPower);
    }
    public void SetDir(double DegreeAngle, double speed){
        if (DegreeAngle > 360) DegreeAngle = DegreeAngle-360;
        int TargetPosition = (int)Math.round((DegreeAngle/360*TurretTicks));//1480 is one motor rotation, 4770 is one turret rotation
        DirectionM.setTargetPosition(TargetPosition);
        DirectionM.setPower(speed);
    }
    public void testDir(int tick, double speed){
        DirectionM.setTargetPosition(tick);
        DirectionM.setPower(speed);
    }*/

    public void SetFlightAngle(double Angle){
        //set the angle the ball will fly at
        //calc the angle needed for the angle
        //
        //set the servo to that angle
    }
    public void SetAngle(double Angle){
        //0 to 90 degrees
        //0 to 1 servo
        //for next practice:
        //try to figure out how to set the angle of the big thing in general.
        if (Angle > 101){
            telemetry.addData("Error", "Angle above 90 degrees!");// if angle is more than
            return;
        }
        Angle = 101-Angle;//actual 90 is 79 degrees
        double AnglePercent = Angle/50;//servomax is the degrees at the servos max rotation.
        // depending on which way the servo goes, b make the below say 1-AnglePercent or just anglepercent
        double ServoAngle = AnglePercent;//servo 1 is 90, servo 0 is 0. if you go over that it will give an error and not do anything.
        AngleServo.setPosition(AnglePercent);
    }
    /*public void InitTwitch(){
        //this sets the turret to straight forward and 90 degrees at the start.
        SetDir(200);//sets turret to point forward at start- it's 20 degrees to 0, and 180 to the other side
        while (DirectionM.isBusy());
        DirectionM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DirectionM.setTargetPosition(0);
        DirectionM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }*/

    /*public void BallPush(){
        DirectionM.setPosition(1);
             try {
                 Thread.sleep((long) ((ServoTime/4) * 1000));
             } catch (InterruptedException ex) {
                 Thread.currentThread().interrupt();
             }
             DirectionM.setPosition(0.495);
    }*/
}
class ShooterClass{
    public DcMotor MotorLeft;
    public DcMotor MotorRight;
    private Telemetry telemetry;
    private int MaxTPS = 2985; //Max ticks per second, get from EncoderTest.java
    //2985 is the safe val. 3009 is the median. See excel worksheet about it.
    public ShooterClass(HardwareMap hwmap, Telemetry gettelemetry) {
        MotorLeft = hwmap.dcMotor.get("ShootMotorLeft");
        MotorRight = hwmap.dcMotor.get("ShootMotorRight");
        this.telemetry = gettelemetry;
        MotorLeft.setPower(0);//stop motors
        MotorRight.setPower(0);
        MotorLeft.setMaxSpeed(MaxTPS);//these need to be the same speed for the flywheels to run at the same speed
        MotorRight.setMaxSpeed(MaxTPS);
        MotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        /*
        MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
    }
}
class SensorClass{
    public ColorSensor LSense;//line sensor
    public ColorSensor BSense;//beacon sensor
    public SensorClass(HardwareMap hwmap){
        LSense = hwmap.colorSensor.get("Line Sensor");
        BSense = hwmap.colorSensor.get("Beacon Sensor");
    }
    public boolean Line(){
        double LineThreshold = 80;
        if (LSense.red()> LineThreshold)return true;
        else return false;
    }

}

class Lift{
    private DcMotor LiftMotor;
    private Servo Release;


    Lift(HardwareMap hwmap){
        LiftMotor = hwmap.dcMotor.get("Ball Lift");
        Release = hwmap.servo.get("Release");
    }

    public void Release(){
        double ServoTime = 1.14;
            Release.setPosition(1);
            try {
                Thread.sleep((long) ((ServoTime/4) * 1000));
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            Release.setPosition(0.495);
    }

    public void Raise(float LiftPower){
        if (LiftPower >= 0) {
            LiftMotor.setPower(LiftPower);
        }
        else LiftMotor.setPower(0);
    }
}


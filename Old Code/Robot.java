package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by David on 11/2/2017.
 */

public class Robot {

    HardwareMap hwMap = null;   // Making HardwareMap...
    DcMotor FRM = null;
    DcMotor FLM = null;
    DcMotor BRM = null;      // Making Motors...
    DcMotor BLM = null;
    public double power;
    int sleepErrors = 0;
    ElapsedTime timer;
    Recorder recorder;

    public Robot(){
        // Honestly what would I even do in this constructor anyway???
        hwMap = null;   // Making HardwareMap...
        FRM = null;
        FLM = null;
        BRM = null;      // Making Motors...
        BLM = null;
        power = 0.5;
        sleepErrors = 0;
        timer = new ElapsedTime();
    }


    public void Initialize(){

        FLM = hwMap.dcMotor.get("FLM");
        FRM = hwMap.dcMotor.get("FRM");
        BRM = hwMap.dcMotor.get("BRM");   // Instantiation of Motors on Robot
        BLM = hwMap.dcMotor.get("BLM");

        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        // This Forces the Motors to run with Encoders...
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        power = 0.5;
    }

    public void Initialize(DcMotor FLM, DcMotor FRM, DcMotor BRM, DcMotor BLM, String FLMname, String FRMname, String BRMname, String BLMname){

        FLM = hwMap.dcMotor.get(FLMname);
        FRM = hwMap.dcMotor.get(FRMname);
        BRM = hwMap.dcMotor.get(BRMname);   // Instantiation of Motors on Robot
        BLM = hwMap.dcMotor.get(BLMname);

        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        // This Forces the Motors to run with Encoders...
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        power = 0.5;

    }

    public void Move(double FLM, double FRM, double BRM, double BLM){
        this.FRM.setPower(FRM);
        this.BRM.setPower(BRM);
        this.FLM.setPower(FLM);
        this.BLM.setPower(BLM);
    }

    public void Sleep(int time){
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            ++sleepErrors;
        }
    }

    public void TankDrive(double RightSpeed, double LeftSpeed){
        Move(LeftSpeed , RightSpeed, RightSpeed, LeftSpeed);
    }

    public void POVDrive(double drive, double turn){
        double RightPower;
        double LeftPower;

        LeftPower = Range.clip(-drive + turn, -1.0, 1.0) ;
        RightPower = Range.clip(-drive - turn, -1.0, 1.0) ;

        Move(LeftPower, -RightPower, -RightPower, LeftPower);
    }

    public void ButtonTurn(boolean RightButton, boolean LeftButton){

        while (RightButton) {
            Move(power, -power, -power, power);
        }

        while (LeftButton) {
            Move(-power, power, power, -power);
        }
    }

    public void TriggerTurn(double RT, double LT){

        while (RT > 0){
            Move(power*RT, -power*RT, -power*RT, power*RT);
        }

        while (LT > 0){
            Move(-power * LT, power * LT, power * LT, -power * LT);
        }
    }

    public void PowerChange(boolean UpButton, boolean DownButton){

        if (UpButton) {
            while (UpButton) ;
            power = power + 0.1;
            if (power >= 1) power = 1;
        }
        if (DownButton) {
            while (DownButton) ;
            power = power - 0.1;
            if (power <= 0) power = 0;
        }
    }

    public void PowerChange(double UpDouble, double DownDouble){
        if (UpDouble == 1) {
            while (UpDouble == 1) ;
            power = power + 0.1;
            if (power >= 1) power = 1;
        }
        if (DownDouble == 1) {
            while (DownDouble == 1) ;
            power = power - 0.1;
            if (power <= 0) power = 0;
        }
    }

    public void DpadDrive(boolean Up, boolean Down, boolean Right, boolean Left){

        while (Up) {
            TankDrive(power , power);
        }

        while (Down) {
            TankDrive(-power, -power);
        }

        ButtonTurn(Right, Left);
    }


    /*
     *   Class "Robot" is a basic four-wheel drive platform and can perform only with regular wheels on it...
     *
     *   Class "OmniBot" is the daughter class of "Robot" and can instantiate 2 Servos and 1 Integrating Gyroscope as well as perform Mecanum Drive...
     */


}

class OmniBot extends Robot{

    double ServoTime = 1.14;
    Servo ArmServo1 = null;
    Servo ArmServo2 = null;
    Servo ArmClaw = null;
    Servo Wrist = null;
    Servo JewelArm = null;
    DcMotor LiftClaw = null;
    Servo Extender = null;

    TouchSensor LiftButton;
    ColorSensor cc;

    double L1;
    double L2;
    double c2;
    double s2;
    double theta;
    double psi;
    double K1;
    double K2;
    double CurrPos1;
    double CurrPos2;
    double CurrPosWrist;
    double CurrPosArmClaw;
    double CurrPosLiftClaw;
    double CurrPosExtender;
    double servoDelta;
    double DeltaX;
    double DeltaY;
    double MagMax;
    DcMotor Lift;

    double GyroCaliTime;           // These are Values used by the Motors and the Gyro...
    int rawX;
    int rawY;
    int rawZ;
    int heading;
    int integratedZ;
    AngularVelocity rates;
    float zAngle;
    int zAxisOffset;
    int zAxisScalingCoefficient;

    // Making Gyro...
    private ModernRoboticsI2cGyro gyro;

    public OmniBot(){
        hwMap = null;   // Making HardwareMap...
        FRM = null;
        FLM = null;
        BRM = null;      // Making Motors...
        BLM = null;
        Lift = null;
        power = 0.5;
        sleepErrors = 0;
        timer = new ElapsedTime();
        // Aaaaannnnnd yet another constructor I have no idea what to do with!!! Yay!!!
        ServoTime = 1.14;
        ArmServo1 = null;
        ArmServo2 = null;
        Wrist = null;
        ArmClaw = null;
        LiftClaw = null;
        Extender = null;
        CurrPos1 = 0;
        CurrPos2 = 0;
        CurrPosWrist = 0;
        CurrPosArmClaw = 0;
        CurrPosLiftClaw = 0;
        CurrPosExtender = 0;
        servoDelta = 0.001;
        DeltaX = 1.0;
        DeltaY = 1.0;
        MagMax = Math.pow(L1+L2,2);
        GyroCaliTime = 0;           // These are Values used by the Motors and the Gyro...
        rawX = 0;
        rawY = 0;
        rawZ = 0;
        heading = 0;
        integratedZ = 0;
        rates = null;
        zAngle = 0;
        zAxisOffset = 0;
        zAxisScalingCoefficient = 0;
    }

    public void InitializeOmniWheels(){

        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);            // Reverse Direction For Mecanum Wheels...
    }


    public void InitializeArmServos(){

        ArmServo1 = hwMap.servo.get("ArmServo1"); // Instantiating Servos
        ArmServo2 = hwMap.servo.get("ArmServo2");
    }

    public void InitializeArmServos(double Length1, double Length2){

        ArmServo1 = hwMap.servo.get("ArmServo1"); // Instantiating Servos
        ArmServo2 = hwMap.servo.get("ArmServo2");

        L1 = Length1;
        L2 = Length2;
    }

    public void InitializeArmServos(double Length1, double Length2, double RestPos1, double RestPos2){
        ArmServo1 = hwMap.servo.get("ArmServo1"); // Instantiating Servos
        ArmServo2 = hwMap.servo.get("ArmServo2");

        L1 = Length1;
        L2 = Length2;

        ArmServo1.setPosition(RestPos1);
        ArmServo2.setPosition(RestPos2);
    }

    public void InitializeArmClaw(){
        ArmClaw = hwMap.servo.get("ArmClaw");
        Wrist = hwMap.servo.get("Wrist");
    }

    public void ReverseServo(Servo servo){
        if (servo.getDirection() == Servo.Direction.FORWARD) {
            servo.setDirection(Servo.Direction.REVERSE);
        }

        if (servo.getDirection() == Servo.Direction.REVERSE){
            servo.setDirection(Servo.Direction.FORWARD);
        }
    }

    public void ServoButtonMove(double Position, boolean Back, boolean Forward){
        if (Back) {
            Position = Position - servoDelta;
        }

        if (Forward) {
            Position = Position + servoDelta;
        }
    }

    public void ArmControl(double ArmX, double ArmY, double wrist, double ClawUp, double ClawDown) {
        CurrPos1 += servoDelta * ArmX * -1;
        CurrPos2 += servoDelta * ArmY;
        CurrPosWrist += servoDelta * wrist;
        CurrPosArmClaw += servoDelta * (ClawUp-ClawDown);
    }

    public void SetServo1(double Rad){
        double Pos;

        Pos = Rad/Math.PI;

        ArmServo1.setPosition(Pos);
    }

    public void SetServo2(double Rad){

        double Pos;

        Rad = Math.PI - Rad;

        Pos = Rad/Math.PI;

        ArmServo2.setPosition(Pos);
    }

    public boolean SetShoulder(double deg)
    {
        // Set Servo Position based on 180 degree allowed range
        double pos = (deg)/180;
        boolean error = true;
        if (pos >= 0 && pos<= 1.0) {
            ArmServo1.setPosition((deg) / 180);
            error = false;
        }
        return error;
    }

    public boolean SetElbow(double deg)
    {
        // Set Servo Position based on 180 degree allowed range
        double pos = (deg)/180;
        boolean error = true;
        if (pos >= 0 && pos<= 1.0) {
            ArmServo2.setPosition(((180-deg)) / 180);
            error = false;
        }
        return error;
    }


    public double PositionToAngle(double Position){
        double Angle; // Declare what we are going to return later

        Angle = Position*360; // Values on Servo are 0 to 1, multiply it by 360 and it's a wonderful circle!

        return Angle;
    }

    public double AngleToPosition(double Angle){
        double Position;

        Position = Angle/360;

        return Position;
    }

    public void InitializeGyro(){

        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");   // Instantiating Gyro...

        timer.reset();
        // Gyro Calibration...
        gyro.calibrate();

        GyroCaliTime = timer.seconds();   // This is so we can keep track of the time it takes for Gyro Calibration to complete...
    }

    public void GyroUpdate(){

        rawX = gyro.rawX();
        rawY = gyro.rawY();
        rawZ = gyro.rawZ();
        heading = gyro.getHeading();
        integratedZ = gyro.getIntegratedZValue();

        rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        zAxisOffset = gyro.getZAxisOffset();
        zAxisScalingCoefficient = gyro.getZAxisScalingCoefficient();

    }

    public int GetSignedHeading(){
        int CurrentHeading = gyro.getHeading();

        if (CurrentHeading > 180){
            CurrentHeading -= 360;
        }

        return CurrentHeading;
    }

    public void GyroTurn(int TargetHeading){
        int CurrentHeading = GetSignedHeading();

        while(CurrentHeading < TargetHeading){
            FLM.setPower(power);
            BLM.setPower(power);   // This makes the bot turn right...
            FRM.setPower(-power);
            BRM.setPower(-power);

            CurrentHeading = GetSignedHeading();
        }

    }

    public void InitializeLift(){
        Lift = hwMap.dcMotor.get("Lift");
        //LiftClaw = hwMap.dcMotor.get("LiftClaw");
        LiftButton = hwMap.touchSensor.get("Touch");

        //Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void LiftControl(double up, double down){
        if (!LiftButton.isPressed()){

            Lift.setPower((up-down)*0.25);

        } else Lift.setPower(up*0.25);


    }

    public void LiftClawControl(boolean OpenButton, boolean CloseButton, double OpenPos, double ClosePos){
        if (OpenButton) {
            CurrPosLiftClaw = OpenPos;
        }

        if (CloseButton){
            CurrPosLiftClaw = ClosePos;
        }

    }

    public void LiftClawControl(boolean OpenButton, boolean CloseButton){
        if (OpenButton) {
            CurrPosLiftClaw += servoDelta;
        }

        if (CloseButton){
            CurrPosLiftClaw -= servoDelta;
        }

    }

    public void OmniDrive(double y, double x, double rotation){

        FRM.setPower((y + x+rotation) * power);
        FLM.setPower((y - x-rotation) * power);
        BRM.setPower((y - x+rotation) * power);     // This is the code I made to make the bot go in all directions...
        BLM.setPower((y + x-rotation) * power);

    }

    public void DpadDriveOmni(boolean Up, boolean Down, boolean Right, boolean Left){
        while (Up) {
            TankDrive(power, power);
        }

        while (Down) {
            TankDrive(-power, -power);
        }

        while (Right) {
            Move(power,-power,power,-power);
        }

        while (Left) {
            Move(-power,power,-power,power);
        }
    }

    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }

}



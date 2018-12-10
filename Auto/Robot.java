package org.firstinspires.ftc.teamcode;

import android.text.style.TtsSpan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    //Just used as a marker/flag
    static public final int LEFTMOTORS = 0;
    static public final int RIGHTMOTORS = 1;

    public Robot(HardwareMap hwMap){
        // Honestly what would I even do in this constructor anyway??? -David
        this.hwMap = hwMap;   // Making HardwareMap...
        FRM = null;
        FLM = null;
        BRM = null;      // Making Motors...
        BLM = null;
        power = 0.5;
        sleepErrors = 0;
        timer = new ElapsedTime();
    }


    public void InitializeMotors(){

        FLM = hwMap.dcMotor.get("FLM");
        FRM = hwMap.dcMotor.get("FRM");
        BRM = hwMap.dcMotor.get("BRM");   // Instantiation of Motors on Robot
        BLM = hwMap.dcMotor.get("BLM");

        FRM.setDirection(DcMotorSimple.Direction.FORWARD);
        BRM.setDirection(DcMotorSimple.Direction.FORWARD);
        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        // This Forces the Motors to run with Encoders...
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           */
        power = 0.2;
    }
    public void enableEncoders(){
        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        // This Forces the Motors to run with Encoders...
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void disableEncoders(){
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        // This Forces the Motors to run with Encoders...
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void enableBraking(){
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // This Forces the Motors to run with Encoders...
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void InitializeMotorsRUE(DcMotor FLM, DcMotor FRM, DcMotor BRM, DcMotor BLM, String FLMname, String FRMname, String BRMname, String BLMname){

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
    public void TargetDist(int SIDE, int ticks){
        //140 ticks per rev
        if (SIDE == RIGHTMOTORS){
            FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FRM.setTargetPosition(ticks);
            BRM.setTargetPosition(ticks);
        }
        if (SIDE == LEFTMOTORS){
            FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FLM.setTargetPosition(ticks);
            BLM.setTargetPosition(ticks);
        }

    }
    public void linearTankDrive(double RightSpeed, double LeftSpeed, LinearOpMode LO, double Time){
        double InitialTime = LO.getRuntime();
        while(LO.opModeIsActive()&& LO.getRuntime()<InitialTime+Time){
            Move(LeftSpeed , RightSpeed, RightSpeed, LeftSpeed);
        }
    }
    public void linearTankDrive(double RightDist, double LeftDist, double power, double WheelRadius, LinearOpMode LO){
        double InitialTime = LO.getRuntime();
        //set both motors to go 2 revolutions
        int tworevs = 280;
        double pow = 0.7;
        TargetDist(LEFTMOTORS,tworevs);
        TargetDist(RIGHTMOTORS,tworevs);
        while(LO.opModeIsActive()&& FLM.isBusy()){
            Move(pow,pow,pow,pow);
        }
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


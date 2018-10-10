package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sean on 2/8/2018.
 */

public class gyrothing {
    ModernRoboticsI2cGyro gyro = null;
    double wantedposition = 0;
    double y = 0;
    double x = 0;
    double spin = 0;
    double power = 0.5;
    HardwareKraken robot = null;

    public gyrothing(){
        //don't do anything
    }

    public void init(HardwareMap hardwareMap, HardwareKraken robot){
        this.robot = robot;
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");   // Instantiating Gyro...
        // Gyro Calibration...
        gyro.calibrate();
        while(gyro.isCalibrating()){
            /*
            telemetry.addData("gyro"," calibrating");
            telemetry.update();
            */
        }
        /*
        telemetry.addData("gyro"," done");
        telemetry.update();
        */
    }
    public void GyroDrive(double sticky, double stickx, double rstickx){
        y = sticky;
        x = stickx;

        x = x*1.5;//scales for strafing
        if (rstickx ==0){
            //robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,GetSpeed()/5);
            spin = GetTurnSpeed()/2;
            //honestly i think this is just omnidrive written out
            robot.FRM.setPower((y + x+spin) * power);
            robot.FLM.setPower((y - x-spin) * power);
            robot.RRM.setPower((y - x+spin) * power);
            robot.RLM.setPower((y + x-spin) * power);
        }
        else {
            //robot.OmniDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,rstickx);
            spin = rstickx/2;
            //honestly i think this is just omnidrive written out
            robot.FRM.setPower((y + x+spin) * power);
            robot.FLM.setPower((y - x-spin) * power);
            robot.RRM.setPower((y - x+spin) * power);     // This is the code I made to make the bot go in all directions...
            robot.RLM.setPower((y + x-spin) * power);
            wantedposition = GetSignedHeading();
        }

    }
    public int GetSignedHeading(){
        int CurrentHeading = this.gyro.getHeading();
        if (CurrentHeading > 180){
            CurrentHeading = CurrentHeading - 360;
        }

        return CurrentHeading;
    }
    public double GetTurnSpeed(){
        double positiondif = wantedposition-GetSignedHeading();
        double speed = -0.003*positiondif*positiondif*positiondif;
        if(positiondif<5 &&positiondif>-5) positiondif = 0;
        return speed;
    }
}


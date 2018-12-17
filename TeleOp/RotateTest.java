package org.firstinspires.ftc.robotcontroller.internal;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

//Created by Gabo on 12/17/18

/**
 * USED TO GET ANGLES FOR TURNING
 * HOLD X TO TURN LEFT
 * HOLD B TO TURN RIGHT
 * PRESS A TO RESET THE ANGLE READING
 */
public class RotateTest extends OpMode {

    private BNO055IMU imu; //declare imu
    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    private double globalAngle; //the number of degrees the robot has turned

    private double minPower = 0.25; //least amount of power the robot can have

    private DcMotor BLM;
    private DcMotor BRM;
    private DcMotor FRM;
    private DcMotor FLM;

    private double power = 0.5; //declares the power
    private double oldPower; //declares oldPower
    private boolean fullPower = false; //fullPower is set to false at first
    private boolean noPower = false; //no power is false

    private double serverPosition = 0.6;

    public void init() //runs when you press init on phone
    {
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets the imu

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        imu.initialize(parameters); //initalizes the imu

        HardwareMap hwMap = hardwareMap; //sets the hardware map to hwMap

        BLM = hwMap.dcMotor.get("BLM"); //gets the motors
        BRM = hwMap.dcMotor.get("BRM");
        FRM = hwMap.dcMotor.get("FRM");
        FLM = hwMap.dcMotor.get("FLM");

        FLM.setDirection(DcMotor.Direction.REVERSE);
        BLM.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() //runs after you press play
    {
        if (gamepad1.a)
        {
            while (gamepad1.a) {}
            resetAngle();
        }
        if (gamepad1.b)
        {
            move(minPower, -minPower);
        }
        if (gamepad1.x)
        {
            move(-minPower, minPower);
        }
        telemetry.addData("Current degrees", getAngle());
        telemetry.update();
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //sets lastAngles to current angles

        globalAngle = 0; //global angle is set to 0
    }
    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //gets the angle

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle; //deltaAngle is the first angle - the lastangle it got

        if (deltaAngle < -180) //switches it to use 0 to 360
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle; //adds the deltaAngle to the globalAngle

        lastAngles = angles; //lastAngle is the anlges

        return globalAngle; //returns the amount turned
    }
    
    public void move(double leftPower, double rightPower) //move function for ez use
    {
        BLM.setPower(-leftPower); //left motors are leftPower
        FLM.setPower(-leftPower);
        FRM.setPower(-rightPower); //right motors are rightPower
        BRM.setPower(-rightPower);
    }
}

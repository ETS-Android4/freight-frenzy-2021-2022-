package org.firstinspires.ftc.team6168;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Utilities {
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    ModernRoboticsI2cGyro gyro;
    //28 * 20 / (2ppi * 4.125)
    Double width = 16.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    double amountError = 2;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable

    double DRIVE_SPEED = 0.2;
    double TURN_SPEED = 0.2;

    Double conversion = cpi * bias;
    Boolean exit = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public static void strafeToPosition(double inches, double speed) {
        DcMotor frontleft;
        DcMotor frontright;
        DcMotor backleft;
        DcMotor backright;

        ModernRoboticsI2cGyro gyro;
        //28 * 20 / (2ppi * 4.125)
        Double width = 16.0; //inches
        Integer cpr = 28; //counts per rotation
        Integer gearratio = 40;
        Double diameter = 4.125;
        Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
        Double bias = 0.8;//default 0.8
        Double meccyBias = 0.9;//change to adjust only strafing movement
        double amountError = 2;

        double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
        double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
        double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable

        double DRIVE_SPEED = 0.2;
        double TURN_SPEED = 0.2;

        Double conversion = cpi * bias;
        Boolean exit = false;

        BNO055IMU imu;
        Orientation angles;
        Acceleration gravity;

        int move = (int) (Math.round(inches * cpi * meccyBias));

        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);

        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
}

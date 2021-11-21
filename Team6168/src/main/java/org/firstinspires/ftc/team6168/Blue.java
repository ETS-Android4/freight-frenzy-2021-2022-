package org.firstinspires.ftc.team6168;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="BlueAuto")
public class Blue extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    public DcMotor BlMotor;
    public DcMotor FlMotor;
    public DcMotor BrMotor;
    public DcMotor FrMotor;


    //28 * 20 / (2ppi * 4.125)
    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement

    Double conversion = cpi * bias;
    Boolean exit = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


    public void runOpMode() {

        telemetry.clearAll();
        telemetry.addData("Status", "Auto Initialization In Progress");
        telemetry.update();


        BlMotor = hardwareMap.get(DcMotor.class, "Backleft");
        FlMotor = hardwareMap.get(DcMotor.class, "Frontleft");
        BrMotor = hardwareMap.get(DcMotor.class, "Backright");
        FrMotor = hardwareMap.get(DcMotor.class, "Frontright");

        BlMotor.setDirection(DcMotor.Direction.FORWARD);
        FlMotor.setDirection(DcMotor.Direction.FORWARD);
        BrMotor.setDirection(DcMotor.Direction.REVERSE);
        FrMotor.setDirection(DcMotor.Direction.REVERSE);



        BlMotor.setPower(0);
        FlMotor.setPower(0);
        BrMotor.setPower(0);
        FrMotor.setPower(0);

        telemetry.clearAll();
        telemetry.addData("Status", "Auto Initialization complete");
        telemetry.update();


        waitForStart();
    }


}
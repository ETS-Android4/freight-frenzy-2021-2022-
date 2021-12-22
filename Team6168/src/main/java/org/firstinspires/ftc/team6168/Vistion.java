package org.firstinspires.ftc.team6168;

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
import org.firstinspires.ftc.team6168.RingDetector;

@Autonomous(name="Vistion")
public class Vistion extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //
    public DcMotor frontleft;
    public DcMotor backleft;
    public DcMotor frontright;
    public DcMotor backright;
    public DcMotor WobbleFlipper;
    public DcMotor Intake;
    public DcMotor LeftShooter;
    public DcMotor RightShooter;


    public Servo WobbleGrabber;
    public CRServo ConveyorBelt;
    public Servo IntakeFlipper;

    //28 * 20 / (2ppi * 4.125)
    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //



    public void runOpMode() {

        telemetry.clearAll();
        telemetry.addData("Status", "Auto Initialization In Progress");
        telemetry.update();
        //
//        initGyro();
        //
        frontleft = hardwareMap.dcMotor.get("Frontleft");
        frontright = hardwareMap.dcMotor.get("Frontright");
        backleft = hardwareMap.dcMotor.get("Backleft");
        backright = hardwareMap.dcMotor.get("Backright");
   

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        RingDetector detector = new RingDetector(this);

        //
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

        telemetry.clearAll();
        telemetry.addData("Status", "Auto Initialization complete");
        telemetry.update();

        //WobbleGrabber.setPosition(1);


        waitForStart();

        int rings = detector.getDecision();
        if (rings == 4) {
//
//            strafeToPosition(90, 0.8);
//            //
//            moveToPosition(6.8, 0.8);
//
//            sleep(1000);
//
//            WobbleMove(-60, 0.8);
//
//            sleep(3000);
//
//            WobbleUnGrab();
//
//            sleep(1000);
//            //
//            moveToPosition(-17, 0.8);
//            //
//            strafeToPosition(-45, 0.8);
////            turns robot
//            encoderDrive(0.8,-12,12,5000);
////
//            Shooter();
        }

        if (rings == 0) {

//            strafeToPosition(35, 0.8);
//            //
//            moveToPosition(6.8, 0.8);
//
//            sleep(1000);
//
//            WobbleMove(-60, 0.8);
//
//            sleep(3000);
//
//            WobbleUnGrab();
//
//            sleep(1000);//
//            moveToPosition(-17, 0.8);
//            //
//            strafeToPosition(-5, 0.8);
////            turns robot
//            encoderDrive(0.8,-12,12,5000);
////
//            Shooter();

        }


        if (rings == 1) {

//
//            strafeToPosition(55, 0.8);
//
//            moveToPosition(-10,0.5);
//            //
//            sleep(1000);
//
//            WobbleMove(-60, 0.8);
//
//            sleep(3000);
//
//            WobbleUnGrab();
//
//            sleep(1000);
//
//            //
//            strafeToPosition(-17.5, 0.8);
////            turns robot
//            encoderDrive(0.8,-12,12,5000);
////
//            Shooter();
//
//        }
            //
        }
    }
}

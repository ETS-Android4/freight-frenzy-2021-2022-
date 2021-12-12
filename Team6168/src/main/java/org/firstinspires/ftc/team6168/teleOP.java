package org.firstinspires.ftc.team6168;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;





@TeleOp(name="TeleOP")
public class teleOP extends OpMode {


    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor backleft;
    public DcMotor frontleft;
    public DcMotor backright;
    public DcMotor frontright;
    public DcMotor SpinnerMotor;
    public DcMotor InandOut;
    public DcMotor UpandDown;

    public Servo Grabber;


    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        //Hardware map
        backleft = hardwareMap.get(DcMotor.class, "Backleft");
        frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        backright = hardwareMap.get(DcMotor.class, "Backright");
        frontright = hardwareMap.get(DcMotor.class, "Frontright");
        SpinnerMotor = hardwareMap.get(DcMotor.class, "Car");
        InandOut = hardwareMap.get(DcMotor.class, "InandOut");
        UpandDown = hardwareMap.get(DcMotor.class, "UpandDown");
        Grabber = hardwareMap.get(Servo.class, "Grabber");


        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        SpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
        InandOut.setDirection(DcMotor.Direction.FORWARD);
        UpandDown.setDirection(DcMotor.Direction.FORWARD);
        Grabber.setDirection(Servo.Direction.FORWARD);


        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        InandOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        UpandDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        InandOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        UpandDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InandOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        UpandDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backleft.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        frontright.setPower(0);
        SpinnerMotor.setPower(0);
        InandOut.setPower(0);
        UpandDown.setPower(0);
        Grabber.setPosition(1);





        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }


    @Override
    public void loop() {

        //Drive Train Code
        double stopBuffer = 0;


        double speedMode = 1;
        if (gamepad1.right_bumper == true) {
            speedMode = .4;
        } else if (gamepad1.b == true) {
            speedMode = 1;
        }

        double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
        double right = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
        double turn = -speedMode * Math.pow(gamepad1.right_stick_x,3);

        double leftFrontPower = forward + right + turn;
        double leftBackPower = forward - right + turn;
        double rightFrontPower = forward - right - turn;
        double rightBackPower = forward + right - turn;
        double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        boolean needToScale = false;
        for (double power : powers){
            if(Math.abs(power) > 1){
                needToScale = true;
                break;
            }
        }
        if (needToScale){
            double greatest = 0;
            for (double power : powers){
                if (Math.abs(power) > greatest){
                    greatest = Math.abs(power);
                }
            }
            leftFrontPower /= greatest;
            leftBackPower /= greatest;
            rightFrontPower /= greatest;
            rightBackPower /= greatest;
        }

        boolean stop = true;
        for (double power : powers){
            if (Math.abs(power) > stopBuffer){
                stop = false;
                break;
            }
        }
        if (stop){
            leftFrontPower = 0;
            leftBackPower = 0;
            rightFrontPower = 0;
            rightBackPower = 0;
        }

        frontleft.setPower(leftFrontPower);
        //base().getTelemetry().addData("Setting frontLeft power to " , leftFrontPower);
        backleft.setPower(leftBackPower);
        //base().getTelemetry().addData("Setting backLeft power to " , leftBackPower);

        frontright.setPower(rightFrontPower);
        //base().getTelemetry().addData("Setting frontRight power to " , rightFrontPower);

        backright.setPower(rightBackPower);
        //base().getTelemetry().addData("Setting rightBack power to " , rightBackPower);


        //Carosel Spinner Code
        if (gamepad2.left_bumper) {
            SpinnerMotor.setPower(1);
        }else {
            SpinnerMotor.setPower(0);
        }

        if (gamepad2.right_bumper) {
            SpinnerMotor.setPower(-1);
        }else {
            SpinnerMotor.setPower(0);
        }


        //Grabber Code
        if (gamepad2.dpad_left) {
            Grabber.setPosition(1);
        } else if (gamepad2.dpad_right) {
            Grabber.setPosition(.5);
        } else {
            //Grabber.setPosition(.5);
        }

        //InandOut Code
        if (gamepad2.right_stick_x >= 0.3) {
            InandOut.setPower(0.9);
        } else if (gamepad2.right_stick_x <= -0.3) {
            InandOut.setPower(-0.9);
        }else{
            InandOut.setPower(0);
        }

        //UpandDown
        if (gamepad2.left_stick_y >= 0.3) {
            UpandDown.setPower(0.9);
        } else if (gamepad2.left_stick_y <= -0.3) {
            UpandDown.setPower(-0.9);
        }else{
            UpandDown.setPower(0);
        }


        }


    }

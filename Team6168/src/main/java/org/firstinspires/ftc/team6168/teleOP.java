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
        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_x, 4) + Math.pow(-gamepad1.left_stick_y, 4), 0.5);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rotation = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle - (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle - (Math.PI / 4.0)));

        backleft.setPower(secondaryDiagonalSpeed - rotation);
        frontleft.setPower(secondaryDiagonalSpeed + rotation);
        backright.setPower(primaryDiagonalSpeed - rotation);
        frontright.setPower(primaryDiagonalSpeed + rotation);


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
            Grabber.setPosition(-1);
        } else {
            Grabber.setPosition(0);
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

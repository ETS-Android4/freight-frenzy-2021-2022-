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
    public DcMotor FlMotor;
    public DcMotor BlMotor;
    public DcMotor FrMotor;
    public DcMotor BrMotor;
    public DcMotor SpinnerMotor;
    public DcMotor InandOut;
    public DcMotor UpandDown;

    public Servo Grabber;

//CHEESE BORGIUR

    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        //Hardware map
        BlMotor = hardwareMap.get(DcMotor.class, "Backleft");
        FlMotor = hardwareMap.get(DcMotor.class, "Frontleft");
        BrMotor = hardwareMap.get(DcMotor.class, "Backright");
        FrMotor = hardwareMap.get(DcMotor.class, "Frontright");
        SpinnerMotor = hardwareMap.get(DcMotor.class, "Car");
        InandOut = hardwareMap.get(DcMotor.class, "InandOut");
        UpandDown = hardwareMap.get(DcMotor.class, "UpandDown");
        Grabber = hardwareMap.get(Servo.class, "Grabber");


        BlMotor.setDirection(DcMotor.Direction.FORWARD);
        FlMotor.setDirection(DcMotor.Direction.FORWARD);
        BrMotor.setDirection(DcMotor.Direction.REVERSE);
        FrMotor.setDirection(DcMotor.Direction.REVERSE);
        SpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
        InandOut.setDirection(DcMotor.Direction.FORWARD);
        UpandDown.setDirection(DcMotor.Direction.FORWARD);
        Grabber.setDirection(Servo.Direction.FORWARD);


        BlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SpinnerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        InandOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        UpandDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SpinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        InandOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        UpandDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InandOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        UpandDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BlMotor.setPower(0);
        FlMotor.setPower(0);
        BrMotor.setPower(0);
        FrMotor.setPower(0);
        SpinnerMotor.setPower(0);
        InandOut.setPower(0);
        UpandDown.setPower(0);
        Grabber.setPosition(0);


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

        BlMotor.setPower(secondaryDiagonalSpeed - rotation);
        FrMotor.setPower(secondaryDiagonalSpeed + rotation);
        FlMotor.setPower(primaryDiagonalSpeed - rotation);
        BrMotor.setPower(primaryDiagonalSpeed + rotation);


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
            Grabber.setPosition(0.9);
        } else if (gamepad2.dpad_left) {
            Grabber.setPosition(0.5);
        }

        if (gamepad2.dpad_right) {
            Grabber.setPosition(0.1);
        } else if (gamepad2.dpad_right){
            Grabber.setPosition(0.5);
        }

        //InandOut Code
        if (gamepad2.right_stick_y >= 0.3) {
            InandOut.setPower(0.9);
        } else if (gamepad2.right_stick_y <= -0.3) {
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

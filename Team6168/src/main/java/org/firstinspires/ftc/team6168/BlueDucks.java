//package org.firstinspires.ftc.team6168;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//@Autonomous(name="AutoBlueLeft")
//public class AutoBlueLeft extends LinearOpMode {
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    //
//    public DcMotor FlMotor;
//    public DcMotor BlMotor;
//    public DcMotor FrMotor;
//    public DcMotor BrMotor;
//    public DcMotor SpinnerMotor;
//    public DcMotor InandOut;
//    public DcMotor UpandDown;
//
//    public Servo Grabber;
//
//    //28 * 20 / (2ppi * 4.125)
//    Double width = 18.0; //inches
//    Integer cpr = 28; //counts per rotation
//    Integer gearratio = 40;
//    Double diameter = 4.125;
//    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
//    Double bias = 0.8;//default 0.8
//    Double meccyBias = 0.9;//change to adjust only strafing movement
//    //
//    Double conversion = cpi * bias;
//    Boolean exit = false;
//    //
//    BNO055IMU imu;
//    Orientation angles;
//    Acceleration gravity;
//    //
//
//
//    public void runOpMode() {
//
//        telemetry.clearAll();
//        telemetry.addData("Status", "Auto Initialization In Progress");
//        telemetry.update();
//        //
////        initGyro();
//        //
//        BlMotor = hardwareMap.get(DcMotor.class, "Backleft");
//        FlMotor = hardwareMap.get(DcMotor.class, "Frontleft");
//        BrMotor = hardwareMap.get(DcMotor.class, "Backright");
//        FrMotor = hardwareMap.get(DcMotor.class, "Frontright");
//
//        BlMotor.setDirection(DcMotor.Direction.FORWARD);
//        FlMotor.setDirection(DcMotor.Direction.FORWARD);
//        BrMotor.setDirection(DcMotor.Direction.REVERSE);
//        FrMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        RingDetector detector = new RingDetector(this);
//
//        //
//        FlMotor.setPower(0);
//        FrMotor.setPower(0);
//        BlMotor.setPower(0);
//        BrMotor.setPower(0);
//
//
//        telemetry.clearAll();
//        telemetry.addData("Status", "Auto Initialization complete");
//        telemetry.update();
//
//
//        waitForStart();
//
//        int rings = detector.getDecision();
//
//        if (rings == 4) {
//
//            strafeToPosition(90, 0.8);
////            //
////            moveToPosition(6.8, 0.8);
////
////            sleep(1000);
////
////            WobbleMove(-60, 0.8);
////
////            sleep(3000);
////
////            WobbleUnGrab();
////
////            sleep(1000);
////            //
////            moveToPosition(-17, 0.8);
////            //
////            strafeToPosition(-45, 0.8);
//////            turns robot
////            encoderDrive(0.8,-12,12,5000);
//////
////            Shooter();
//        }
//
//        if (rings == 0) {
//
//            strafeToPosition(35, 0.8);
////            //
////            moveToPosition(6.8, 0.8);
////
////            sleep(1000);
////
////            WobbleMove(-60, 0.8);
////
////            sleep(3000);
////
////            WobbleUnGrab();
////
////            sleep(1000);//
////            moveToPosition(-17, 0.8);
////            //
////            strafeToPosition(-5, 0.8);
//////            turns robot
////            encoderDrive(0.8,-12,12,5000);
//////
////            Shooter();
//
//        }
//
//
//        if (rings == 1) {
//
//
//            strafeToPosition(55, 0.8);
////
////            moveToPosition(-10,0.5);
////            //
////            sleep(1000);
////
////            WobbleMove(-60, 0.8);
////
////            sleep(3000);
////
////            WobbleUnGrab();
////
////            sleep(1000);
////
////            //
////            strafeToPosition(-17.5, 0.8);
//////            turns robot
////            encoderDrive(0.8,-12,12,5000);
//////
////            Shooter();
//
//        }
//	//
//    }
//
//    //Wobble arm
////    public void WobbleArm(double inches, double speed) {
////        int move =  (int)(Math.round(inches*conversion));
////        //WobbleFlipper.setTargetPosition(WobbleFlipper.getCurrentPosition() + move);
////
////       // WobbleFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////        //WobbleFlipper.setPower(speed);
////    }
////
////    // Wobble Grab
////    //public void WobbleGrab() {
////        WobbleGrabber.setPosition(1);
////    }
////
////    // Wobble UnGrab
////    public void WobbleUnGrab() {
////        WobbleGrabber.setPosition(0);
////    }
////
////    public void WobbleMove(double inches, double speed) {
////        int move =  (int)(Math.round(inches*conversion));
////
////        WobbleFlipper.setTargetPosition(WobbleFlipper.getCurrentPosition() + move);
////
////        WobbleFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////        WobbleFlipper.setPower(speed);
////
////    }
//
//    //
//    /*
//    This function's purpose is simply to drive forward or backward.
//    To drive backward, simply make the inches input negative.
//     */
//    public void moveToPosition(double inches, double speed){
//        //
//        int move = (int)(Math.round(inches*conversion));
//        //
//        BlMotor.setTargetPosition(BlMotor.getCurrentPosition() + move);
//        FlMotor.setTargetPosition(FlMotor.getCurrentPosition() + move);
//        BrMotor.setTargetPosition(BrMotor.getCurrentPosition() + move);
//        FrMotor.setTargetPosition(FrMotor.getCurrentPosition() + move);
//        //
//        FlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        FlMotor.setPower(speed);
//        BlMotor.setPower(speed);
//        FrMotor.setPower(speed);
//        BrMotor.setPower(speed);
//        //
//        while (FlMotor.isBusy() && FrMotor.isBusy() && BlMotor.isBusy() && BrMotor.isBusy()){
//            if (exit){
//                FrMotor.setPower(0);
//                FlMotor.setPower(0);
//                BrMotor.setPower(0);
//                BlMotor.setPower(0);
//                return;
//            }
//        }
//        FrMotor.setPower(0);
//        FlMotor.setPower(0);
//        BrMotor.setPower(0);
//        BlMotor.setPower(0);
//        return;
//    }
//    //
//    /*
//    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
//    Degrees should always be positive, make speedDirection negative to turn left.
//     */
//    public void turnWithGyro(double degrees, double speedDirection){
//        //<editor-fold desc="Initialize">
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double yaw = -angles.firstAngle;//make this negative
//        telemetry.addData("Speed Direction", speedDirection);
//        telemetry.addData("Yaw", yaw);
//        telemetry.update();
//        //
//        telemetry.addData("stuff", speedDirection);
//        telemetry.update();
//        //
//        double first;
//        double second;
//        //</editor-fold>
//        //
//        if (speedDirection > 0){//set target positions
//            //<editor-fold desc="turn right">
//            if (degrees > 10){
//                first = (degrees - 10) + devertify(yaw);
//                second = degrees + devertify(yaw);
//            }else{
//                first = devertify(yaw);
//                second = degrees + devertify(yaw);
//            }
//            //</editor-fold>
//        }else{
//            //<editor-fold desc="turn left">
//            if (degrees > 10){
//                first = devertify(-(degrees - 10) + devertify(yaw));
//                second = devertify(-degrees + devertify(yaw));
//            }else{
//                first = devertify(yaw);
//                second = devertify(-degrees + devertify(yaw));
//            }
//            //
//            //</editor-fold>
//        }
//        //
//        //<editor-fold desc="Go to position">
//        Double firsta = convertify(first - 5);//175
//        Double firstb = convertify(first + 5);//-175
//        //
//        turnWithEncoder(speedDirection);
//        //
//        if (Math.abs(firsta - firstb) < 11) {
//            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }else{
//            //
//            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }
//        //
//        Double seconda = convertify(second - 5);//175
//        Double secondb = convertify(second + 5);//-175
//        //
//        turnWithEncoder(speedDirection / 3);
//        //
//        if (Math.abs(seconda - secondb) < 11) {
//            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            FlMotor.setPower(0);
//            FrMotor.setPower(0);
//            BlMotor.setPower(0);
//            BrMotor.setPower(0);
//        }
//        //</editor-fold>
//        //
//        FlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    //
//    /*
//    This function uses the encoders to strafe left or right.
//    Negative input for inches results in left strafing.
//     */
//    public void strafeToPosition(double inches, double speed){
//        //
//        int move = (int)(Math.round(inches * cpi * meccyBias));
//        //
//        BlMotor.setTargetPosition(BlMotor.getCurrentPosition() - move);
//        FlMotor.setTargetPosition(FlMotor.getCurrentPosition() + move);
//        BrMotor.setTargetPosition(BrMotor.getCurrentPosition() + move);
//        FrMotor.setTargetPosition(FrMotor.getCurrentPosition() - move);
//        //
//        FlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        FlMotor.setPower(speed);
//        BlMotor.setPower(speed);
//        FrMotor.setPower(speed);
//        BrMotor.setPower(speed);
//        //
//        while (FlMotor.isBusy() && FrMotor.isBusy() && BlMotor.isBusy() && BrMotor.isBusy()){}
//        FrMotor.setPower(0);
//        FlMotor.setPower(0);
//        BrMotor.setPower(0);
//        BlMotor.setPower(0);
//        return;
//    }
//
//    //
//    /*
//    These functions are used in the turnWithGyro function to ensure inputs
//    are interpreted properly.
//     */
//    public double devertify(double degrees){
//        if (degrees < 0){
//            degrees = degrees + 360;
//        }
//        return degrees;
//    }
//    public double convertify(double degrees){
//        if (degrees > 179){
//            degrees = -(360 - degrees);
//        } else if(degrees < -180){
//            degrees = 360 + degrees;
//        } else if(degrees > 360){
//            degrees = degrees - 360;
//        }
//        return degrees;
//    }
//    //
//    /*
//    This function is called at the beginning of the program to activate
//    the IMU Integrated Gyro.
//     */
//    public void initGyro(){
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        //
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//    }
//    //
//    /*
//    This function is used in the turnWithGyro function to set the
//    encoder mode and turn.
//     */
//    public void turnWithEncoder(double input){
//        FlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //
//        FlMotor.setPower(input);
//        BlMotor.setPower(input);
//        FrMotor.setPower(-input);
//        BrMotor.setPower(-input);
//    }
//
//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS) {
//        int newLeftFrontTarget;
//        int newRightFrontTarget;
//        int newLeftBackTarget;
//        int newRightBackTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftFrontTarget = FlMotor.getCurrentPosition() + (int)(leftInches * cpi);
//            newRightFrontTarget = FrMotor.getCurrentPosition() + (int)(rightInches * cpi);
//            newLeftBackTarget  = BlMotor.getCurrentPosition() + (int) (leftInches * cpi);
//            newRightBackTarget = BrMotor.getCurrentPosition() + (int) (rightInches * cpi);
//            FlMotor.setTargetPosition(newLeftFrontTarget);
//            FrMotor.setTargetPosition(newRightFrontTarget);
//            BrMotor.setTargetPosition(newRightBackTarget);
//            BlMotor.setTargetPosition(newLeftBackTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            FlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            FrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            BlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            BrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            FlMotor.setPower(Math.abs(speed));
//            FrMotor.setPower(Math.abs(speed));
//            BrMotor.setPower(Math.abs(speed));
//            BlMotor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (FrMotor.isBusy() && FlMotor.isBusy() && BlMotor.isBusy() && BrMotor.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newLeftFrontTarget, newRightBackTarget, newRightFrontTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        FlMotor.getCurrentPosition(),
//                        FrMotor.getCurrentPosition(),
//                        BrMotor.getCurrentPosition(),
//                        BlMotor.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            FrMotor.setPower(0);
//            FlMotor.setPower(0);
//            BrMotor.setPower(0);
//            BlMotor.setPower(0);
//            // Turn off RUN_TO_POSITION
//            FlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            FrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            BlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            BrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }
//
//
//
//    //
//}
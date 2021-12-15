package org.firstinspires.ftc.team6168;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Utilities{

    public static void strafeToPosition(DcMotor[] motors, double inches, double speed) {
        DcMotor frontleft = motors[0];
        DcMotor frontright = motors[1];
        DcMotor backleft = motors[2];
        DcMotor backright = motors[3];

        Integer cpr = 28; //counts per rotation
        Integer gearratio = 40;
        Double diameter = 4.125;
        Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
        Double bias = 0.8;//default 0.8
        Double meccyBias = 0.9;//change to adjust only strafing movement


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

            frontright.setPower(0);
            frontleft.setPower(0);
            backright.setPower(0);
            backleft.setPower(0);
            return;
        }
    }

    public static void initGyro(){
        BNO055IMU imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = BlocksOpModeCompanion.hardwareMap.get(BNO055IMU.class, "imu");
        //Test
        imu.initialize(parameters);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    public boolean onHeading (DcMotor[] motors, double speed, double angle, double PCoeff){
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        DcMotor frontleft = motors[0];
        DcMotor frontright = motors[1];
        DcMotor backleft = motors[2];
        DcMotor backright = motors[3];

        // Send desired speeds to motors.
        frontleft.setPower(leftSpeed);
        backleft.setPower(leftSpeed);
        backright.setPower(rightSpeed);
        frontright.setPower(rightSpeed);

        // Display it for the driver.
        BlocksOpModeCompanion.telemetry.addData("Target", "%5.2f", angle);
        BlocksOpModeCompanion.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        BlocksOpModeCompanion.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public static double getError(double targetAngle){

        ModernRoboticsI2cGyro gyro = null;

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return -robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public static double getSteer(double error, double PCoeff){
        double DRIVE_SPEED = 0.2;

        return Range.clip(error * PCoeff, -DRIVE_SPEED, 1);
    }

    public static void gyroDrive(DcMotor[] motors,
                                 double speed,
                                 double frontLeftInches, double frontRightInches, double backLeftInches,
                                 double backRightInches,
                                 double angle){

        Integer cpr = 28; //counts per rotation
        Integer gearratio = 40;
        Double diameter = 4.125;
        Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))

        double amountError = 2;

        double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        double HalfMaxOne;
        double HalfMaxTwo;

        double max;

        double error;
        double steer;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (true) {

            DcMotor frontleft = motors[0];
            DcMotor frontright = motors[1];
            DcMotor backleft = motors[2];
            DcMotor backright = motors[3];





            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontleft.getCurrentPosition() + (int) (frontLeftInches * cpi);
            newFrontRightTarget = frontright.getCurrentPosition() + (int) (frontRightInches * cpi);
            newBackLeftTarget = backleft.getCurrentPosition() + (int) (backLeftInches * cpi);
            newBackRightTarget = backright.getCurrentPosition() + (int) (backRightInches * cpi);


            // Set Target and Turn On RUN_TO_POSITION
            frontleft.setTargetPosition(newFrontLeftTarget);
            frontright.setTargetPosition(newFrontRightTarget);
            backleft.setTargetPosition(newBackLeftTarget);
            backright.setTargetPosition(newBackRightTarget);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontleft.setPower(Math.abs(speed));
            frontright.setPower(Math.abs(speed));
            backleft.setPower(Math.abs(speed));
            backright.setPower(Math.abs(speed));
            // keep looping while we are still active, and BOTH motors are running.
            while ((frontleft.isBusy() && frontright.isBusy()) && (backleft.isBusy() && backright.isBusy()) && !goodEnough) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (frontLeftInches < 0 && frontRightInches < 0 && backLeftInches < 0 && backRightInches < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                frontleft.setPower(frontLeftSpeed);
                frontright.setPower(frontRightSpeed);
                backleft.setPower(backLeftSpeed);
                backright.setPower(backRightSpeed);

                // Display drive status for the driver.
                BlocksOpModeCompanion.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                BlocksOpModeCompanion.telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                BlocksOpModeCompanion.telemetry.addData("Actual", "%7d:%7d", backleft.getCurrentPosition(), backright.getCurrentPosition(), frontleft.getCurrentPosition(), frontright.getCurrentPosition());
                BlocksOpModeCompanion.telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                BlocksOpModeCompanion.telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backleft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontleft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backright.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontright.getCurrentPosition()))))) / cpi);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

            // Turn off RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

//    public void gyroTurn (DcMotor[] motors, double speed, double angle){
//        DcMotor frontleft = motors[0];
//        DcMotor frontright = motors[1];
//        DcMotor backleft = motors[2];
//        DcMotor backright = motors[3];
//
//        double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
//
//
//        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        // keep looping while we are still active, and not on heading.
//        while (!onHeading(speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            BlocksOpModeCompanion.telemetry.update();
//        }
//        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    /**
//     *  Method to obtain & hold a heading for a finite amount of time
//     *  Move will stop once the requested time has elapsed
//     *
//     * @param speed      Desired speed of turn.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     * @param holdTime   Length of time (in seconds) to hold the specified heading.
//     */
//
//    public void gyroHold (DcMotor[] motors, double speed, double angle, double holdTime){
//
//        ElapsedTime holdTimer = new ElapsedTime();
//
//        // keep looping while we have time remaining.
//        holdTimer.reset();
//        while (holdTimer.time() < holdTime) {
//
//            double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
//
//            // Update telemetry & Allow time for other processes to run.
//            onHeading(speed, angle, P_TURN_COEFF);
//            BlocksOpModeCompanion.telemetry.update();
//
//
//        }
//        DcMotor frontleft = motors[0];
//        DcMotor frontright = motors[1];
//        DcMotor backleft = motors[2];
//        DcMotor backright = motors[3];
//
//        // Stop all motion;
//        frontleft.setPower(0);
//        backleft.setPower(0);
//        backright.setPower(0);
//        frontright.setPower(0);
//    }
}
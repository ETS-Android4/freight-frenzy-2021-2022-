/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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



    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        BlMotor = hardwareMap.get(DcMotor.class, "Backleft");
        FlMotor = hardwareMap.get(DcMotor.class, "Frontleft");
        BrMotor = hardwareMap.get(DcMotor.class, "Backright");
        FrMotor = hardwareMap.get(DcMotor.class, "Frontright");
        SpinnerMotor = hardwareMap.get(DcMotor.class, "OandH");

//  SET POSITION OF OUR SERVOS EXAMPLE BELOW
//        wideGrabber.setPosition(1);


        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        BlMotor.setDirection(DcMotor.Direction.FORWARD);
        FlMotor.setDirection(DcMotor.Direction.FORWARD);
        BrMotor.setDirection(DcMotor.Direction.REVERSE);
        FrMotor.setDirection(DcMotor.Direction.REVERSE);
        SpinnerMotor.setDirection(DcMotor.Direction.FORWARD);

        BlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SpinnerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SpinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FlMotor.setPower(0);
        FrMotor.setPower(0);
        BlMotor.setPower(0);
        BrMotor.setPower(0);
        SpinnerMotor.setPower(0);




        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void loop() {

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
            SpinnerMotor.setPower(1);
        }else {
            SpinnerMotor.setPower(0);
        }


    }


}
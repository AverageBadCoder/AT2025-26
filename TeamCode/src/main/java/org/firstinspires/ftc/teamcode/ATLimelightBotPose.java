/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;

@TeleOp(name="Limelight AutoMove", group="Linear OpMode")
public class ATLimelightBotPose extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;
    private DcMotorEx fwl = null;
    private DcMotorEx fwr = null;
    private DcMotor intake1 = null;
    private Servo sorting1 = null;
    private Servo sorting2 = null;
    private Servo limelightmount = null;



    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        fL = hardwareMap.get(DcMotorEx.class, "fL");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwr = hardwareMap.get(DcMotorEx.class, "fwr");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        sorting1 = hardwareMap.get(Servo.class, "sorting1");
        sorting2 = hardwareMap.get(Servo.class, "sorting2");
        limelightmount = hardwareMap.get(Servo.class, "limelightmount");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        fL.setDirection(DcMotor.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bL.setDirection(DcMotor.Direction.FORWARD);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fR.setDirection(DcMotor.Direction.REVERSE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bR.setDirection(DcMotor.Direction.REVERSE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fwl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fwr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fwl.setDirection(DcMotor.Direction.FORWARD);
        fwr.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{fL, fR, bL, bR}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        limelight.start();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        double maxVelocity = 60000; // adjust to your motor's spec
        double step = 50;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double x = 0, y = 0, yaw = 0;

            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                x = botpose.getPosition().x;
                y = botpose.getPosition().y;
                yaw = botpose.getOrientation().getYaw();
                telemetry.addData("X (m)", x);
                telemetry.addData("Y (m)", y);
                telemetry.addData("Yaw (deg)", Math.toDegrees(yaw));
            } else {
                telemetry.addLine("No Limelight pose");
            }

            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            if (gamepad1.a && result != null && result.isValid()) {
                driveToOrigin(x, y);
            } else {
                driveMecanum(axial, lateral, rotation);
            }
//regular code

            if (gamepad1.left_trigger > 0.1 && gamepad1.left_trigger < 0.5) { //intake speed forward
                intake1.setDirection(DcMotor.Direction.FORWARD);
                intake1.setPower(.5);
            } else if (gamepad1.left_trigger > 0.5) {
                intake1.setPower(.9);
            }

            if (gamepad1.right_trigger > 0.1 && gamepad1.right_trigger < 0.5) { //reverse direction
                intake1.setDirection(DcMotor.Direction.REVERSE);
                intake1.setPower(.5);
            } else if (gamepad1.right_trigger > 0.5) {
                intake1.setPower(.9);
            } else {
                intake1.setPower(0);
            }
            if (gamepad1.b) {
                fwr.setPower(0);
                fwl.setPower(0);
            }
            if (gamepad1.b) {
             sorting1.setPosition(0.5);
            }
            if (gamepad1.b) {
                sorting2.setPosition(0.61);
            }
            if (gamepad1.b) {
                limelightmount.setPosition(0.61);
            }






            telemetry.update();
        }
    }

    private void driveToOrigin(double x, double y) {
        // Simple proportional control to move toward (0,0)
        double kP = 0.8;  // Tune this value (0.5–1.0 works well)
        double tolerance = 0.1; // meters from origin

        double distance = Math.sqrt(x * x + y * y);
        double targetLateral = -y * kP;
        double targetAxial = x * kP;

        // Normalize motor powers
        double frontLeftPower = targetAxial + targetLateral;
        double frontRightPower = targetAxial - targetLateral;
        double backLeftPower = targetAxial - targetLateral;
        double backRightPower = targetAxial + targetLateral;
        telemetry.addData("target lateral", targetLateral);
        telemetry.addData("target axial", targetAxial);

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        if (distance > tolerance) {
            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);
        } else {
            // Stop when near origin
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
        }


    }

    private void driveMecanum(double axial, double lateral, double yaw) {
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        fL.setPower(frontLeftPower);
        fR.setPower(frontRightPower);
        bL.setPower(backLeftPower);
        bR.setPower(backRightPower);
        fwl.setVelocity(fwlSpeed);
        fwr.setVelocity(fwrSpeed);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.addData("Flywheel Left Speed", "%.2f ticks/sec", fwlSpeed);
        telemetry.addData("Flywheel Right Speed", "%.2f ticks/sec", fwrSpeed);
        telemetry.addData("Flywheel Left (actual)", "%.2f ticks/sec", fwl.getVelocity());
        telemetry.addData("Flywheel Right (actual)", "%.2f ticks/sec", fwr.getVelocity());

        telemetry.update();
    }

}

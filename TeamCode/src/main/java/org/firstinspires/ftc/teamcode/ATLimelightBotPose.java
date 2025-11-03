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

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;

import com.qualcomm.robotcore.hardware.CRServo;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Limelight AutoMove", group="Linear OpMode")
public class ATLimelightBotPose extends LinearOpMode {
    GoBildaPinpointDriver odo;
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
    private CRServo limelightmount = null;
    private Limelight3A limelight;
    private ColorSensor colorSensor;
    private int servoIndex = 0;  // start at first position
    private String[] slotColors = {"Empty", "Empty", "Empty"};
    private String lastBallColor = "Unknown";

    @Override
    public void runOpMode() {
        fL = hardwareMap.get(DcMotorEx.class, "fL");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwr = hardwareMap.get(DcMotorEx.class, "fwr");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        sorting1 = hardwareMap.get(Servo.class, "sorting1");
        sorting2 = hardwareMap.get(Servo.class, "sorting2");
        limelightmount = hardwareMap.get(CRServo.class, "limelightmount");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double currX = 0, currY = 0;
            LLStatus status = limelight.getStatus();
            adjustLl(result);

            telemetry.addData("Limelight Device Name", limelight.getDeviceName());
            telemetry.addData("Pipeline Index", status.getPipelineIndex());
            telemetry.addData("Has Result", (result != null));
            telemetry.addData("Result Valid", (result != null && result.isValid()));
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                currX = botpose.getPosition().x;
                currY = botpose.getPosition().y;
                telemetry.addData("X (m)", currX);
                telemetry.addData("Y (m)", currY);
            } else {
                telemetry.addLine("No Limelight pose");
            }

//            driver values
            double axial   = gamepad1.left_stick_y;
            double lateral =  -gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            double targX = 0, targY = 0, targYaw = 0;
            if (gamepad1.a && result != null && result.isValid()) {
                driveToOrigin(currX, targX, currY, targY, targYaw);
            } else {
                driveMecanum(axial, lateral, rotation);
            }

            String ballColor = checkColor();  // Get current detected color
            sorting1.setPosition(suzani[servoIndex]);

            if (gamepad1.left_trigger > 0.1) {
                intake1.setDirection(DcMotor.Direction.REVERSE);
                if (ballColor.equals("Unknown")) {
                    intake1.setPower(intakeSpeed);
                    lastBallColor = "Unknown";
                } else {
                    intake1.setPower(0);
                    if (lastBallColor.equals("Unknown")) {
                        ballColor = checkColor();
                        lastBallColor = ballColor;
                        // Store color in current slot
                        slotColors[servoIndex] = ballColor;
                        // Advance to next slot if available
                        if (servoIndex < 2) {
                            servoIndex++;
                            sorting1.setPosition(suzani[servoIndex]);
                        }
                    }
                }
            } else if (gamepad1.right_trigger > 0.1) {
                intake1.setDirection(DcMotor.Direction.FORWARD);
            } else {
                intake1.setPower(0);
            }
            telemetry.addData("Slot 1", slotColors[0]);
            telemetry.addData("Slot 2", slotColors[1]);
            telemetry.addData("Slot 3", slotColors[2]);
            telemetry.addData("Servo Index", servoIndex);

            if (gamepad1.b) {
                String[] pattern = {"purple", "green", "purple"};
                List<Double> servoSequence = new ArrayList<>();

                // Copy of slotColors so we don't reuse the same slot twice
                boolean[] used = new boolean[slotColors.length];

                for (String targetColor : pattern) {
                    for (int i = 0; i < slotColors.length; i++) {
                        if (!used[i] && slotColors[i].equalsIgnoreCase(targetColor)) {
                            servoSequence.add(suzano[i]);  // add servo position corresponding to that slot
                            used[i] = true;                // mark slot as used
                            break;                         // move on to next pattern color
                        }
                    }
                }

                fwr.setPower(0);
                fwl.setPower(0);
                for (int i = 0; i < servoSequence.size(); i++) {
                    double pos = servoSequence.get(i);
                    sorting1.setPosition(pos);
                    sleep(1000);  // wait for servo to reach position
                    sorting2.setPosition(wackUp);
                    sleep(1000);
                    sorting2.setPosition(wackDown);
                    sleep(1000);
                }
            }
            if (gamepad1.x) {
                sorting2.setPosition(wackDown);//three postions are .82, .44, .07
            }
            if (gamepad1.y) {
                sorting2.setPosition(wackUp);
            }
            if (gamepad1.dpad_left) {
                limelightmount.setPower(0.61);
            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flywheel Left Speed", "%.2f ticks/sec", fwlSpeed);
            telemetry.addData("Flywheel Right Speed", "%.2f ticks/sec", fwrSpeed);
            telemetry.addData("Flywheel Left (actual)", "%.2f ticks/sec", fwl.getVelocity());
            telemetry.addData("Flywheel Right (actual)", "%.2f ticks/sec", fwr.getVelocity());

        }
    }

    private void driveToOrigin(double currX, double targX, double currY, double targY, double targYaw) {
        Pose2D pos = odo.getPosition();
        odo.update();
        double currYaw = pos.getHeading(AngleUnit.DEGREES);

        // Tunable gains
        double kP_drive = 0.8;     // position proportional gain
        double kP_turn  = 1.0;     // yaw proportional gain (adjust 0.5–1.5)
        double tolerance = 0.1;    // meters from target
        double yawTolerance = Math.toRadians(2); // degrees of allowable yaw error

        // --- Compute position error in field coordinates ---
        double fieldErrorX = targX - currX;
        double fieldErrorY = targY - currY;
        double distance = Math.sqrt(fieldErrorX * fieldErrorX + fieldErrorY * fieldErrorY);

        // --- Compute heading error (normalize to [-π, π]) ---
        double yawError = targYaw - currYaw;
        yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

        // --- Rotate error into robot's frame so motion stays straight ---
        // This ensures that translation remains in a fixed world direction
        double robotErrorX =  fieldErrorX * Math.cos(-currYaw) - fieldErrorY * Math.sin(-currYaw);
        double robotErrorY =  fieldErrorX * Math.sin(-currYaw) + fieldErrorY * Math.cos(-currYaw);

        // --- Proportional control for motion and rotation ---
        double targetAxial   = robotErrorX * kP_drive;   // forward/back
        double targetLateral = -robotErrorY * kP_drive;  // strafe
        double targetYawPower = yawError * kP_turn;      // spin to target yaw

        // --- Combine for mecanum drive ---
        double frontLeftPower  = targetAxial + targetLateral + targetYawPower;
        double frontRightPower = targetAxial - targetLateral - targetYawPower;
        double backLeftPower   = targetAxial - targetLateral + targetYawPower;
        double backRightPower  = targetAxial + targetLateral - targetYawPower;

        // --- Normalize motor powers ---
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // --- Apply power or stop if within tolerance ---
        if (distance > tolerance || Math.abs(yawError) > yawTolerance) {
            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);
        } else {
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
        }

        // --- Telemetry for debugging/tuning ---
        telemetry.addData("Field Error X", fieldErrorX);
        telemetry.addData("Field Error Y", fieldErrorY);
        telemetry.addData("Robot Error X", robotErrorX);
        telemetry.addData("Robot Error Y", robotErrorY);
        telemetry.addData("Distance", distance);
        telemetry.addData("Yaw Error (deg)", Math.toDegrees(yawError));
        telemetry.addData("Target Axial", targetAxial);
        telemetry.addData("Target Lateral", targetLateral);
        telemetry.addData("Target Yaw Power", targetYawPower);
        telemetry.update();
    }


    private void driveMecanum(double axial, double lateral, double yaw) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        fL.setPower(frontLeftPower);
        fR.setPower(frontRightPower);
        bL.setPower(backLeftPower);
        bR.setPower(backRightPower);
    }

    private void adjustLl(LLResult result){
        if (result != null && result.isValid()) {
            double kP = 0.1;
            double deadband = 10;
            double servoPower = 0;

            double tx = result.getTx();
            if (tx > 0){
                servoPower = kP;
            } else {
                servoPower = -kP;
            }

            if (Math.abs(tx) < deadband) {
                servoPower = 0;
            }
            limelightmount.setPower(servoPower);
        } else {
            limelightmount.setPower(0);
        }
    }

    private String checkColor(){
        double[] current = {
                colorSensor.red(),
                colorSensor.green(),
                colorSensor.blue(),
                colorSensor.alpha()
        };
        // Compute Euclidean distance to each reference
        double purpleDistance = 0;
        double greenDistance = 0;
        for (int i = 0; i < 4; i++) {
            purpleDistance += Math.pow(current[i] - purpleBall[i], 2);
            greenDistance  += Math.pow(current[i] - greenBall[i], 2);
        }
        purpleDistance = Math.sqrt(purpleDistance);
        greenDistance  = Math.sqrt(greenDistance);

        // Choose the closer match if it’s within a tolerance
        double tolerance = 40; // adjust as needed
        String detectedColor = "Unknown";

        if (purpleDistance < greenDistance && purpleDistance < tolerance) {
            detectedColor = "Purple";
        } else if (greenDistance < purpleDistance && greenDistance < tolerance) {
            detectedColor = "Green";
        }

        // Telemetry for debugging
        telemetry.addData("Red", current[0]);
        telemetry.addData("Green", current[1]);
        telemetry.addData("Blue", current[2]);
        telemetry.addData("Alpha", current[3]);
        telemetry.addData("Purple Distance", purpleDistance);
        telemetry.addData("Green Distance", greenDistance);
        telemetry.addData("Detected", detectedColor);
        telemetry.update();

        return detectedColor;
    }


}

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
import com.qualcomm.hardware.rev.RevColorSensorV3;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="AT Tele-Op", group="Linear OpMode")
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
    private Servo limelightmount = null;
    private Limelight3A limelight;
    private double llServoPos = 0.3;
    private ColorSensor colorSensor;
    private int servoIndex = 0;  // start at first position
    private String[] slotColors = {"Empty", "Empty", "Empty"};
    private String lastBallColor = "Unknown";
    private boolean sweepingForward = true;
    private boolean intakeReady = true;

//    manual booleans
    private int manualServoIndex = 0;
    private boolean flywheelOn = false;
    private boolean aWasPressed = false;
    private boolean xWasPressed = false;
    ElapsedTime llTimer = new ElapsedTime();

    @Override//
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-65.0, -145.5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        fL = hardwareMap.get(DcMotorEx.class, "fL");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwr = hardwareMap.get(DcMotorEx.class, "fwr");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        sorting1 = hardwareMap.get(Servo.class, "sorting1");
        sorting2 = hardwareMap.get(Servo.class, "sorting2");
        limelightmount = hardwareMap.get(Servo.class, "limelightmount");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

        String[] pattern = {"", "", ""};
//        get pattern
        limelight.pipelineSwitch(0);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            int tagId = 0;
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tagId = fr.getFiducialId();
            }

            if (tagId == 21){
                pattern[0] = "green";
                pattern[1] = "purple";
                pattern[2] = "purple";
            } else if (tagId == 22){
                pattern[0] = "purple";
                pattern[1] = "green";
                pattern[2] = "purple";
            } else if (tagId == 23){
                pattern[0] = "purple";
                pattern[1] = "purple";
                pattern[2] = "green";
            }


            telemetry.addData("AprilTag ID", tagId);
            telemetry.addData("Pattern 0", pattern[0]);
            telemetry.addData("Pattern 1", pattern[1]);
            telemetry.addData("Pattern 2", pattern[2]);

        } else {
            telemetry.addLine("No valid AprilTag detected");
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double currX = 0, currY = 0;
            limelight.pipelineSwitch(1);
            result = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();
            adjustLl(result);
            odo.update();
            Pose2D pos = odo.getPosition();

            telemetry.addData("Yaw", pos.getHeading(AngleUnit.DEGREES));
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
            double rotation = -gamepad1.right_stick_x;
            if (gamepad1.dpad_up){
                axial = 0.3;
            }

            double targX = 0, targY = 0, targYaw = 0;
            if (gamepad1.a && result != null && result.isValid()) {
//                driveToOrigin(targX, targY, targYaw);
                driveToOrigin(blueX, blueY, blueYaw);
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
                } else if (intakeReady){
                    intake1.setPower(0);
                    if (lastBallColor.equals("Unknown")) {
                        intakeReady = false;
                        ballColor = checkColor();
                        lastBallColor = ballColor;
                        // Store color in current slot
                        slotColors[servoIndex] = ballColor;
                        new Thread(()->{
                            sleep(10);
                            if (servoIndex < 2) {
                                servoIndex++;
                                sorting1.setPosition(suzani[servoIndex]);
                            }
                        }).start();
//                        delay looking for new color
                        new Thread(()->{
                            sleep(2000);
                            intakeReady = true;
                        }).start();
                    }
                }
            } else if (gamepad1.right_trigger > 0.1) {
                intake1.setDirection(DcMotor.Direction.FORWARD);
                intake1.setPower(intakeSpeed);
            } else {
                intake1.setPower(0);
            }
            telemetry.addData("Slot 1", slotColors[0]);
            telemetry.addData("Slot 2", slotColors[1]);
            telemetry.addData("Slot 3", slotColors[2]);
            telemetry.addData("Servo Index", servoIndex);

            if (gamepad1.b) {
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

                fwl.setVelocity(fwSpeed);
                fwr.setVelocity(fwSpeed);
                sleep(2000);

                for (int i = 0; i < servoSequence.size(); i++) {
                    double servoPos = servoSequence.get(i);
                    sorting1.setPosition(servoPos);
                    sleep(1000);  // wait for servo to reach position
                    sorting2.setPosition(wackUp);
                    sleep(1000);
                    sorting2.setPosition(wackDown);
                    sleep(1000);
                }
                fwl.setVelocity(0);
                fwr.setVelocity(0);
                slotColors[0] = "Empty";
                slotColors[1] = "Empty";
                slotColors[2] = "Empty";
            }
            if (gamepad1.x) {
                sorting2.setPosition(wackDown);//three postions are .82, .44, .07
            }
            if (gamepad1.y) {
                sorting2.setPosition(wackUp);
            }

//            MANUAL OUTTAKING
            if (gamepad2.a && !aWasPressed) {
                aWasPressed = true;
                manualServoIndex++;
                if (manualServoIndex > 2) manualServoIndex = 0;
                sorting1.setPosition(suzani[manualServoIndex]);
            }
            if (!gamepad2.a) aWasPressed = false;
// --- Wack Up/Down (GP2.B) ---
            if (gamepad2.b) {
                sorting2.setPosition(wackUp);
                sleep(200);
                sorting2.setPosition(wackDown);
                sleep(200);
            }
// --- Flywheel Toggle (GP2.X) ---
            if (gamepad2.x && !xWasPressed) {
                xWasPressed = true;

                flywheelOn = !flywheelOn;

                if (flywheelOn) {
                    fwl.setVelocity(fwSpeed);
                    fwr.setVelocity(fwSpeed);
                } else {
                    fwl.setVelocity(0);
                    fwr.setVelocity(0);
                }
            }
            if (!gamepad2.x) xWasPressed = false;


            telemetry.addData("Flywheel Left", "%.2f ticks/sec", fwl.getVelocity());
            telemetry.addData("Flywheel Right", "%.2f ticks/sec", fwr.getVelocity());
            telemetry.update();
        }
    }

    private void driveToOrigin(double targX, double targY, double targYaw) {
        // Update odometry and get position
        odo.update();
        Pose2D pos = odo.getPosition();

        // Get heading in radians (always use radians for trig)
        double currYaw = pos.getHeading(AngleUnit.RADIANS);

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Pose3D botpose = result.getBotpose();
            double currX = botpose.getPosition().x;
            double currY = botpose.getPosition().y;

            // --- Tunable gains ---
            double kP_drive = 0.8;
            double kP_turn = 0.7;
            double tolerance = 0.1; // meters
            double yawTolerance = Math.toRadians(2); // radians

            // --- Field errors ---
            double fieldErrorX = targX - currX;
            double fieldErrorY = targY - currY;
            double distance = Math.hypot(fieldErrorX, fieldErrorY);

            // --- Heading error (normalized) ---
            double yawError = targYaw - currYaw;
            yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

            // --- Convert field error to robot coordinates ---
            double robotErrorX = fieldErrorX * Math.cos(-currYaw) - fieldErrorY * Math.sin(-currYaw);
            double robotErrorY = fieldErrorX * Math.sin(-currYaw) + fieldErrorY * Math.cos(-currYaw);

            // --- Proportional control ---
            double targetAxial = robotErrorX * kP_drive;   // forward/backward
            double targetLateral = -robotErrorY * kP_drive;  // left/right strafe
            double targetYawPower = yawError * kP_turn;      // rotation (if enabled)
//        targetYawPower = 0; // temporarily disabled for straight-line testing

            // --- Mecanum power mixing ---
            double frontLeftPower = targetAxial + targetLateral + targetYawPower;
            double frontRightPower = targetAxial - targetLateral - targetYawPower;
            double backLeftPower = targetAxial - targetLateral + targetYawPower;
            double backRightPower = targetAxial + targetLateral - targetYawPower;

            // --- Normalize powers ---
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // --- Apply or stop ---
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
            telemetry.addData("Yaw Error (deg)", Math.toDegrees(yawError));
        }
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

    private void adjustLl(LLResult result) {
        limelightmount.setPosition(llServoPos);
        if (llTimer.milliseconds() < 10) return;
        llTimer.reset();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal offset (deg)
            double kP = 0.0012;          // proportional gain (tune this)
            double step = kP * tx;
            llServoPos += step;
            llServoPos = Range.clip(llServoPos, llServoMin, llServoMax);
            limelightmount.setPosition(llServoPos);
            sleep(5);
            telemetry.addData("tx", tx);
        } else {
            if (sweepingForward) {
                llServoPos += searchSpeed;
                if (llServoPos >= llServoMax) {
                    llServoPos = llServoMax;
                    sweepingForward = false;
                }
            } else {
                llServoPos -= searchSpeed;
                if (llServoPos <= llServoMin) {
                    llServoPos = llServoMin;
                    sweepingForward = true;
                }
            }

            limelightmount.setPosition(llServoPos);
            telemetry.addLine("No target detected — searching...");
        }
    }

//
//    private String checkColor(){
//        double[] current = {
//                colorSensor.red(),
//                colorSensor.green(),
//                colorSensor.blue(),
//                colorSensor.alpha()
//        };
//        // Compute Euclidean distance to each reference
//        double purpleDistance = 0;
//        double greenDistance = 0;
//        for (int i = 0; i < 4; i++) {
//            purpleDistance += Math.pow(current[i] - purpleBall[i], 2);
//            greenDistance  += Math.pow(current[i] - greenBall[i], 2);
//        }
//        purpleDistance = Math.sqrt(purpleDistance);
//        greenDistance  = Math.sqrt(greenDistance);
//
//        // Choose the closer match if it’s within a tolerance
//        double tolerance = 40; // adjust as needed
//        String detectedColor = "Unknown";
//
//        if (purpleDistance < greenDistance && purpleDistance < tolerance) {
//            detectedColor = "Purple";
//        } else if (greenDistance < purpleDistance && greenDistance < tolerance) {
//            detectedColor = "Green";
//        }
//
//        // Telemetry for debugging
//        telemetry.addData("Red", current[0]);
//        telemetry.addData("Green", current[1]);
//        telemetry.addData("Blue", current[2]);
//        telemetry.addData("Alpha", current[3]);
//        telemetry.addData("Purple Distance", purpleDistance);
//        telemetry.addData("Green Distance", greenDistance);
//        telemetry.addData("Detected", detectedColor);
//        return detectedColor;
//    }

    private String checkColor() {
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();
        purpleBall = normalizeColor(purpleBall);
        greenBall  = normalizeColor(greenBall);

        // Normalize input
        double sum = red + green + blue;
        if (sum > 0) {
            red /= sum;
            green /= sum;
            blue /= sum;
        }

        // Compare with *normalized* reference colors
        double purpleDistance = colorDistance(new double[]{red, green, blue}, purpleBall);
        double greenDistance  = colorDistance(new double[]{red, green, blue}, greenBall);

        String detected = "Unknown";
        double tolerance = 0.12;  // tuned for normalized distances

        if (purpleDistance < greenDistance && purpleDistance < tolerance)
            detected = "Purple";
        else if (greenDistance < purpleDistance && greenDistance < tolerance)
            detected = "Green";

        telemetry.addData("Norm Red", red);
        telemetry.addData("Norm Green", green);
        telemetry.addData("Norm Blue", blue);
        telemetry.addData("Detected", detected);
        return detected;
    }


    private double[] normalizeColor(double[] rgb) {
        double sum = rgb[0] + rgb[1] + rgb[2];
        return new double[]{ rgb[0]/sum, rgb[1]/sum, rgb[2]/sum };
    }

    private double colorDistance(double[] c1, double[] c2) {
        return Math.sqrt(Math.pow(c1[0]-c2[0],2) + Math.pow(c1[1]-c2[1],2) + Math.pow(c1[2]-c2[2],2));
    }



}

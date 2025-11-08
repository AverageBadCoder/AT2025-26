package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import static org.firstinspires.ftc.teamcode.CONSTANTS.AutoFast;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="AT Auto", group="Robot")
//@Disabled
public class BlueAuto extends LinearOpMode {
    GoBildaPinpointDriver odo;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx fL = null;
    private DcMotorEx bL = null;
    private DcMotorEx fR = null;
    private DcMotorEx bR = null;
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
    private String[] slotColors = {"Purple", "Purple", "Green"};
    private String[] pattern = {"purple", "purple", "green"};
    private String lastBallColor = "Unknown";
    private boolean sweepingForward = true;
    private double heading = 0.0;
    private double axial = 0.0;
    private double lateral = 0.0;
    private double yaw = 0.0;

    private boolean intakeReady = true;
    private boolean firstShoot = false;
    private boolean firstIntake = false;

    private boolean needPattern = true;
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
        limelight.pipelineSwitch(1);
        limelight.start();
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        fL.setDirection(DcMotor.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bL.setDirection(DcMotor.Direction.FORWARD);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fR.setDirection(DcMotor.Direction.REVERSE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bR.setDirection(DcMotor.Direction.REVERSE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fwl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fwr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fwl.setDirection(DcMotor.Direction.FORWARD);
        fwr.setDirection(DcMotor.Direction.REVERSE);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                fL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds()<28.5){
            if (needPattern) {
//                checkPattern();
                sorting1.setPosition(suzani[servoIndex]);
                limelightmount.setPosition(SERVO_CENTER_POS);
                needPattern = false; // COMMENT OUT ONCE CHECK PATTERN IS ON
            } else {
                if (!firstShoot) {
//                    fwOn();
                    move(blueX, blueY, 0);
//                    rotateToHeading(blueShootYaw);
//                    outtake();
//                    rotateToHeading(0);
//                    move(blueIntake1X, blueIntakeY, 0);
//                    rotateToHeading(blueIntakeYaw);
//                    off();
                    firstShoot = true;
                } if (!firstIntake){
//                    intakeMacro();
//                    rotateToHeading(0);
//                    move(blueX, blueY, 0);
//                    rotateToHeading(blueShootYaw);
//                    outtake();
                    firstIntake = true;
                }
                off();
            }

            telemetry.addData("Pattern 0", pattern[0]);
            telemetry.addData("Pattern 1", pattern[1]);
            telemetry.addData("Pattern 2", pattern[2]);
            telemetry.update();
            sleep(1000);  // pause to display final telemetry message.
        }
    }

    public void off(){
        fL.setVelocity(0);
        bL.setVelocity(0);
        bR.setVelocity(0);
        fR.setVelocity(0);
    }

    private void move(double targetX, double targetY, double targetYaw) {
        double posTol = 2;

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currX = pos.getX(DistanceUnit.INCH);
            double currY = pos.getY(DistanceUnit.INCH);
            double currYaw = pos.getHeading(AngleUnit.RADIANS);

            // Error in field space
            double dx = targetX - currX;
            double dy = targetY - currY;

            // Distance to target
            double distance = Math.hypot(dx, dy);

            // Stop if close enough
            if (distance < posTol) {
                axial = 0;
                lateral = 0;
                yaw = 0;
                fL.setVelocity(0);
                fR.setVelocity(0);
                bL.setVelocity(0);
                bR.setVelocity(0);
                break;
            }

            // Proportional velocity control
            double slowdownScale = Math.min(distance / 8.0, 1.0);
            double axialVel   = -dx*slowdownScale*AutoFast;
            double lateralVel = dy*slowdownScale*AutoFast;
//            x + forward
//            y + left
            // Feed to robot motion system
            axial = axialVel;
            lateral = lateralVel;
            yaw = 0;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            double max = Math.max(Math.abs(fl), Math.abs(fr));
            max = Math.max(max, Math.abs(bl));
            max = Math.max(max, Math.abs(br));
            if (max > AutoFast) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            fL.setVelocity(fl);
            fR.setVelocity(fr);
            bL.setVelocity(bl);
            bR.setVelocity(br);
            telemetry.addData("currx", currX);
            telemetry.addData("curry", currY);
            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);
            telemetry.addData("Distance", distance);
            telemetry.addData("axial vel", axial);
            telemetry.addData("lat vel", lateral);
            telemetry.update();
        }
    }

    private void rotateToHeading(double targetHeadingDeg) {

        double yawTol = Math.toRadians(4);   // stop within 2 degrees
        double maxYawVel = 300;              // adjust to your motor units
        double kP_yaw = 2.0;                 // main tuning param

        double targetHeading = Math.toRadians(targetHeadingDeg);

        while (opModeIsActive()) {
            // Update odometry
            odo.update();
            Pose2D pos = odo.getPosition();
            double currYaw = pos.getHeading(AngleUnit.RADIANS);

            // Compute error
            double yawError = targetHeading - currYaw;
            yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

            telemetry.addData("TargetHeading", targetHeadingDeg);
            telemetry.addData("CurrentHeadingDeg", Math.toDegrees(currYaw));
            telemetry.addData("YawErrorDeg", Math.toDegrees(yawError));

            // Stop if close enough
            if (Math.abs(yawError) < yawTol) {
                fL.setVelocity(0);
                fR.setVelocity(0);
                bL.setVelocity(0);
                bR.setVelocity(0);
                telemetry.addLine("Rotation complete");
                return;
            }

            // Proportional yaw velocity
            double yawVel = -yawError * kP_yaw*maxYawVel;
            yawVel = Range.clip(yawVel, -maxYawVel, maxYawVel);

            // Apply to mecanum (zero translation)
            double fl =  yawVel;
            double fr = -yawVel;
            double bl =  yawVel;
            double br = -yawVel;

            fL.setVelocity(fl);
            fR.setVelocity(fr);
            bL.setVelocity(bl);
            bR.setVelocity(br);
            telemetry.update();
        }
    }

    private void checkPattern() {
        limelight.pipelineSwitch(0);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            int tagId = 0;
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tagId = fr.getFiducialId();
            }

            if (tagId == 21) {
                pattern[0] = "green";
                pattern[1] = "purple";
                pattern[2] = "purple";
                needPattern = false;
            } else if (tagId == 22) {
                pattern[0] = "purple";
                pattern[1] = "green";
                pattern[2] = "purple";
                needPattern = false;
            } else if (tagId == 23) {
                pattern[0] = "purple";
                pattern[1] = "purple";
                pattern[2] = "green";
                needPattern = false;
            }
            telemetry.addData("AprilTag ID", tagId);
            telemetry.addData("Pattern 0", pattern[0]);
            telemetry.addData("Pattern 1", pattern[1]);
            telemetry.addData("Pattern 2", pattern[2]);
        } else {
            telemetry.addLine("No valid AprilTag detected");
        }
    }

    private void outtake() {
        List<Double> servoSequence = new ArrayList<>();
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

//        also add unused slots in case color was mismatched
        for (int i = 0; i < slotColors.length; i++) {
            if (!used[i]) {
                servoSequence.add(suzano[i]);
                used[i] = true;
            }
        }

        for (int i = 0; i < servoSequence.size(); i++) {
            double servoPos = servoSequence.get(i);
            sorting1.setPosition(servoPos);
            sleep(1500);  // wait for servo to reach position
            sorting2.setPosition(wackUp);
            sleep(1000);
            sorting2.setPosition(wackDown);
            sleep(1000);
        }
        servoIndex=0;
        fwOff();
        slotColors[0] = "Empty";
        slotColors[1] = "Empty";
        slotColors[2] = "Empty";
    }

    private void intakeMacro(){
        lateral=0;
        yaw=0;
        servoIndex = 0;
        new Thread(()->{
            while (servoIndex < 2){
                intake();
            }
        }).start();

        axial = -AutoSlow;
        dumbMove();
        sleep(1000);
        off();
        sleep(1000);

        axial = -AutoSlow;
        dumbMove();
        sleep(1000);
        off();
        sleep(1000);

        axial = -AutoSlow;
        dumbMove();
        sleep(1000);
        off();
        sleep(1000);
    }

    private void dumbMove(){
        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        double max = Math.max(Math.abs(fl), Math.abs(fr));
        max = Math.max(max, Math.abs(bl));
        max = Math.max(max, Math.abs(br));
        if (max > AutoFast) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        fL.setVelocity(fl);
        fR.setVelocity(fr);
        bL.setVelocity(bl);
        bR.setVelocity(br);
    }

    private void intake() {
        String ballColor = checkColor();  // Get current detected color
        sorting1.setPosition(suzani[servoIndex]);
        telemetry.addData("Intake ready", intakeReady);

        intake1.setDirection(DcMotor.Direction.REVERSE);
        if (ballColor.equals("Unknown")) {
            intake1.setPower(intakeSpeed);
            lastBallColor = "Unknown";
        } else if (intakeReady) {
            intake1.setPower(0);
            intakeReady = false;
            ballColor = checkColor();
            lastBallColor = ballColor;
            // Store color in current slot
            slotColors[servoIndex] = ballColor;
            new Thread(() -> {
                sleep(10);
                if (servoIndex < 2) {
                    servoIndex++;
                    sorting1.setPosition(suzani[servoIndex]);
                }
                sleep(1500);
                intakeReady = true;
            }).start();
        }
    }

    private String checkColor() {
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();
        purpleBall = normalizeColor(purpleBall);
        greenBall = normalizeColor(greenBall);

        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);

        // Normalize input
        double sum = red + green + blue;
        if (sum > 0) {
            red /= sum;
            green /= sum;
            blue /= sum;
        }

        // Compare with *normalized* reference colors
        double purpleDistance = colorDistance(new double[]{red, green, blue}, purpleBall);
        double greenDistance = colorDistance(new double[]{red, green, blue}, greenBall);

        String detected = "Unknown";
        double tolerance = 0.06;  // tuned for normalized distances

        if (purpleDistance < greenDistance && purpleDistance < tolerance)
            detected = "Purple";
        else if (greenDistance < purpleDistance && greenDistance < tolerance)
            detected = "Green";

        telemetry.addData("Detected", detected);
        return detected;
    }

    private double[] normalizeColor(double[] rgb) {
        double sum = rgb[0] + rgb[1] + rgb[2];
        return new double[]{rgb[0] / sum, rgb[1] / sum, rgb[2] / sum};
    }

    private double colorDistance(double[] c1, double[] c2) {
        return Math.sqrt(Math.pow(c1[0] - c2[0], 2) + Math.pow(c1[1] - c2[1], 2) + Math.pow(c1[2] - c2[2], 2));
    }

    private void fwOn(){
        fwl.setVelocity(fwSpeed);
        fwr.setVelocity(fwSpeed);
    }

    private void fwOff(){
        fwl.setVelocity(0);
        fwr.setVelocity(0);
    }


}
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.CONSTANTS.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Limelight TeleOp", group="Linear OpMode")
//@Disabled
public class ATLimelightOpModeOldDoNotUse extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;
    private DcMotorEx fwl = null;
    private DcMotorEx fwr = null;
    private DcMotor intake1 = null;
    private Servo sorting1 = null;

    /*
    DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "myMotor");
    myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    myMotor.setVelocity(speed);

    where speed = (target ticks/second) * motor CPR
    CPR is counts per revolution of the encoder
     */




    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        fL = hardwareMap.get(DcMotor.class, "fL");
        bL = hardwareMap.get(DcMotor.class, "bL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwr = hardwareMap.get(DcMotorEx.class, "fwr");
//        intake1 = hardwareMap.get(DcMotor.class, "intake1");
//        sorting1 = hardwareMap.get(Servo.class, "sorting1");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
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

//        intake1.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        limelight.start();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        waitForStart();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();

                double x = botpose.getPosition().x; // in meters
                double y = botpose.getPosition().y; // in meters
                double yaw = botpose.getOrientation().getYaw(); // in radians

                telemetry.addData("Robot X (m)", x);
                telemetry.addData("Robot Y (m)", y);
                telemetry.addData("Robot Yaw (deg)", Math.toDegrees(yaw));
            } else {
                telemetry.addLine("no result");
            }
            sleep(200);

            if (gamepad1.left_trigger > 0.1 && gamepad1.left_trigger < 0.5){ //intake speed forward
                intake1.setDirection(DcMotor.Direction.FORWARD);
                intake1.setPower(.5);
            }
            else if (gamepad1.left_trigger > 0.5) {
                intake1.setPower(.9);
            }

            if (gamepad1.right_trigger > 0.1 && gamepad1.right_trigger < 0.5){ //reverse direction
                intake1.setDirection(DcMotor.Direction.REVERSE);
                intake1.setPower(.5);
            }

            else if (gamepad1.right_trigger > 0.5) {
                intake1.setPower(.9);
            }

            else {
                intake1.setPower(0);
            }

            if (gamepad1.a){
                //set fwr and fwl power
            }
            if (gamepad1.b){
                fwr.setPower(0);
                fwl.setPower(0);
            }



            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  -gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);
            fwl.setVelocity(fwlSpeed);
            fwr.setVelocity(fwrSpeed);

            // Show the elapsed game time and wheel power.
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
    }

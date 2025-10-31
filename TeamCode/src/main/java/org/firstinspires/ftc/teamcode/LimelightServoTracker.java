package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * This OpMode uses a Limelight 3 to detect an AprilTag,
 * then automatically turns a continuous rotation servo
 * so that the Limelight stays centered on the tag.
 */
@TeleOp(name = "Limelight Servo Tracker", group = "Concept")
@Disabled
public class LimelightServoTracker extends LinearOpMode {

    private Limelight3A limelight;
    private CRServo limelightmount;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware initialization
        limelightmount = hardwareMap.get(CRServo.class, "limelightmount");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Limelight Servo Tracker Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double currX = 0, currY = 0, currYaw = 0;
            LLStatus status = limelight.getStatus();

            telemetry.addData("Limelight Device Name", limelight.getDeviceName());
            telemetry.addData("Pipeline Index", status.getPipelineIndex());
            telemetry.addData("Has Result", (result != null));
            telemetry.addData("Result Valid", (result != null && result.isValid()));
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                currX = botpose.getPosition().x;
                currY = botpose.getPosition().y;
                currYaw = botpose.getOrientation().getYaw();
                telemetry.addData("X (m)", currX);
                telemetry.addData("Y (m)", currY);
                telemetry.addData("Yaw (deg)", Math.toDegrees(currYaw));
                telemetry.addData("Class", limelightmount.getClass().getSimpleName());
                double kP = 0.1;      // tune this (start small!)
                double deadband = 15; // degrees
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

                telemetry.addData("tx", tx);
                telemetry.addData("Servo Power", servoPower);
            } else {
                // No valid target — stop moving
                limelightmount.setPower(0);
                telemetry.addLine("No target detected");
            }
            telemetry.update();
        }
    }
}
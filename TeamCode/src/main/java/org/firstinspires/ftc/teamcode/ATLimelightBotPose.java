package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="Limelight Auto-Strafe", group="Linear OpMode")
public class ATLimelightBotPose extends LinearOpMode {

    private DcMotor fL, fR, bL, bR;
    private Limelight3A limelight;

    // proportional control coefficient (tune as needed)
    private final double kP = 0.7;

    // target position (meters, relative to field)
    private final double targetX = 1.2; // forward distance
    private final double targetY = -0.5; // strafe left = negative, right = positive

    @Override
    public void runOpMode() {

        fL = hardwareMap.get(DcMotor.class, "fL");
        bL = hardwareMap.get(DcMotor.class, "bL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bR = hardwareMap.get(DcMotor.class, "bR");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{fL, fR, bL, bR}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        limelight.pipelineSwitch(2);
        limelight.start();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result == null || result.getBotpose() == null) {
                telemetry.addLine("No valid Limelight pose!");
                telemetry.update();
                continue;
            }

            Pose3D botpose = result.getBotpose();

            double robotX = botpose.getPosition().x;
            double robotY = botpose.getPosition().y;

            double errorX = targetX - robotX; // forward/backward
            double errorY = targetY - robotY; // strafe

            // stop condition (within 5 cm tolerance)
            if (Math.abs(errorX) < 0.05 && Math.abs(errorY) < 0.05) {
                stopAllMotors();
                telemetry.addLine("Reached target!");
                telemetry.update();
                break;
            }

            // proportional movement toward target
            double axial = kP * errorX;    // forward/backward power
            double lateral = kP * errorY;  // strafe power
            double yaw = 0;                // no rotation

            // mecanum drive math
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // normalize powers
            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
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

            telemetry.addData("Target", "(%.2f, %.2f)", targetX, targetY);
            telemetry.addData("Current", "(%.2f, %.2f)", robotX, robotY);
            telemetry.addData("Error", "(%.2f, %.2f)", errorX, errorY);
            telemetry.update();
        }

        stopAllMotors();
    }

    private void stopAllMotors() {
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
}

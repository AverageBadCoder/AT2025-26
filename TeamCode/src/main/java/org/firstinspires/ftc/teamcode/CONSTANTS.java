package org.firstinspires.ftc.teamcode;
public class CONSTANTS {

//check commit!

    static final double fwSpeed = 1090;
    static final double autoFwSpeed = 1080;
    static final double intakeSpeed = 0.9;
    static final double dpadSpeed = 0.36;
    static final double wackUp = 0.48;
    static final double wackDown = 0.23;
    static final double llServoMin = 0.05;
    static final double llServoMax = 1;
//    static final double[] suzani = {0.1, 0.48, .83};
    static final double[] suzani = {0.19, 0.56, 0.93};

//    static final double[] suzano = {0.67, 1, 0.3};
    static final double[] suzano = {0.75, 0.01, 0.38};

    //    0.74  0.36  0
//    0.16  0.53  0.91
    static final double searchSpeed = 0.02;
    static double[] purpleBall = {145, 175, 250, 190};
    static double[] greenBall = {55, 142, 113, 103};
    static final double blueX = 1.0;
    static final double blueY = -0.5;
    static final double blueYaw = Math.toRadians(-69);
    static final double blueInX = 1.5;
    static final double blueInY = -1.5;
    static final double blueInYaw = Math.toRadians(90);

//    Limelight offsets
// Camera offset relative to robot CENTER, in meters
    static final double CAMERA_OFFSET_X = -0.026; // forward (+), backward (-)
    static final double CAMERA_OFFSET_Y = 0.163; // left (+), right (-)
    // Servo angle limits → convert servo position to angle

    static final double SERVO_CENTER_POS = 0.51;  // Forward = 0 rad
    static final double SERVO_MIN_POS = 0.15;
    static final double SERVO_MAX_POS = 0.80;

    // Total useful span (example): 260 degrees = 4.537 rad
    static final double SERVO_TOTAL_ANGLE = Math.toRadians(260);

//    AUTO
    static final double AutoSlow = 500;
    static final double AutoFast = 1200;
    static final double AutoTurnSlow = 600;
    static final double AutoTurnFast = 1000;
    static final double posTol = 1;
    static final double yawTol = Math.toRadians(1);
    static final double blueShootX = 6;
    static final double blueShootY = 2;
    static final double blueShootYaw = -65;
    static final double blueIntake1X = 29;
    static final double blueIntakeY = 8;
    static final double blueIntakeYaw = 90;
}

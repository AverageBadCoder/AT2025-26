package org.firstinspires.ftc.teamcode;
public class CONSTANTS {

//check commit!

    static final double fwSpeed = 1090;
    static final double intakeSpeed = 0.9;
    static final double dpadSpeed = 0.32;
    static final double wackUp = 0.43;
    static final double wackDown = 0.2;
    static final double llServoMin = 0.05;
    static final double llServoMax = 1;
//    static final double[] suzani = {0.1, 0.48, .83};
    static final double[] suzani = {0.19, 0.56, 0.93};

//    static final double[] suzano = {0.67, 1, 0.3};
    static final double[] suzano = {0.74, 0, 0.37};

    //    0.74  0.36  0
//    0.16  0.53  0.91
    static final double searchSpeed = 0.02;
    static double[] purpleBall = {145, 175, 250, 190};
    static double[] greenBall = {55, 142, 113, 103};
    static final double blueX = 1.3;
    static final double blueY = -0.5;
    static final double blueYaw = Math.toRadians(-71);
    static final double blueInX = 1.5;
    static final double blueInY = -1.5;
    static final double blueInYaw = Math.toRadians(90);

//    Limelight offsets
// Camera offset relative to robot CENTER, in meters
    static final double CAMERA_OFFSET_X = -0.02; // forward (+), backward (-)
    static final double CAMERA_OFFSET_Y = 0.165; // left (+), right (-)
    // Servo angle limits → convert servo position to angle
    static final double SERVO_CENTER_POS = 0.47;  // Forward = 0 rad
    static final double SERVO_MIN_POS = 0.15;
    static final double SERVO_MAX_POS = 0.80;

    // Total useful span (example): 260 degrees = 4.537 rad
    static final double SERVO_TOTAL_ANGLE = Math.toRadians(260);

//    AUTO
    static final double AutoSlow = 100;
    static final double AutoFast = 500;
    static final double blueShootX = 5;
    static final double blueShootY = 0;
    static final double blueShootYaw = -70;
    static final double blueIntake1X = 30;
    static final double blueIntakeY = 16;
    static final double blueIntakeYaw = 90;
}

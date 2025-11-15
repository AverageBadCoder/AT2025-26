
/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/*
 * This OpMode illustrates how to use a video source (camera) as a color sensor
 *
 * A "color sensor" will typically determine the color of the object that it is pointed at.
 *
 * This sample performs the same function, except it uses a video camera to inspect an object or scene.
 * The user may choose to inspect all, or just a Region of Interest (ROI), of the active camera view.
 * The user must also provide a list of "acceptable colors" (Swatches) from which the closest matching
 * color will be selected.
 *
 * To perform this function, a VisionPortal runs a PredominantColorProcessor process.
 *   The PredominantColorProcessor (PCP) process is created first, and then the VisionPortal is built.
 *   The (PCP) analyses the ROI and splits the colored pixels into several color-clusters.
 *   The largest of these clusters is then considered to be the "Predominant Color"
 *   The process then matches the Predominant Color with the closest Swatch and returns that match.
 *
 * To aid the user, a colored rectangle is drawn on the camera preview to show the RegionOfInterest,
 * The Predominant Color is used to paint the rectangle border, so the user can visualize the color.
 *
 * Tip:  Connect an HDMI monitor to the Control Hub to view the Color Sensor process in real-time.
 *       Or use a screen copy utility like ScrCpy.exe to view the video remotely.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Concept: Vision Color-Sensor", group = "Concept")
@Disabled
public class ColorSensorTestOpMode extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Read color values
            int red   = colorSensor.red();
            int green = colorSensor.green();
            int blue  = colorSensor.blue();
            int alpha = colorSensor.alpha();

//            // Optional: get distance if sensor supports it
//            double distanceCM = 0.0;
//            if (colorSensor instanceof DistanceSensor) {
//                distanceCM = ((DistanceSensor) colorSensor).getDistance(com.qualcomm.robotcore.hardware.DistanceUnit.CM);
//            }

            // Log values to telemetry
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.update();

            sleep(100); // update every 100ms
        }
    }
}

/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@Autonomous(name = "Red Wall Position with AprilTag")
public class RedWallPosition extends LinearOpMode {

    private final Robot robot = new Robot(this);
    
    // Vision Variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware and Vision
        robot.initializeMotors(hardwareMap);
        robot.setRunUsingEncoder();
        robot.initAprilTag();

        telemetry.addData("Status", "Initialized & Camera Streaming");
        telemetry.update();

        waitForStart();

        // 2. Initial Movement
        // Drive forward to get within sight of the tags
        robot.runDriveInstructions(0.5, 24, 24, 3.0); 

        // 3. Scan for AprilTag
        // Give the robot 3 seconds to try and find a tag
        long scanStartTime = System.currentTimeMillis();
        boolean tagFound = false;

        while (opModeIsActive() && (System.currentTimeMillis() - scanStartTime < 3000)) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                desiredTag = detections.get(0); // Grab the first tag seen
                tagFound = true;
                break; 
            }
            telemetry.addLine("Searching for AprilTag...");
            telemetry.update();
        }

        // 4. Conditional Logic based on what the camera saw
        if (tagFound) {
            telemetry.addData("Found Tag ID", desiredTag.id);
            // Example: If ID is 1, drive further left; if 2, stay center
            if (desiredTag.id == 1) {
                robot.runDriveInstructions(0.4, 5, -5, 2.0); // Adjust position
            }
        } else {
            telemetry.addLine("No tag found - using default path");
        }
        telemetry.update();

    // 5. Shoot Artifact 
    robot.setLauncherVelocity(0.26, 6);
    robot.transportTop.setPower(1);
    sleep(3000);
    robot.setLauncherVelocity(0.28, 6);
    robot.intake.setPower(1);
    robot.transportBottom.setPower(1);
    sleep(5000);
    robot.intake.setPower(0);
    robot.setTransportPower(0);
    robot.setLauncherPower(0);


    robot.rightStrafe();
    sleep(250);
    robot.stopMotors();
 
     sleep(1000);  // pause to display final telemetry message.
        // Cleanup
        robot.stopMotorEncoder();
        visionPortal.close(); // Shut down camera to save battery/CPU
    }
}

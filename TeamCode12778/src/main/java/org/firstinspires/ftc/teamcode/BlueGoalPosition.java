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

@Autonomous(name = "Blue Goal Position")
public class BlueGoalPosition extends LinearOpMode {

  private final Robot robot = new Robot(this);

  @Override
  public void runOpMode() {
    robot.initializeHardware(hardwareMap);
    robot.setRunUsingEncoder();

    // Wait for the game to start (driver presses START)
    waitForStart();

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    // S1: Backwards 35 Inches with 10 Sec timeout
    robot.runDriveInstructions(0.5, -35, -35, 10.0);

    // shoot artifact code here
    robot.setLauncherVelocity(0.231, 6);
    robot.transportTop.setPower(1);
    sleep(2000);
    robot.setLauncherVelocity(0.242, 6);
    robot.intake.setPower(1);
    robot.transportBottom.setPower(1);
    sleep(5000);
    robot.intake.setPower(0);
    robot.setTransportPower(0);
    robot.setLauncherPower(0);

    sleep(2500);
    robot.runDriveInstructions(0.6, -24, 24, 4.0);  // S2: Turn Left 12 Inches with 4 Sec timeout
    robot.runDriveInstructions(0.5, 10.5, 10.5, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
  }
}


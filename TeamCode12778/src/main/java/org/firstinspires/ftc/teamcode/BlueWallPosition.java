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

@Autonomous(name="Blue Wall Position")
public class BlueWallPosition extends LinearOpMode {
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.initializeMotors(hardwareMap);
        robot.setRunUsingEncoder();

        // Send telemetry message to indicate successful Encoder reset
        robot.logCurrentPosition(telemetry);

        // Wait for the game to start (driver presses START)
        waitForStart();

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
        robot.runDriveInstructions(this, 0.5, -65, -65, 5.0);
        robot.runDriveInstructions(this, 0.5, -12, 12, 5.0);// S1: Forward 72 Inches with 7 Sec timeout
        robot.runDriveInstructions(this,0.5,6,6,5.0);

        // shoot artifact code here
        robot.setLauncherPower(0.3);
        sleep(1000);
        robot.transportTop.setPower(1);
        sleep(1500);
        robot.transportTop.setPower(0);
        sleep(500);
        robot.intake.setPower(1);
        robot.transportBottom.setPower(1);
        robot.setLauncherPower(0.33);
        sleep(3000);
        robot.transportTop.setPower(1);
        sleep(3500);
        robot.intake.setPower(0);
        robot.setTransportPower(0);
        robot.setLauncherPower(0);

        robot.stopMotorEncoder();

        robot.leftStrafe();
        sleep(250);
        robot.stopMotors();

        robot.logPathCompleted(telemetry);
        sleep(1000);  // pause to display final telemetry message.
    }
}

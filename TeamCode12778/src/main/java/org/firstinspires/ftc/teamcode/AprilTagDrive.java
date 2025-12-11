package org.firstinspires.ftc.teamcode; // Updated package name

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name="AprilTagDrive")
// @Disabled // Remove this line to make it appear on the DS
public class AprilTagDrive extends LinearOpMode {
    // Define the desired distance here, or in your library
    final double DESIRED_DISTANCE = 12.0; // inches
    private static final int DESIRED_TAG_ID = -1; // -1 for any tag

    // Create instances of our new library classes
    private final Robot robot = new Robot(this);

    @Override public void runOpMode()
    {
        // Initialize the hardware using the library's method
        robot.initializeMotors(hardwareMap);

        // Initialize the Apriltag Detection process using the library's method
        // We pass the desired tag ID to the library config
        myNavigator.initAprilTag(hardwareMap, DESIRED_TAG_ID);

        waitForStart();

        while (opModeIsActive())
        {
            double drive = 0;
            double strafe = 0;
            double turn = 0;

            // Use the library method to find a target and update library variables
            myNavigator.checkForTarget(telemetry);

            // Check if we should activate auto-drive (e.g., left bumper pressed)
            if (myNavigator.targetFound && gamepad1.left_bumper) {
                // The navigator calculates *what* the robot should do
                myNavigator.calculateDriveSpeeds(DESIRED_DISTANCE);
                drive = myNavigator.drive;
                strafe = myNavigator.strafe;
                turn = myNavigator.turn;
            } else {
                // Manual Control (move this manual control to the RobotHardware class if desired)
                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce power for manual control
                strafe = gamepad1.left_stick_x   / 2.0;
                turn   = gamepad1.right_stick_x  / 2.0;
            }

            // Tell the RobotHardware library how to move the robot with the calculated speeds
            myRobot.moveRobot(drive, strafe, turn);

            // Update all telemetry generated within the navigator class
            myNavigator.updateTelemetry(telemetry);
            telemetry.update();
        }
    }
}


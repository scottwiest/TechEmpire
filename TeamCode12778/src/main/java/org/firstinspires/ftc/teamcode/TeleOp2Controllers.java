package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TeleOp2Controllers")
public class TeleOp2Controllers  extends LinearOpMode {
    private final Robot robot = new Robot();

    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        float axial;
        float lateral;
        float yaw;
        double max;

        robot.initializeMotors(hardwareMap);

        runtime = new ElapsedTime();
        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            axial = gamepad1.right_stick_y;
            lateral = gamepad1.right_stick_x;
            yaw = gamepad1.left_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = (axial - lateral) - yaw;
            leftBackPower = (axial - lateral) + yaw;
            rightBackPower = (axial + lateral) - yaw;
            // Normalize the values so no wheel power exceeds 50%
            // This ensures that the robot maintains the desired motion.
            max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (max > 1.0) {
                leftFrontPower = leftFrontPower / max;
                rightFrontPower = rightFrontPower / max;
                leftBackPower = leftBackPower / max;
                rightBackPower = rightBackPower / max;
            }
            // Send calculated power to wheels.
            robot.leftFrontMotor.setPower(leftFrontPower * 0.5);
            robot.rightFrontMotor.setPower(rightFrontPower * 0.5);
            robot.leftBackMotor.setPower(leftBackPower * 0.5);
            robot.rightBackMotor.setPower(rightBackPower * 0.5);
            if (gamepad2.a) {
                robot.setLauncherPower(0.35);
            }
            if (gamepad2.b) {
                robot.setLauncherPower(0.3);
            }
            if (gamepad2.y) {
                robot.setLauncherPower(0.25);
            }
            if (gamepad2.x) {
                robot.setLauncherPower(0);
            }
            if (gamepad2.dpadRightWasPressed()) {
                robot.intake.setPower(1);
                robot.transportBottom.setPower(1);
            }
            if (gamepad2.dpadRightWasReleased()) {
                robot.intake.setPower(0);
                robot.transportBottom.setPower(0);

             }
            if (gamepad2.dpadDownWasPressed()) {
                robot.intake.setPower(1);
                robot.setTransportPower(1);
            }
            if (gamepad2.dpadDownWasReleased()) {
                robot.intake.setPower(0);
                robot.setTransportPower(0);
            }
            if (gamepad2.dpadLeftWasPressed()) {
                robot.transportBottom.setPower(1);
            }
            if (gamepad2.dpadLeftWasReleased()) {
                robot.transportBottom.setPower(0);
            }
            if (gamepad2.dpadUpWasPressed()) {
                robot.transportTop.setPower(1);
            }
            if (gamepad2.dpadUpWasReleased()) {
                robot.transportTop.setPower(0);
            }
            if (gamepad2.rightBumperWasPressed()) {
                robot.transportTop.setPower(-0.5);
                robot.transportBottom.setPower(-0.5);
                robot.setLauncherPower(-0.2);
            }
            if (gamepad2.rightBumperWasReleased()) {
                robot.transportTop.setPower(0);
                robot.transportBottom.setPower(0);
                robot.setLauncherPower(0);
            }
            if (gamepad1.right_trigger > 0.5){
                robot.transportTop.setPower(-1);
                robot.transportBottom.setPower(-1);
                robot.setLauncherPower(-0.25);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
            telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
            telemetry.update();
        }
    }}

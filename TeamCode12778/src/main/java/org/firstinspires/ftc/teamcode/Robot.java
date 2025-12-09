package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {

  private final ElapsedTime runtime = new ElapsedTime();
  DcMotor leftFrontMotor;
  DcMotor rightFrontMotor;
  DcMotor leftBackMotor;
  DcMotor rightBackMotor;
  DcMotorEx leftLauncher;
  DcMotorEx rightLauncher;
  DcMotor intake;
  CRServo transportTop;
  CRServo transportBottom;

  LinearOpMode opMode;

  static final double COUNTS_PER_TIRE_REV = 537.7;
  static final double WHEEL_DIAMETER_INCHES = 4.0;
  static final double COUNTS_PER_INCH = (COUNTS_PER_TIRE_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

  static final double COUNTS_PER_LAUNCHER_REV = 28;

  public Robot(LinearOpMode opMode) {
    this.opMode = opMode;
  }

  public void initializeMotors(HardwareMap hardwareMap) {
    // Initialize the drive system variables.
    leftFrontMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    leftBackMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    rightBackMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
    leftLauncher = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftLauncher");
    rightLauncher = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightLauncher");
    intake = hardwareMap.get(DcMotor.class, "intake");
    transportTop = hardwareMap.get(CRServo.class, "transportTop");
    transportBottom = hardwareMap.get(CRServo.class, "transportBottom");

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
    rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
    leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
    leftLauncher.setDirection(DcMotor.Direction.REVERSE);
    rightLauncher.setDirection(DcMotor.Direction.FORWARD);
    transportBottom.setDirection(CRServo.Direction.FORWARD);
    transportTop.setDirection(CRServo.Direction.FORWARD);
  }

  public void setRunUsingEncoder() {
    leftFrontMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    rightFrontMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    leftBackMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    rightBackMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    leftLauncher.setMode(RunMode.STOP_AND_RESET_ENCODER);
    rightLauncher.setMode(RunMode.STOP_AND_RESET_ENCODER);

    leftFrontMotor.setMode(RunMode.RUN_USING_ENCODER);
    rightFrontMotor.setMode(RunMode.RUN_USING_ENCODER);
    leftBackMotor.setMode(RunMode.RUN_USING_ENCODER);
    rightBackMotor.setMode(RunMode.RUN_USING_ENCODER);
    leftLauncher.setMode(RunMode.RUN_USING_ENCODER);
    rightLauncher.setMode(RunMode.RUN_USING_ENCODER);
  }

  public void setLauncherPower(double power) {
    leftLauncher.setPower(power);
    rightLauncher.setPower(power);
  }

  public void setLauncherVelocity(double targetVelocityPower) {
    // Motor constants for goBILDA 5202
    final double MAX_MOTOR_TPS = 2800.0;

    leftLauncher.setMode(RunMode.STOP_AND_RESET_ENCODER);
    rightLauncher.setMode(RunMode.STOP_AND_RESET_ENCODER);

    setLauncherPower(0);

    leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    double targetVelocityTPS = (MAX_MOTOR_TPS * targetVelocityPower);

    leftLauncher.setVelocity(targetVelocityTPS);
    rightLauncher.setVelocity(targetVelocityTPS);

    // Dynamic Tolerance Parameters
    final double MIN_ABS_TOLERANCE = 5.0; // Always be within at least 5 TPS
    final double PERCENT_TOLERANCE = 0.01; // 1% of the target velocity

    // Calculate the tolerance required for this specific velocity
    double requiredTolerance = Math.max(MIN_ABS_TOLERANCE,
        Math.abs(targetVelocityTPS) * PERCENT_TOLERANCE);

    // Don't return until the velocity is reached for BOTH motors
    while (opMode.opModeIsActive()) {
      double leftError = Math.abs(leftLauncher.getVelocity() - targetVelocityTPS);
      double rightError = Math.abs(rightLauncher.getVelocity() - targetVelocityTPS);

      if (leftError <= requiredTolerance && rightError <= requiredTolerance) {
        break; // Exit the loop only when both motors are in tolerance
      }

      opMode.sleep(10);
    }
    opMode.sleep(100);
  }

  public void stopMotorEncoder() {
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void leftStrafe() {
    rightFrontMotor.setPower(1);
    leftFrontMotor.setPower(-1);
    rightBackMotor.setPower(-1);
    leftBackMotor.setPower(1);
  }

  public void rightStrafe() {
    rightFrontMotor.setPower(-1);
    leftFrontMotor.setPower(1);
    rightBackMotor.setPower(1);
    leftBackMotor.setPower(-1);
  }

  public void stopMotors() {
    rightFrontMotor.setPower(0);
    leftFrontMotor.setPower(0);
    rightBackMotor.setPower(0);
    leftBackMotor.setPower(0);
  }

  public void setTransportPower(double power) {
    transportBottom.setPower(power);
    transportTop.setPower(power);
  }

  /*
   *  Method to perform a relative move, based on encoder counts.
   *  Encoders are not reset as the move is based on the current position.
   *  Move will stop if any of three conditions occur:
   *  1) Move gets to the desired position
   *  2) Move runs out of time
   *  3) Driver stops the OpMode running.
   */
  public void runDriveInstructions(
      double speed,
      double leftInches,
      double rightInches,
      double timeoutS
  ) {
    int newLeftFrontTarget;
    int newRightFrontTarget;
    int newLeftBackTarget;
    int newRightBackTarget;

    // Ensure that the OpMode is still active
    if (opMode.opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      newLeftFrontTarget =
          leftFrontMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
      newRightFrontTarget =
          rightFrontMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
      newLeftBackTarget = leftBackMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
      newRightBackTarget =
          rightBackMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
      leftFrontMotor.setTargetPosition(newLeftFrontTarget);
      rightFrontMotor.setTargetPosition(newRightFrontTarget);
      leftBackMotor.setTargetPosition(newLeftBackTarget);
      rightBackMotor.setTargetPosition(newRightBackTarget);

      // Turn On RUN_TO_POSITION
      leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();
      leftFrontMotor.setPower(Math.abs(speed));
      rightFrontMotor.setPower(Math.abs(speed));
      leftBackMotor.setPower(Math.abs(speed));
      rightBackMotor.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both motors are running.
      // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
      // its target position, the motion will stop.  This is "safer" in the event that the robot will
      // always end the motion as soon as possible.
      // However, if you require that BOTH motors have finished their moves before the robot continues
      // onto the next step, use (isBusy() || isBusy()) in the loop test.
      while (opMode.opModeIsActive() &&
          (runtime.seconds() < timeoutS) &&
          (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy()
              && rightBackMotor.isBusy())) {
        opMode.sleep(10);
      }

      // Stop all motion;
      leftFrontMotor.setPower(0);
      rightFrontMotor.setPower(0);
      leftBackMotor.setPower(0);
      rightBackMotor.setPower(0);

      // Turn off RUN_TO_POSITION
      leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      opMode.sleep(100);   // pause after each move.
    }
  }
}

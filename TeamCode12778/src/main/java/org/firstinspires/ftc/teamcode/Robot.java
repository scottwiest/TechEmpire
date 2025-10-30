package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

  private final ElapsedTime runtime = new ElapsedTime();
  DcMotor leftFrontMotor;
  DcMotor rightFrontMotor;
  DcMotor leftBackMotor;
  DcMotor rightBackMotor;
  DcMotor leftLauncher;
  DcMotor rightLauncher;
  DcMotor intake;
  CRServo transportRight;
  CRServo transportLeft;

  static final double COUNTS_PER_MOTOR_REV = 538;
  static final double WHEEL_DIAMETER_INCHES = 4.0;
  static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

  public void initializeMotors(HardwareMap hardwareMap) {
    // Initialize the drive system variables.
    leftFrontMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    leftBackMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    rightBackMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
    leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
    rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
    intake = hardwareMap.get(DcMotor.class, "intake");
    transportRight = hardwareMap.get(CRServo.class, "transportRight");
    transportLeft = hardwareMap.get(CRServo.class, "transportLeft");

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
    rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
    leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
    leftLauncher.setDirection(DcMotor.Direction.REVERSE);
    rightLauncher.setDirection(DcMotor.Direction.FORWARD);
  }

  public void setRunUsingEncoder() {
    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void logCurrentPosition(Telemetry telemetry) {
    // Send telemetry message to indicate successful Encoder reset
    telemetry.addData("Starting at",  "%7d :%7d",
        leftFrontMotor.getCurrentPosition(),
        rightFrontMotor.getCurrentPosition(),
        leftBackMotor.getCurrentPosition(),
        rightBackMotor.getCurrentPosition(),
        leftLauncher.getCurrentPosition(),
        rightLauncher.getCurrentPosition());
    telemetry.update();
  }

  public void logPathCompleted(Telemetry telemetry) {
    telemetry.addData("Path", "Complete");
    telemetry.update();
  }

  public void setLauncherPower(double power) {
    leftLauncher.setPower(power);
    rightLauncher.setPower(power);
  }
  public void stopMotorEncoder() {
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }
  public void RightStrafe() {
    rightFrontMotor.setPower(1);
    leftFrontMotor.setPower(-1);
    rightBackMotor.setPower(-1);
    leftBackMotor.setPower(1);
  }
  public void LeftStrafe() {
    rightFrontMotor.setPower(-1);
    leftFrontMotor.setPower(1);
    rightBackMotor.setPower(1);
    leftBackMotor.setPower(-1);
  }

  /*
   *  Method to perform a relative move, based on encoder counts.
   *  Encoders are not reset as the move is based on the current position.
   *  Move will stop if any of three conditions occur:
   *  1) Move gets to the desired position
   *  2) Move runs out of time
   *  3) Driver stops the OpMode running.
   */
  public void setDriveInstructions(
          LinearOpMode opMode,
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
      newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
      newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
      newLeftBackTarget = leftBackMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
      newRightBackTarget = rightBackMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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
          (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy())) {

        // Display it for the driver.
        opMode.telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
        opMode.telemetry.addData("Currently at",  " at %7d :%7d",
            leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition());
        opMode.telemetry.update();
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

      opMode.sleep(250);   // optional pause after each move.
    }
  }
}

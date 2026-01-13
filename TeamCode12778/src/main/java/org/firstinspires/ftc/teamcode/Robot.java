package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.JavaUtil.createListWith;
import static org.firstinspires.ftc.robotcore.external.JavaUtil.maxOfList;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;

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
  WebcamName webcam;

  LinearOpMode opMode;

  static final double COUNTS_PER_TIRE_REV = 537.7;
  static final double WHEEL_DIAMETER_INCHES = 4.0;
  static final double COUNTS_PER_INCH = (COUNTS_PER_TIRE_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

  private static final boolean USE_WEBCAM = true;
  private static final int DESIRED_TAG_ID = -1;
  private VisionPortal visionPortal;
  private AprilTagProcessor aprilTag;
  private AprilTagDetection desiredTag = null;
  boolean targetFound = false;    // Set to true when an AprilTag target is detected
  double drive = 0;        // Desired forward power/speed (-1 to +1)
  double strafe = 0;        // Desired strafe power/speed (-1 to +1)
  double turn = 0;
  // Or add a constructor/method to set this

  public Robot(LinearOpMode opMode) {
    this.opMode = opMode;
  }

  public void initializeHardware(HardwareMap hardwareMap) {
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
    //webcam = hardwareMap.get(WebcamName.class, "Webcam");

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

  public void setLauncherVelocity(double targetVelocityPower, double timeout) {
    // Motor constants for goBILDA 5202
    final double MAX_MOTOR_TPS = 2800.0;

    runtime.reset();

    double targetVelocityTPS = (MAX_MOTOR_TPS * targetVelocityPower);

    leftLauncher.setVelocity(targetVelocityTPS);
    rightLauncher.setVelocity(targetVelocityTPS);

    // Dynamic Tolerance Parameters
    final double MIN_ABS_TOLERANCE = 5.0; // Always be within at least 5 TPS
    final double PERCENT_TOLERANCE = 0.01; // 1% of the target velocity

    // Calculate the tolerance required for this specific velocity
    double requiredTolerance = Math.max(MIN_ABS_TOLERANCE,
        abs(targetVelocityTPS) * PERCENT_TOLERANCE);

    // Don't return until the velocity is reached for BOTH motors
    while (opMode.opModeIsActive()) {
      double leftError = abs(leftLauncher.getVelocity() - targetVelocityTPS);
      double rightError = abs(rightLauncher.getVelocity() - targetVelocityTPS);

      if ((leftError <= requiredTolerance && rightError <= requiredTolerance)
              || runtime.seconds() > timeout) {
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
      leftFrontMotor.setPower(abs(speed));
      rightFrontMotor.setPower(abs(speed));
      leftBackMotor.setPower(abs(speed));
      rightBackMotor.setPower(abs(speed));

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

  public void moveRobot(float axial, float lateral, float yaw) {
    // Calculate wheel powers.
    double frontLeftPower = axial + lateral + yaw;
    double frontRightPower = axial - lateral - yaw;
    double backLeftPower = axial - lateral + yaw;
    double backRightPower = axial + lateral - yaw;

    // Normalize wheel powers to be less than 1.0
    double max = maxOfList(createListWith(
        abs(frontLeftPower), abs(frontRightPower), abs(backLeftPower), abs(backRightPower
        )));

    if (max > 1.0) {
      frontLeftPower /= max;
      frontRightPower /= max;
      backLeftPower /= max;
      backRightPower /= max;
    }

    double speedScaleFactor = 0.5;
    // Send powers to the wheels.
    leftFrontMotor.setPower(frontLeftPower * speedScaleFactor);
    rightFrontMotor.setPower(frontRightPower * speedScaleFactor);
    leftBackMotor.setPower(backLeftPower * speedScaleFactor);
    rightBackMotor.setPower(backRightPower * speedScaleFactor);
  }

  public void initializeAprilTag() {
    // Create the AprilTag processor by using a builder.
    aprilTag = new AprilTagProcessor.Builder()
        .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .build();

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // e.g. Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    aprilTag.setDecimation(2);

    // Create the vision portal by using a builder.
    if (USE_WEBCAM) {
      visionPortal = new VisionPortal.Builder()
          .setCamera(webcam)
          .addProcessor(aprilTag)
          .build();
    } else {
      visionPortal = new VisionPortal.Builder()
          .setCamera(BuiltinCameraDirection.BACK)
          .addProcessor(aprilTag)
          .build();
    }
  }
  public void alignToAprilTag() {

  }

  private void setManualExposure(int exposureMS, int gain) {
    // Wait for the camera to be open, then use the controls

    if (visionPortal == null) {
      return;
    }

    // Make sure camera is streaming before we try to set the exposure controls
    if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
      opMode.telemetry.addData("Camera", "Waiting");
      opMode.telemetry.update();
      while (!opMode.isStopRequested() && (visionPortal.getCameraState()
          != VisionPortal.CameraState.STREAMING)) {
        opMode.sleep(20);
      }
      opMode.telemetry.addData("Camera", "Ready");
      opMode.telemetry.update();
    }

    // Set camera controls unless we are stopping.
    if (!opMode.isStopRequested()) {
      ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
      if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
        exposureControl.setMode(ExposureControl.Mode.Manual);
        opMode.sleep(50);
      }
      exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
      opMode.sleep(20);
      GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
      gainControl.setGain(gain);
      opMode.sleep(20);
    }
  }

  public void checkForTarget() {
    targetFound = false;
    desiredTag = null;

    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    for (AprilTagDetection detection : currentDetections) {
      // Look to see if we have size info on this tag.
      if (detection.metadata != null) {
        // Check to see if we want to track towards this tag.
        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
          targetFound = true;
          desiredTag = detection;
          break;  // don't look any further.
        } else {
          // This tag is in the library, but we do not want to track it right now.
          // Use the passed-in telemetry object
          opMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
        }
      } else {
        // This tag is NOT in the library, so we don't have enough information to track to it.
        // Use the passed-in telemetry object
        opMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
      }
    }

    // Add display messages using a separate updateTelemetry method if you prefer,
    // or keep them here as they are tightly coupled to the detection step.

    // Tell the driver what we see, and what to do.
    if (targetFound) {
      opMode.telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
      opMode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
      opMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
      opMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
      opMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
    } else {
      opMode.telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
    }
    opMode.telemetry.update();
  }

  public void updateTelemetry() {
    opMode.telemetry.addData("Target Found", desiredTag != null ? "Yes" : "No");
    opMode.telemetry.addData("Drive/Strafe/Turn", "%.2f, %.2f, %.2f", drive, strafe, turn);

    if (desiredTag != null) {
      opMode.telemetry.addData("ID", desiredTag.id);
      opMode.telemetry.addData("Range (in)", "%.2f", desiredTag.ftcPose.range);
      opMode.telemetry.addData("Bearing (deg)", "%.2f", desiredTag.ftcPose.bearing);
      opMode.telemetry.addData("Yaw (deg)", "%.2f", desiredTag.ftcPose.yaw);
    } else {
      opMode.telemetry.addLine("Manual Control Active");
    }
    opMode.telemetry.update();
  }

  // Constants for tuning
  final double SPEED_GAIN  =  0.02;   // How fast the robot turns (Adjust this!)
  final double HEADING_THRESHOLD = 1.0; // Stop turning if within 1 degree

  public void alignToTag(int targetID) {
    boolean aligned = false;

    while (opMode.opModeIsActive() && !aligned) {
      List<AprilTagDetection> currentDetections = aprilTag.getDetections();
      AprilTagDetection targetTag = null;

      // Find the specific tag we are looking for
      for (AprilTagDetection detection : currentDetections) {
        if (detection.metadata != null && detection.id == targetID) {
          targetTag = detection;
          break;
        }
      }

      if (targetTag != null) {
        double bearing = targetTag.ftcPose.bearing;

        if (Math.abs(bearing) <= HEADING_THRESHOLD) {
          // We are centered! Stop the motors.
          stopMotors();
          aligned = true;
        } else {
          // Calculate turn power based on bearing
          // If bearing is positive, tag is to the right, so turn right
          double turnPower = bearing * SPEED_GAIN;

          // Limit max power so it doesn't spin out of control
          turnPower = Range.clip(turnPower, -0.3, 0.3);

          moveRobot(0, (float)turnPower, 0);
        }
      } else {
        // Tag not found - stop or rotate slowly to search
        stopMotors();
      }
    }
  }
}



package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseTeleop {

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;
  double headingOffset;

  Gamepad currentGamepad1 = new Gamepad();
  Gamepad currentGamepad2 = new Gamepad();
  Gamepad lastGamepad1 = new Gamepad();
  Gamepad lastGamepad2 = new Gamepad();

  double distance;
  double power;

  LLResult currentTagResult;
  private boolean autoCalculateShootPower = true;

  public BaseTeleop(LinearOpMode opMode, Robot robot, double headingOffset) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
    this.headingOffset = Math.toRadians(headingOffset);
  }

  private void updateGamepads() {
    lastGamepad1.copy(currentGamepad1);
    currentGamepad1.copy(this.opMode.gamepad1);

    lastGamepad2.copy(currentGamepad2);
    currentGamepad2.copy(this.opMode.gamepad2);
  }

  public void run() {
    // --- INIT ---

    // --- INIT LOOP ---
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE COLOR", robot.getAllianceColor());
      telemetry.update();
    }

    // --- START ---
    robot.stateTimer.reset();
    while (opMode.opModeIsActive()) {
      updateGamepads();
      robot.intake.updateSampleColor();

      // DRIVETRAIN
      double x = currentGamepad1.left_stick_x;
      double y = -currentGamepad1.left_stick_y;
      double rx;
      if (currentGamepad1.right_bumper) {
        currentTagResult = robot.limelight.updateGoal(); // only poll the limelight when trying to auto aim
        rx = robot.limelight.updateAimPID(); // auto aim
      } else {
        rx = currentGamepad1.right_stick_x; // normal drive
      }
      this.fieldCentricDrive(x, y, rx);

      // OUTTAKE
      if (currentGamepad2.triangle) {
        robot.outtake.setShoot();
      } else {
        robot.outtake.setBase();
      }

      if (currentGamepad2.right_stick_button) {
        currentGamepad2.setLedColor(0, 1, 0, 2000);
        currentGamepad2.rumble(500);
        autoCalculateShootPower = true;
      } else if (currentGamepad2.left_stick_button) {
        currentGamepad2.setLedColor(1, 1, 0, 2000);
        currentGamepad2.rumble(500);
        autoCalculateShootPower = false;
      }

      if (currentGamepad1.dpad_down) {
        robot.outtake.stop();
        this.autoCalculateShootPower = false; // don't continue calculating and setting target if stopping
      } else if (autoCalculateShootPower) {
        if (Math.abs(currentGamepad1.left_stick_x) <= .05 && Math.abs(currentGamepad1.left_stick_y) <= .05) {
          robot.outtake.setTargetVelocity(robot.limelight.calculateTargetVelocity());
        }
      } else {
        if (currentGamepad2.dpad_up) {
          robot.outtake.setTargetVelocity(Outtake.farSpeed);
        } else if (currentGamepad2.dpad_down) {
          robot.outtake.setTargetVelocity(Outtake.medSpeed);
        } else if (currentGamepad2.dpad_right) {
          robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
        } else if (currentGamepad2.dpad_left) {
          robot.outtake.setTargetVelocity(robot.limelight.calculateTargetVelocity());
        }
      }
      robot.outtake.updatePIDControl();

      // INTAKE
      if (currentGamepad1.right_trigger > 0.05 || currentGamepad1.left_trigger > 0.05) {
        robot.intake.setPower(currentGamepad1.right_trigger - currentGamepad1.left_trigger);
      } else if (currentGamepad2.right_trigger > 0.05 || currentGamepad2.left_trigger > 0.05) {
        robot.intake.setPower(currentGamepad2.right_trigger - currentGamepad2.left_trigger);
      } else if (currentGamepad2.left_bumper) {
        robot.intake.setPowerVertical(-.6);
      } else {
        robot.intake.setPowerVertical(0);
      }

      // TELEMETRY
      updateTelemetry();
    }
  }

  private void fieldCentricDrive(double x, double y, double rx) {
    if (currentGamepad1.left_bumper) {
      robot.imu.resetYaw();
      this.headingOffset = 0;
    }

    // Field Centric Drive
    double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + this.headingOffset;

    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
    rotX = rotX * 1.1;  // Counteract imperfect strafing

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower = (rotY + rotX - rx) / denominator;

    robot.fr.setPower(frontRightPower);
    robot.fl.setPower(frontLeftPower);
    robot.br.setPower(backRightPower);
    robot.bl.setPower(backLeftPower);
  }

  private void updateTelemetry() {
    // TODO: do not re-read sensor values for telemetry. Cache values from updateSampleColor, etc if necessary!!
    telemetry.addData("Vel Current", robot.outtake.getCurrentVelocity()); // TODO: this makes a control hub call
    telemetry.addData("Vel Target", robot.outtake.getTargetVelocity());
    telemetry.addData("At Target", robot.outtake.atTarget());
    telemetry.addData("Distance Estimate", distance);
    telemetry.addData("Power Estimate", power);
    telemetry.addLine("=== SENSORS ===");
    NormalizedRGBA cs1Colors = robot.intake.cs1.getNormalizedColors(); // TODO: this makes a control hub call
    NormalizedRGBA cs2Colors = robot.intake.cs2.getNormalizedColors(); // TODO: this makes a control hub call
    telemetry.addData("Red 1", cs1Colors.red);
    telemetry.addData("Green 1", cs1Colors.green);
    telemetry.addData("Dist CM 1", robot.intake.readDistance(robot.intake.cs1)); // TODO: this makes a control hub call
    telemetry.addData("Red 2", cs2Colors.red);
    telemetry.addData("Green 2", cs2Colors.green);
    telemetry.addData("Dist CM 2", robot.intake.readDistance(robot.intake.cs2)); // TODO: this makes a control hub call

    telemetry.update();
  }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    while (opMode.opModeIsActive()) {
      updateGamepads();
      robot.intake.updateSampleColor();
      // currentTagResult = robot.limelight.updateGoal();

      // DRIVETRAIN
      double x = currentGamepad1.left_stick_x;
      double y = -currentGamepad1.left_stick_y;
      float rotStickAvg = currentGamepad1.right_stick_x + currentGamepad2.right_stick_x;
      double rx;
      if (currentGamepad1.right_bumper) {
        robot.limelight.updateGoal();
        rx = robot.limelight.updateAimPID(
            rotStickAvg); // auto aim
      } else {
        rx = rotStickAvg; // normal drive // TODO: Test this
      }
      this.fieldCentricDrive(x, y, rx);

      // OUTTAKE
      if (currentGamepad2.triangle) {
        robot.outtake.setShoot();
      } else if (currentGamepad2.square) {
        robot.outtake.setCycle();
      } else {
        robot.outtake.setBase();
      }

      if (currentGamepad2.right_stick_button) {
        currentGamepad2.rumble(500);
        autoCalculateShootPower = true;
      } else if (currentGamepad2.left_stick_button) {
        currentGamepad2.rumble(500);
        autoCalculateShootPower = false;
      }

      if (currentGamepad1.dpad_down) {
        robot.outtake.stop();
        this.autoCalculateShootPower = false; // don't continue calculating and setting target if stopping
      } else if (autoCalculateShootPower) {
        if (currentGamepad1.right_bumper) {
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
          robot.limelight.updateGoal();
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

    telemetry.addData("Vel Current", robot.outtake.getCurrentVelocity());
    telemetry.addData("Vel Target", robot.outtake.getTargetVelocity());
    telemetry.addData("At Target", robot.outtake.atTarget());

    telemetry.update();
  }
}

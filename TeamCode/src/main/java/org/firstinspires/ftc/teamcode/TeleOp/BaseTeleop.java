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


  Gamepad gamepad1 = new Gamepad();
  Gamepad gamepad2 = new Gamepad();

  double distance;
  double power;

  LLResult currentTagResult;
  private boolean autoAiming = true;

  public BaseTeleop(LinearOpMode opMode, Robot robot, double headingOffset) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
    this.headingOffset = Math.toRadians(headingOffset);
  }

  private void updateGamepads() {
    gamepad1.copy(this.opMode.gamepad1);
    gamepad2.copy(this.opMode.gamepad2);
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
      currentTagResult = robot.limelight.updateGoal();
      updateGamepads();
      robot.intake.updateSampleColor();

      if (gamepad1.right_bumper) {
        // autoaim
        fieldCentricDriveAim();
      } else {
        // default drive
        this.fieldCentricDrive();
      }

      if (gamepad2.triangle) {
        robot.outtake.setShoot();
      } else {
        robot.outtake.setBase();
      }
      if (gamepad2.dpad_up && !autoAiming) {
        robot.outtake.setTargetVelocity(Outtake.farSpeed);
      }
      if (gamepad2.dpad_down && !autoAiming) {
        robot.outtake.setTargetVelocity(Outtake.medSpeed);
      }
      if (gamepad2.dpad_right && !autoAiming) {
        robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
      }

      if (gamepad2.dpad_left && !autoAiming) {
        robot.outtake.setPower(robot.limelight.getShooterPower());
      }

      if (gamepad2.right_stick_button) { // set to aiming
        gamepad2.setLedColor(0, 1, 0, 2000);
        gamepad2.rumble(500);
        autoAiming = true;
      }
      if (gamepad2.left_stick_button) { // set to not aiming
        gamepad2.setLedColor(1, 1, 0, 2000);
        gamepad2.rumble(500);
        autoAiming = false;
      }

      if (autoAiming) {
        if (Math.abs(gamepad1.left_stick_x) <= .05 || Math.abs(gamepad1.right_stick_x) <= .05) {
          robot.outtake.setPower(robot.limelight.getShooterPower());
        }
      }

      if (gamepad1.dpad_down) {
        robot.outtake.stop();
      }
      // INTAKE
      if (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0) {
        robot.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
      } else if (gamepad2.right_trigger != 0 || gamepad2.left_trigger != 0) {
        robot.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
      } else if (gamepad2.left_bumper) {
        robot.intake.setPowerVertical(-.6);
      } else {

        robot.intake.setPowerVertical(0);

      }

      robot.outtake.updatePIDControl();

      // TELEMETRY
      updateTelemetry();
    }
  }

  private void fieldCentricDrive() {
    if (gamepad1.left_bumper) {
      robot.imu.resetYaw();
      this.headingOffset = 0;
    }
    // Field Centric Drive
    double y = -gamepad1.left_stick_y;
    double x = gamepad1.left_stick_x;
    double rx = gamepad1.right_stick_x;

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

  private void fieldCentricDriveAim() {
    double turnPower = robot.limelight.updateAimPID();

    // Inject PID turn power into field-centric drive
    double y = -gamepad1.left_stick_y;
    double x = gamepad1.left_stick_x;
    double rx = turnPower;   // <-- PID replaces right stick turning

    // run drive using rx as rotation
    double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + this.headingOffset;

    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
    rotX *= 1.1;

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double fl = (rotY + rotX + rx) / denominator;
    double bl = (rotY - rotX + rx) / denominator;
    double fr = (rotY - rotX - rx) / denominator;
    double br = (rotY + rotX - rx) / denominator;

    robot.fr.setPower(fr);
    robot.fl.setPower(fl);
    robot.br.setPower(br);
    robot.bl.setPower(bl);
  }


  private void updateTelemetry() {
    // TODO: do not re-read sensor values for telemetry. Cache values from updateSampleColor, etc if necessary
    telemetry.addData("Vel Current", robot.outtake.getCurrentVelocity());
    telemetry.addData("Vel Target", robot.outtake.getTargetVelocity());
    telemetry.addData("At Target", robot.outtake.atTarget());
    telemetry.addData("Distance Estimate", distance);
    telemetry.addData("Power Esstimeate", power);
    telemetry.addLine("=== SENSORS ===");
    NormalizedRGBA cs1Colors = robot.intake.cs1.getNormalizedColors();
    NormalizedRGBA cs2Colors = robot.intake.cs2.getNormalizedColors();
    telemetry.addData("Red 1", cs1Colors.red);
    telemetry.addData("Green 1", cs1Colors.green);
    telemetry.addData("Dist CM 1", robot.intake.readDistance(robot.intake.cs1));
    telemetry.addData("Red 2", cs2Colors.red);
    telemetry.addData("Green 2", cs2Colors.green);
    telemetry.addData("Dist CM 2", robot.intake.readDistance(robot.intake.cs2));

    telemetry.update();
  }
}

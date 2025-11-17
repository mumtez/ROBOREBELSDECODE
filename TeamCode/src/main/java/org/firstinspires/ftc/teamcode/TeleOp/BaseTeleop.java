package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
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

  private final ElapsedTime aimTimer = new ElapsedTime();

  public static double aimKp = 0.018;
  public static double aimKi = 0.0;
  public static double aimKd = 0.001;

  private double aimIntegral = 0;
  private double aimLastError = 0;

  Gamepad gamepad1 = new Gamepad();
  Gamepad gamepad2 = new Gamepad();

  double distance;
  double power;

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
    robot.limelight.pipelineSwitch(this.robot.getAllianceColor().getLLPipelineTeleOP());
    robot.limelight.setPollRateHz(100);
    robot.limelight.start();

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
      if (gamepad2.dpad_up) {
        robot.outtake.setTargetVelocity(Outtake.farSpeed);
      }
      if (gamepad2.dpad_down) {
        robot.outtake.setTargetVelocity(Outtake.medSpeed);
      }
      if (gamepad2.dpad_right) {
        robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
      }

      if (gamepad2.dpad_left) {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
          distance = (((41.275) / Math.tan((Math.toRadians(result.getTy() + 1.0)))) / 100.0);
          power = (distance * Math.pow(0.243301244553 * distance - 0.173469387755, -0.5)) / 0.0025344670037;
          robot.outtake.setTargetVelocity(power);
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
    LLResult result = robot.limelight.getLatestResult();
    if (result != null && result.isValid()) {
      double tx = result.getTx();
      double turnPower = updateAimPID(tx);

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
  }

  public double updateAimPID(double tx) {

    double dt = aimTimer.seconds();
    aimTimer.reset();

    double error = tx - robot.getAllianceColor().getAimPose();

    // Integral
    aimIntegral += error * dt;

    // Derivative
    double derivative = (error - aimLastError) / dt;
    aimLastError = error;

    // PID Output
    double output = aimKp * error
        + aimKi * aimIntegral
        + aimKd * derivative;

    // Clamp for safety
    output = Math.max(-1, Math.min(1, output));

    return output;   // return turn power
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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
public class BaseTeleop {

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;
  double headingOffset;


  Gamepad gamepad1 = new Gamepad();
  Gamepad gamepad2 = new Gamepad();

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
      updateGamepads();
      robot.updateBall();
      robot.intake.updateSampleColor();

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

      // INTAKE
      robot.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

      // SHOOTER WITH NO AUTOMATED METHODS
      /*
      if (gamepad2.a) {
        robot.outtake.setTargetVelocity(Outtake.medSpeed);
      } else if (gamepad2.dpad_down) {
        robot.outtake.setTargetVelocity(150);
      } else if (gamepad2.b) {
        robot.outtake.setTargetVelocity(0);
      } */

      if (gamepad2.triangle) {
        robot.outtake.setShoot();
      }

      robot.outtake.updatePIDControl();

      // TELEMETRY
      updateTelemetry();
    }
  }

  public void updateTelemetry() {
    telemetry.addData("Vel Current", robot.outtake.getCurrentVelocity());
    telemetry.addData("Vel Target", robot.outtake.getTargetVelocity());
    telemetry.addData("Colors", robot.intake.getColors());

    telemetry.update();
  }
}

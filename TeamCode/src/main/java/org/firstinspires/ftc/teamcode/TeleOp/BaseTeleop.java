package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class BaseTeleop {
  public enum ModeState {
    INTAKING, SHOOTER_SPINUP, READY
  }
  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;
  double headingOffset;
  final ElapsedTime stateTimer = new ElapsedTime();



  Gamepad gamepad1 = new Gamepad();
  Gamepad gamepad2 = new Gamepad();
  public BaseTeleop(LinearOpMode opMode, Robot robot, double headingOffset) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
    this.headingOffset = Math.toRadians(headingOffset);
  }

  public void run() {
    // --- INIT ---

    // --- INIT LOOP ---
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE COLOR", robot.getAllianceColor());
      telemetry.update();
    }
    // --- START ---
    stateTimer.reset();

    while (opMode.opModeIsActive()) {

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

      updateTelemetry();
    }
  }
  public void updateTelemetry() {

    telemetry.addData("Vel Current", robot.outtake.getCurrentVelocity());
    telemetry.addData("Vel Target", robot.outtake.getTargetVelocity());

    telemetry.update();
  }
}

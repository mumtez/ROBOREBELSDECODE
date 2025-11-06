package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "FLYWHEEL PID TESTING", group = "TESTING")
@Configurable
public class FlywheelPIDTesting extends LinearOpMode {

  private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
  public static int TARGET = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    // use dashboard telemetry

    Robot robot = new Robot(this);
    waitForStart();
    while (opModeIsActive()) {
      if (gamepad1.square) {
        robot.outtake.setTargetVelocity(TARGET);
      }

      double pow = robot.outtake.updatePIDControl();
      panelsTelemetry.update(telemetry);

      telemetry.addData("TARGET", TARGET);
      telemetry.addData("REFERENCE", robot.outtake.getCurrentVelocity());
      telemetry.addData("POWER", pow);
      telemetry.update();
    }

  }

}

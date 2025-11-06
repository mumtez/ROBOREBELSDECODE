package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "SENSOR TEST", group = "TESTING")
public class SensorTest extends LinearOpMode {


  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this);
    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("INTAKE COLOR", robot.intake.updateSampleColor());
      telemetry.update();
    }
  }
}

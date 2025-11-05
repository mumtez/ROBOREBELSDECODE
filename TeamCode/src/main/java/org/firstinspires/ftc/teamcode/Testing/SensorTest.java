package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "SENSOR TEST", group = "TESTING")
public class SensorTest extends LinearOpMode {

  Robot robot = new Robot(this);

  @Override
  public void runOpMode() throws InterruptedException {
    while (opModeIsActive()) {
      telemetry.addData("INTAKE COLOR RED", robot.intake.getColors().red);
      telemetry.addData("INTAKE COLOR GREEN", robot.intake.getColors().green);
      telemetry.addData("INTAKE COLOR BLUE", robot.intake.getColors().blue);
      telemetry.addData("INTAKE COLOR ALPHA", robot.intake.getColors().alpha);
      telemetry.update();
    }
  }
}

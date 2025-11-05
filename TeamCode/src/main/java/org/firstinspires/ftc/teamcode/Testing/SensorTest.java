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
      telemetry.addData("INTAKE COLOR RED", robot.intake.colorSensor.getNormalizedColors().red);
      telemetry.addData("INTAKE COLOR GREEN", robot.intake.colorSensor.getNormalizedColors().green);
      telemetry.addData("INTAKE COLOR BLUE", robot.intake.colorSensor.getNormalizedColors().blue);
      telemetry.addData("INTAKE COLOR ALPHA", robot.intake.colorSensor.getNormalizedColors().alpha);
      telemetry.update();
    }
  }
}

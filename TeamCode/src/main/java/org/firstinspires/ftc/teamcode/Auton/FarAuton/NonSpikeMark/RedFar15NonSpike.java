package org.firstinspires.ftc.teamcode.Auton.FarAuton.NonSpikeMark;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "RED FAR NO SPIKE", group = "RED")
public class RedFar15NonSpike extends LinearOpMode {

  public static double[] SHOOT_RED = {90, 12, 67};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED);
    new BaseFar15NonSpike(this, robot, SHOOT_RED).run();
  }
}

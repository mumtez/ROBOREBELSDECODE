package org.firstinspires.ftc.teamcode.Auton.FarAuton.NonSpikeMark;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "BLUE FAR NO SPIKE", group = "BLUE")
public class BlueFar15NonSpike extends LinearOpMode {

  public static double[] SHOOT_BLUE = {66, 12, 100};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    new BaseFar15NonSpike(this, robot, SHOOT_BLUE).run();
  }
}

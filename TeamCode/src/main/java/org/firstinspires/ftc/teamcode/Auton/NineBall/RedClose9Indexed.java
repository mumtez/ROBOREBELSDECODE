package org.firstinspires.ftc.teamcode.Auton.NineBall;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "RED CLOSE 9", group = "RED")
public class RedClose9Indexed extends LinearOpMode {

  public static double[] SHOOT_RED = {78, 86, 41};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED);
    new BaseClose9Indexed(this, robot, SHOOT_RED).run();
  }
}

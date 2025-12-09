package org.firstinspires.ftc.teamcode.Auton.NineBall;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "BLUE CLOSE 9", group = "BLUE")
public class BlueClose9Indexed extends LinearOpMode {

  public static double[] SHOOT_BLUE = {66, 86, 134};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    new BaseClose9Indexed(this, robot, SHOOT_BLUE).run();
  }
}

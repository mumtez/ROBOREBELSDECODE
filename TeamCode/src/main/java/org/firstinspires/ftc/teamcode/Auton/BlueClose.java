package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
@Configurable

@Autonomous(name = "BLUE CLOSE", group = "BLUE")
public class BlueClose extends LinearOpMode {
  public static double[] SHOOT_BLUE = {64, 88, 134};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    new BaseCloseAuto(this, robot, SHOOT_BLUE).run();
  }
}

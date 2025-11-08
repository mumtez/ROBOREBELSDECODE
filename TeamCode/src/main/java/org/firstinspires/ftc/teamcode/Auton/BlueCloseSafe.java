package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "BLUE CLOSE SAFE", group = "BLUE")
public class BlueCloseSafe extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    new BaseCloseAutoSafeForBlue(this, robot).run();
  }
}

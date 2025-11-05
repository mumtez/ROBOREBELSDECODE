package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@TeleOp(name = "SERVO TESTING", group = "TESTING")
public class ServoTest extends LinearOpMode {

  Robot robot;
  public static double SERVO_TEST_POS = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    waitForStart();
    // LOOP
    while (opModeIsActive()) {
      robot.outtake.setServoPos(SERVO_TEST_POS);
    }
  }
}

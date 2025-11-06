package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
@TeleOp(name = "SERVO TESTING", group = "TESTING")
public class ServoTest extends LinearOpMode {

  Robot robot;
  public static double SERVO_TEST_POS = Outtake.SHOOT_POS;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    waitForStart();
    // LOOP
    while (opModeIsActive()) {
      robot.outtake.setServoPos(SERVO_TEST_POS);

      if (gamepad1.a) {
        robot.fl.setPower(1);
      } else {
        robot.fl.setPower(0);
      }
      if (gamepad1.b) {
        robot.bl.setPower(1);
      } else {
        robot.bl.setPower(0);
      }
      if (gamepad1.x) {
        robot.fr.setPower(1);
      } else {
        robot.fr.setPower(0);
      }
      if (gamepad1.y) {
        robot.br.setPower(1);
      } else {
        robot.br.setPower(0);
      }

    }
  }
}

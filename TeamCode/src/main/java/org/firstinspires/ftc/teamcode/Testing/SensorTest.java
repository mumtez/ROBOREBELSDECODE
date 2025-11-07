package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "SENSOR TEST", group = "TESTING")
public class SensorTest extends LinearOpMode {


  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this);
    waitForStart();
    while (opModeIsActive()) {
      LLResult result = robot.limelight.getLatestResult(); //TODO demo limelight code actually have to put real logic
      List<FiducialResult> fiducials = result.getFiducialResults();
      if (result != null) {
        if (result.isValid()) {
          Pose3D botpose = result.getBotpose();
          telemetry.addData("tx", result.getTx());
          telemetry.addData("ty", result.getTy());
          telemetry.addData("Botpose", botpose.toString());
          for (FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            telemetry.addData("Tag ", id);
          }
        }
      }

      telemetry.addData("INTAKE COLOR", robot.intake.updateSampleColor());
      telemetry.update();
    }
  }
}

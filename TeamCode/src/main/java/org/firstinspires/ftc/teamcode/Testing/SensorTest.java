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

  public double llHeight = 33.5;
  public double goalHeight = 75;
  public double llDegrees = 0;


  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this);
    waitForStart();
    robot.limelight.pipelineSwitch(1);
    robot.limelight.start();
    while (opModeIsActive()) {
      LLResult result = robot.limelight.getLatestResult(); //TODO demo limelight code actually have to put real logic
      List<FiducialResult> fiducials = result.getFiducialResults();
      if (result != null) {
        if (result.isValid()) {
          Pose3D botpose = result.getBotpose();
          telemetry.addData("tx", result.getTx());
          telemetry.addData("ty", result.getTy());
          telemetry.addData("area", result.getTa());
          telemetry.addData("distance ares est", (179.44107 * Math.pow(result.getTa(), -0.495624)));
          double angleToGoalRadians = Math.PI * ((result.getTy()) + 1) / 180;
          double distanceFromLimelightToGoal = (goalHeight - llHeight) / Math.tan(angleToGoalRadians);
          telemetry.addData("distance tangent est", distanceFromLimelightToGoal);

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

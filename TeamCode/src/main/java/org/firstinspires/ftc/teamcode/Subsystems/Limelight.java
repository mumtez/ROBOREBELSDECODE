package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import org.firstinspires.ftc.teamcode.AllianceColor;

public class Limelight {


  public Limelight3A limelight;

  AllianceColor currentColor;

  public Limelight(LinearOpMode opMode, AllianceColor color) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    currentColor = color;
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    this.limelight.start();
  }

  public LLResult getGoalResult() {
    this.limelight.pipelineSwitch(currentColor.getLLPipelineTeleOP());
    return this.limelight.getLatestResult();
  }

  public int getPatternIdAuto() {
    this.limelight.pipelineSwitch(0);
    LLResult result = this.limelight.getLatestResult();
    List<FiducialResult> fiducials = result.getFiducialResults();
    if (result != null && result.isValid()) {
      for (FiducialResult fiducial : fiducials) {
        int id = fiducial.getFiducialId(); // The ID number of the fiducial
        return id;
      }
    }
    return 21;
  }


}

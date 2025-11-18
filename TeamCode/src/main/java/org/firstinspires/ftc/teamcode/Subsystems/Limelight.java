package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.teamcode.AllianceColor;

@Configurable
public class Limelight {


  public Limelight3A limelight;

  AllianceColor currentColor;

  public LLResult currentGoal;

  public double lastPower;

  private final ElapsedTime aimTimer = new ElapsedTime();

  public static double aimKp = 0.018;
  public static double aimKi = 0.0;
  public static double aimKd = 0.001;

  private double aimIntegral = 0;
  private double aimLastError = 0;

  public Limelight(LinearOpMode opMode, AllianceColor color) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    currentColor = color;
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    this.limelight.start();
  }


  public LLResult updateGoalTeleop() {
    this.limelight.pipelineSwitch(currentColor.getLLPipelineTeleOP());
    currentGoal = this.limelight.getLatestResult();
    return currentGoal;
  }

  public double getShooterPower() {
    double distance;
    double power;
    if (currentGoal != null && currentGoal.isValid()) {
      distance = (((41.275) / Math.tan((Math.toRadians(currentGoal.getTy() + 1.0)))) / 100.0);
      power = (distance * Math.pow(0.243301244553 * distance - 0.173469387755, -0.5)) / 0.0025344670037;
      lastPower = power;
      return power;
    }
    return lastPower;
  }

  public double updateAimPID() {

    double dt = aimTimer.seconds();
    aimTimer.reset();

    double error = currentGoal.getTx() - currentColor.getAimPose();

    // Integral
    aimIntegral += error * dt;

    // Derivative
    double derivative = (error - aimLastError) / dt;
    aimLastError = error;

    // PID Output
    double output = aimKp * error
        + aimKi * aimIntegral
        + aimKd * derivative;

    // Clamp for safety
    output = Math.max(-1, Math.min(1, output));

    return output;   // return turn power
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

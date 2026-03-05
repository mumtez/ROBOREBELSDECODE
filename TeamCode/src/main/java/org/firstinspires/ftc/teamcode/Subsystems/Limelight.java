package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.teamcode.AllianceColor;

@Configurable
public class Limelight {

  public static double AIM_Kp = 0.016;
  public static double AIM_Ki = 0.0;
  public static double AIM_Kd = 0.0017;
  public static double AIM_Ks = 0.06;

  public static double AIM_DEADBAND = .4;

  public final Limelight3A limelight;
  private final AllianceColor currentColor;
  private final ElapsedTime aimTimer = new ElapsedTime();

  public final Servo rgb;

  private LLResult currentGoal;
  private double lastCalculatedVel = Outtake.medSpeed;
  private double aimIntegral = 0;
  private double aimLastError = 0;


  private double vParallel;
  private double vPerpindicular;

  private double yVelocity;


  public double distance;


  public Limelight(LinearOpMode opMode, AllianceColor color) { // Constructor
    HardwareMap hardwareMap = opMode.hardwareMap;
    this.currentColor = color;
    this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
    this.limelight.start();

    this.rgb = hardwareMap.servo.get("rgb");
    this.rgb.setPosition(.5);
  }

  public void updateAim(double xVelocity, double yVelocity) {
    // Update goal
    this.limelight.pipelineSwitch(this.currentColor.getLLPipelineTeleOP());
    this.currentGoal = this.limelight.getLatestResult();

    // Update Velocities and Angles
    double txRad = Math.toRadians(currentGoal.getTx());

    this.vParallel =
        yVelocity * Math.cos(txRad)
            + xVelocity * Math.sin(txRad); // applying rotation matrix to get velocities relative to the goal
    this.vPerpindicular =
        xVelocity * Math.cos(txRad)
            - yVelocity * Math.sin(txRad);

    this.yVelocity = yVelocity;

    if (currentGoal != null && currentGoal.isValid()) {
      this.distance = (((41.275) / Math.tan((Math.toRadians(currentGoal.getTy() + 1.0)))) / 100.0);
    }

  }

  // TODO: below is how you properly designate return types / method descriptions in java --
  //        do the same for other methods.

  /**
   * Calculates the target velocity for the shooter based on the current teleop goal tag reading
   *
   * @return the target power for the shooter, or the last calculated power if no valid reading
   */
  public double calculateTargetVelocity() {
    double calculatedVel;
    if (currentGoal != null && currentGoal.isValid()) {

      calculatedVel = (20.0 * (Math.round(
          (((distance * Math.pow(0.243301244553 * distance - 0.173469387755, -0.5)) / 0.0025344670037)
              - yVelocity * 253)
              / 20.0))) - 140.0;

      lastCalculatedVel = calculatedVel;

      if (distance > 2.5) {
        return calculatedVel + 80;

      }
      return calculatedVel + 20;
    }
    return lastCalculatedVel;
  }

  public double updateAimPID(float rot) { // returns the turn power from pid for autoaiming
    if (currentGoal != null && currentGoal.isValid()) {
      double dt = aimTimer.seconds();
      aimTimer.reset();

      double leadAngleDeg =
          Math.toDegrees(Math.atan((vPerpindicular * Math.sqrt((2 * ((1.192 * distance) - .85)) / 9.46)) / distance));

      double error =
          currentGoal.getTx() - (currentColor.getAimPose() + leadAngleDeg); //TODO Test without y + distance

      // Integral
      aimIntegral += error * dt;

      // Derivative
      double derivative = (error - aimLastError) / dt;
      aimLastError = error;

      // PID Output

      if (Math.abs(error) < AIM_DEADBAND) {
        aimIntegral = 0;
        rgb.setPosition(.5);
        return 0;
      } else {
        rgb.setPosition(.277);
      }

      double output = AIM_Kp * error
          + AIM_Ki * aimIntegral
          + AIM_Kd * derivative
          + AIM_Ks * Math.signum(error);

      // Clamp for safety
      output = Range.clip(output, -1.0, 1.0);
      return output;   // return turn power
    }
    return rot;
  }


  public int getPatternIdAuto() { // only for auto just returns the tag id for patterns
    this.limelight.pipelineSwitch(0);
    LLResult result = this.limelight.getLatestResult();
    List<FiducialResult> fiducials = result.getFiducialResults();
    if (result.isValid()) {
      for (FiducialResult fiducial : fiducials) {
        return fiducial.getFiducialId();
      }
    }
    return 21;
  }

}

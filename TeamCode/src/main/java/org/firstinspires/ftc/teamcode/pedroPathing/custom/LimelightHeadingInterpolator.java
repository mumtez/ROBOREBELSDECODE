package org.firstinspires.ftc.teamcode.pedroPathing.custom;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;

public class LimelightHeadingInterpolator implements HeadingInterpolator {

  private final Limelight limelight;
  private final Follower follower;
  private final double fallbackHeading;

  public LimelightHeadingInterpolator(Limelight limelight, Follower follower) {
    this(limelight, follower, follower.getHeading());
  }

  /**
   * Creates a LimelightHeadingInterpolator with a specific fallback heading.
   *
   * @param limelight       The Limelight subsystem for vision tracking
   * @param follower        The Pedro Pathing follower (used to get velocity for lead angle)
   * @param fallbackHeading Heading to use when no target is visible (radians)
   */
  public LimelightHeadingInterpolator(Limelight limelight, Follower follower, double fallbackHeading) {
    this.limelight = limelight;
    this.follower = follower;
    this.fallbackHeading = fallbackHeading;
  }

  @Override
  public double interpolate(PathPoint pathPoint) {
    Vector botVelocity = follower.getVelocity();
    botVelocity.rotateVector(-follower.getHeading());

    // Update Limelight with current velocities (convert inches/sec to meters/sec)
    limelight.updateAim(
        (-botVelocity.getYComponent() * 2.54) / 100.0,
        (botVelocity.getXComponent() * 2.54) / 100.0
    );

    if (!limelight.hasValidTarget()) {
      // No target visible - maintain fallback heading
      return MathFunctions.normalizeAngle(fallbackHeading);
    }

    double currentHeading = follower.getHeading();
    double targetAngleDeg = limelight.calculateError();
    return MathFunctions.normalizeAngle(currentHeading + Math.toRadians(targetAngleDeg));
  }
}


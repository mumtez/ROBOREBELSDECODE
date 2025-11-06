package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

  public static FollowerConstants followerConstants = new FollowerConstants().mass(10.5)
      .forwardZeroPowerAcceleration(-42.202).lateralZeroPowerAcceleration(-81.4226)
      .headingPIDFCoefficients(new PIDFCoefficients(1.6, 0, .07, 0))
      .translationalPIDFCoefficients(new PIDFCoefficients(.18, 0, 0.02, 0))
      .drivePIDFCoefficients(new FilteredPIDFCoefficients(.018, 0, .00003, .6, .0)); //TODO UPDATE
  public static MecanumConstants driveConstants = new MecanumConstants()
      .maxPower(1)
      .rightFrontMotorName("fr")
      .rightRearMotorName("br")
      .leftRearMotorName("bl")
      .leftFrontMotorName("fl")
      .xVelocity(79.403)
      .yVelocity(63.081)
      .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
  public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
      .forwardTicksToInches(.001989436789)
      .strafeTicksToInches(.001989436789)
      .turnTicksToInches(-.001989436789)
      .leftPodY(7.5)
      .rightPodY(-7.5)
      .strafePodX(-2.5)
      .leftEncoder_HardwareMapName("br") //TODO FIX
      .rightEncoder_HardwareMapName("fl")
      .strafeEncoder_HardwareMapName("intakealt")
      .leftEncoderDirection(Encoder.REVERSE)
      .rightEncoderDirection(Encoder.REVERSE)
      .strafeEncoderDirection(Encoder.REVERSE);
  public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .pathConstraints(pathConstraints)
        .threeWheelLocalizer(localizerConstants)
        .mecanumDrivetrain(driveConstants)
        .build();
  }
}

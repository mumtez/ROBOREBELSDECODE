package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.pedroPathing.custom.LimelightHeadingInterpolator;

@Configurable
@Autonomous(name = "Test: Limelight Aiming", group = "Testing")
public class TestLimelightAimingAuton extends LinearOpMode {

  public static AllianceColor ALLIANCE_COLOR = AllianceColor.RED;

  public static double[] START_POSE = {0, 0, 0};
  public static double[] END_POSE = {25, 25, 0};
  public static double FALLBACK_ANGLE = 135;

  Robot robot;

  @Override
  public void runOpMode() {
    robot = new Robot(this, ALLIANCE_COLOR);
    robot.initAuton();

    Pose startPose = ALLIANCE_COLOR.poseFromArray(START_POSE);
    Pose endPose = ALLIANCE_COLOR.poseFromArray(END_POSE);

    PathChain path1 = robot.follower.pathBuilder()
        .addPath(new BezierLine(startPose, endPose))
        .setHeadingInterpolation(new LimelightHeadingInterpolator(robot.limelight, robot.follower, FALLBACK_ANGLE))
        .setHeadingConstraint(Math.toRadians(180)) // Required since we are not going to the end pose heading
        .build();

    PathChain path2 = robot.follower.pathBuilder()
        .addPath(new BezierLine(endPose, startPose))
        .setHeadingInterpolation(new LimelightHeadingInterpolator(robot.limelight, robot.follower, FALLBACK_ANGLE))
        .setHeadingConstraint(Math.toRadians(180)) // Required since we are not going to the end pose heading
        .addParametricCallback(.5, () -> robot.intake.setCyclePosition(FlapperState.SHOOT))

        .build();

    waitForStart();

    robot.follower.setStartingPose(startPose);

    robot.follower.followPath(path1);
    while (opModeIsActive() && robot.follower.isBusy()) {
      robot.follower.update();
      sendTelemetry();
    }

    robot.follower.followPath(path2, true);
    while (opModeIsActive() && robot.follower.isBusy()) {
      robot.follower.update();
      sendTelemetry();
    }
  }

  private void sendTelemetry() {
    telemetry.addData("Mode", "Limelight Aiming");
    telemetry.addData("Has Target", robot.limelight.hasValidTarget());
    telemetry.addData("Distance", "%.2f m", robot.limelight.distance);
    telemetry.addData("Target Angle", "%.1f deg", robot.limelight.calculateError());
    telemetry.update();
  }

}


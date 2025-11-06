package org.firstinspires.ftc.teamcode.Auton;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.opencv.core.Point;

@Configurable
public class BaseCloseAuto {

  public Limelight3A limelight;
  public static double[] START = {116, 131, 37};
  public static double[] SHOOT = {83, 83, 48};
  public static double[] INTAKEONELEFT = {100, 83, 0};
  public static double[] INTAKEONERIGHT = {100, 83, 0};

  public static double[] INTAKETWOLEFT = {100, 60, 0};
  public static double[] INTAKETWORIGHT = {100, 60, 0};

  PathChain
      shootPreLoad,
      intakeOneShootOne,
      intakeTwoShootTwo,
      intakeThreeShootThree;
  private int pathState = 0;

  private final Timer pathTimer = new Timer();
  private final ElapsedTime intakeTimer = new ElapsedTime();
  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseCloseAuto(LinearOpMode opMode, Robot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
  }

  public Point pointFromArr(double[] arr) {
    return new Point(arr[0], arr[1]);
  }

  public Pose poseFromArr(double[] arr) {
    return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void buildPaths() {
    shootPreLoad = robot.follower
        .pathBuilder()
        .addPath(
            new BezierLine(poseFromArr(START), poseFromArr(SHOOT))
        )
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), SHOOT[2])
        .build();

    intakeOneShootOne = robot.follower
        .pathBuilder()
        .addPath(
            new BezierLine(poseFromArr(SHOOT), poseFromArr(INTAKEONELEFT))
        )
        .setLinearHeadingInterpolation(Math.toRadians(SHOOT[2]), INTAKEONELEFT[2])
        .addParametricCallback(.5, () -> robot.intake.setPower(1))
        .addPath(
            new BezierLine(poseFromArr(INTAKEONELEFT), poseFromArr(INTAKEONERIGHT))
        )
        .setConstantHeadingInterpolation(0).setVelocityConstraint(5)
        .addPath(
            new BezierLine(poseFromArr(INTAKEONERIGHT), poseFromArr(SHOOT))
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKEONERIGHT[2]), SHOOT[2])
        .build();
    intakeTwoShootTwo = robot.follower
        .pathBuilder()
        .addPath(
            new BezierLine(poseFromArr(SHOOT), poseFromArr(INTAKETWOLEFT))
        )
        .setLinearHeadingInterpolation(Math.toRadians(SHOOT[2]), INTAKETWOLEFT[2])
        .addParametricCallback(.5, () -> robot.intake.setPower(1))
        .addPath(
            new BezierLine(poseFromArr(INTAKETWOLEFT), poseFromArr(INTAKETWORIGHT))
        )
        .setConstantHeadingInterpolation(0).setVelocityConstraint(5)
        .addPath(
            new BezierLine(poseFromArr(INTAKETWORIGHT), poseFromArr(SHOOT))
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKETWORIGHT[2]), SHOOT[2])
        .build();


  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        robot.outtake.setBase();
        robot.outtake.setTargetVelocity(Outtake.medSpeed);
        robot.follower.followPath(shootPreLoad);
        setPathState(1);
        break;
      case 1:
        if (!robot.follower.isBusy() && robot.outtake.atTarget()) {
          setPathState(2);
        }
        break;
      case 2:
        shootThree(3, intakeOneShootOne);
        break;
      case 3:
        if (!robot.follower.isBusy() && robot.outtake.atTarget()) {
          setPathState(4);
        }
        break;
      case 4:
        shootThree(5, intakeTwoShootTwo);
    }
  }

  public void shootThree(int pState, PathChain next) {
    robot.intake.setPower(1);
    robot.outtake.setShoot();
    while (opMode.opModeIsActive() && pathTimer.getElapsedTime() < 500) {
      // delay
      robot.updateAutoControls();
    }
    robot.outtake.setBase();
    while (opMode.opModeIsActive() && pathTimer.getElapsedTime() < 1200 && !robot.outtake.atTarget()) {
      // delay
      robot.updateAutoControls();
    }
    robot.outtake.setShoot();
    while (opMode.opModeIsActive() && pathTimer.getElapsedTime() < 1700) {
      // delay
      robot.updateAutoControls();
    }
    robot.outtake.setBase();
    while (opMode.opModeIsActive() && pathTimer.getElapsedTime() < 2400 && !robot.outtake.atTarget()) {
      // delay
      robot.updateAutoControls();
    }
    robot.outtake.setShoot();
    robot.intake.setPower(0);
    while (opMode.opModeIsActive() && pathTimer.getElapsedTime() < 3000) {
      // delay
      robot.updateAutoControls();
    }
    robot.outtake.setBase();
    while (opMode.opModeIsActive() && pathTimer.getElapsedTime() < 3700) {
      // delay
      robot.updateAutoControls();
    }
    robot.follower.followPath(next);
    setPathState(pState);

  }

  public void run() {
    buildPaths();
    robot.initAuton();

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.update();
    }
  }

}

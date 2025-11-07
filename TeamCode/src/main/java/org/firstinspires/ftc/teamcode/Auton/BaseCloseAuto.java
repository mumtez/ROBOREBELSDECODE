package org.firstinspires.ftc.teamcode.Auton;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Iterator;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseCloseAuto {

  public static double[] START_BLUE = {116, 131, 37};
  public static double[] SHOOT_BLUE = {86, 86, 46};

  public static double[] INTAKE_PPG_START_BLUE = {90, 85, 0};
  public static double[] INTAKE_PPG_END_BLUE   = {120, 85, 0};

  public static double[] INTAKE_PGP_START_BLUE = {90, 62, 0};
  public static double[] INTAKE_PGP_END_BLUE   = {120, 62, 0};

  public static double[] INTAKE_GPP_START_BLUE = {90, 39, 0};
  public static double[] INTAKE_GPP_END_BLUE   = {120, 39, 0};

  public static int OUTTAKE_SERVO_UP_MS = 500;
  public static int OUTTAKE_SERVO_DOWN_MS = 2000;
  public static double INTAKE_DRIVE_SPEED = 0.3;

  // TODO: implement park
  PathChain
      shootPreLoad,
      preIntakePPG, intakePPG, shootPPG,
      preIntakePGP, intakePGP, shootPGP,
      preIntakeGPP, intakeGPP, shootGPP;

  public enum PathState {
    PRELOAD, PPG, PGP, GPP, STOP
  }

  private PathState pathState = PathState.PRELOAD;
  private Iterator<PathState> pathOrder = List.of(PathState.PPG, PathState.PGP, PathState.GPP, PathState.STOP).iterator();

  private final Timer pathTimer = new Timer();
  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseCloseAuto(LinearOpMode opMode, Robot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
  }

  Pose poseFromArr(double[] arr) {
    return this.robot.getAllianceColor().poseFromArray(arr);
  }

  void setPathState(PathState pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  void buildPaths() {

    shootPreLoad = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(START_BLUE), poseFromArr(SHOOT_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(START_BLUE).getHeading(), poseFromArr(SHOOT_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();

    preIntakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT_BLUE), poseFromArr(INTAKE_PPG_START_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(SHOOT_BLUE).getHeading(), poseFromArr(INTAKE_PPG_START_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_START_BLUE), poseFromArr(INTAKE_PPG_END_BLUE)))
        .setConstantHeadingInterpolation(0)
        .setTimeoutConstraint(500)
        .build();
    shootPPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_END_BLUE), poseFromArr(SHOOT_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PPG_END_BLUE).getHeading(), poseFromArr(SHOOT_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();

    preIntakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT_BLUE), poseFromArr(INTAKE_PGP_START_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(SHOOT_BLUE).getHeading(), poseFromArr(INTAKE_PGP_START_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PGP_START_BLUE), poseFromArr(INTAKE_PGP_END_BLUE)))
        .setConstantHeadingInterpolation(0)
        .setTimeoutConstraint(500)
        .build();
    shootPGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PGP_END_BLUE), poseFromArr(SHOOT_BLUE)))
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_PGP_END_BLUE[2]), Math.toRadians(SHOOT_BLUE[2]))
        .setTimeoutConstraint(500)
        .build();

    preIntakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT_BLUE), poseFromArr(INTAKE_GPP_START_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(SHOOT_BLUE).getHeading(), poseFromArr(INTAKE_GPP_START_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_GPP_START_BLUE), poseFromArr(INTAKE_GPP_END_BLUE)))
        .setConstantHeadingInterpolation(0)
        .setTimeoutConstraint(500)
        .build();
    shootGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_GPP_END_BLUE), poseFromArr(SHOOT_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_GPP_END_BLUE).getHeading(), poseFromArr(SHOOT_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case PRELOAD:
        shootThree(shootPreLoad);
        setPathState(pathOrder.next());
        break;

      case PPG:
        intakeThree(preIntakePPG, intakePPG);
        shootThree(shootPPG);
        setPathState(pathOrder.next());
        break;

      case PGP:
        intakeThree(preIntakePGP, intakePGP);
        shootThree(shootPGP);
        setPathState(pathOrder.next());
        break;

      case GPP:
        intakeThree(preIntakeGPP, intakeGPP);
        shootThree(shootGPP);
        setPathState(pathOrder.next());
        break;

      case STOP:
        break;
    }
  }

  private void intakeThree(PathChain shootToIntake, PathChain intake) {
    robot.follower.followPath(shootToIntake);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    robot.intake.setPower(1);
    robot.follower.followPath(intake, INTAKE_DRIVE_SPEED, false);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }
  }

  private void shootThree(PathChain intakeToShoot) {
    robot.outtake.setTargetVelocity(Outtake.medSpeed);
    robot.outtake.setBase();

    robot.follower.followPath(intakeToShoot);
    while (opMode.opModeIsActive() && (robot.follower.isBusy() || !robot.outtake.atTarget())) {
      robot.updateAutoControls();
    }

    ElapsedTime shootTimer = new ElapsedTime();
    robot.intake.setPower(1);

    shootAndWait(shootTimer);
    reloadAndWait(shootTimer);
    shootAndWait(shootTimer);
    reloadAndWait(shootTimer);
    shootAndWait(shootTimer);

    robot.intake.setPower(0);
    robot.outtake.setTargetVelocity(0);
    robot.outtake.setBase();
  }

  private void shootAndWait(ElapsedTime shootTimer) {
    robot.outtake.setShoot();

    shootTimer.reset();
    while (opMode.opModeIsActive() && shootTimer.milliseconds() < OUTTAKE_SERVO_UP_MS) {
      // delay
      robot.updateAutoControls();
    }
  }

  private void reloadAndWait(ElapsedTime shootTimer) {
    robot.outtake.setBase();

    shootTimer.reset();
    while (opMode.opModeIsActive()
        && (shootTimer.milliseconds() < OUTTAKE_SERVO_DOWN_MS || !robot.outtake.atTarget())) {
      // delay
      robot.updateAutoControls();
    }
  }

  public void run() {
    // INIT
    buildPaths();
    robot.initAuton();

    robot.limelight.start();

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    // START
    robot.follower.setStartingPose(poseFromArr(START_BLUE));

    // TODO: read camera with limelight and update pathOrder

    // LOOP
    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.update();
    }
  }

}

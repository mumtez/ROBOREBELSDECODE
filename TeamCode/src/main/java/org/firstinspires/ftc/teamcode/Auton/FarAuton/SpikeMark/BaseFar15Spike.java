package org.firstinspires.ftc.teamcode.Auton.FarAuton.SpikeMark;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
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
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseFar15Spike {

  public static double INTAKE_TIMER = 200;
  public static double SHOOT_TIME = 1100;
  public static double PRELOAD_SHOOT_TIME = 1100;

  public static int CYCLE_LIMIT = 5;
  public static double INTAKE_DRIVE_MAX_POWER = 1;


  public static double[] START_RED = {88, 8, 90};
  public static double[] INTAKE_HP_START_RED = {118, 9, 0};
  public static double[] INTAKE_HP_MIDDLE_RED = {131, 9, 0};
  public static double[] INTAKE_HP_CONTROL_RED = {100, 16, 0};
  public static double[] INTAKE_HP_END_RED = {137, 23, 0};
  public static double[] PARK_POS = {105, 37, 0};


  public static double[] INTAKE_SPIKE_START_RED = {105, 37, 0};

  public static double[] INTAKE_SPIKE_END_RED = {135, 37, 0};


  int cycleCounter = 0;

  PathChain shootPreLoad, preIntakeHP, intakeHP, shootHP, parkPath, intakeSpike, shootSpike;

  public enum PathState {
    PRELOAD, INTAKE, CYCLE, STOP, PARK
  }

  private PathState pathState = PathState.PRELOAD;
  private Iterator<PathState> pathOrder;

  private final Timer pathTimer = new Timer();
  private final double[] shootPos; // This is the one non-mirrored point

  ElapsedTime globalTimer = new ElapsedTime();

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseFar15Spike(LinearOpMode opMode, Robot robot, double[] shootPos) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
    this.shootPos = shootPos;
  }

  Pose poseFromArr(double[] arr) {
    return this.robot.getAllianceColor().poseFromArray(arr);
  }

  Pose poseFromArrNonMirror(double[] arr) {
    return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
  }

  void setPathState(PathState pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  void buildPaths() {
    shootPreLoad = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(START_RED), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(
            poseFromArr(START_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(300)
        .build();

    preIntakeHP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_HP_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_HP_START_RED).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    intakeSpike = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_SPIKE_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_SPIKE_START_RED).getHeading()
        )

        .addPath(new BezierLine(poseFromArrNonMirror(INTAKE_SPIKE_START_RED), poseFromArr(INTAKE_SPIKE_END_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(INTAKE_SPIKE_START_RED).getHeading(),
            poseFromArr(INTAKE_SPIKE_END_RED).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    shootSpike = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(INTAKE_SPIKE_END_RED), poseFromArr(shootPos)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(INTAKE_SPIKE_END_RED).getHeading(),
            poseFromArr(shootPos).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    intakeHP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_HP_START_RED), poseFromArr(INTAKE_HP_MIDDLE_RED)))
        .addPath(new BezierCurve(
            poseFromArr(INTAKE_HP_MIDDLE_RED),
            poseFromArr(INTAKE_HP_CONTROL_RED),
            poseFromArr(INTAKE_HP_END_RED))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_HP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    shootHP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_HP_END_RED), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_HP_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(300)
        .build();

    parkPath = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(PARK_POS)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(PARK_POS).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();
  }

  public void autonomousPathUpdate() {
    if (globalTimer.seconds() >= 27 && pathState != PathState.PARK) {
      pathState = PathState.PARK;
    }

    switch (pathState) {
      case PRELOAD:
        ElapsedTime preloadTimer = new ElapsedTime();

        robot.follower.followPath(shootPreLoad);
        while (opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }

        robot.limelight.updateAim(0, 0);
        robot.outtake.setTargetVelocity(robot.limelight.calculateTargetVelocity());
        do {
          robot.updateAutoControls();
        } while (opMode.opModeIsActive() && !robot.outtake.atTarget());
        robot.intake.setCyclePosition(FlapperState.SHOOT);

        preloadTimer.reset();
        while (opMode.opModeIsActive() && preloadTimer.milliseconds() < PRELOAD_SHOOT_TIME) {
          robot.updateAutoControls();
        }

        robot.intake.setCyclePosition(FlapperState.LOCKED);

        setPathState(pathOrder.next());
        break;

      case INTAKE:
        intakeThree(intakeSpike);

        shootThree(shootSpike);
        setPathState(pathOrder.next());
        break;

      case CYCLE:
        cycleCounter++;
        intakeThree(preIntakeHP, intakeHP);

        robot.intake.setPowerInverse(1);

        shootThree(shootHP);

        if (cycleCounter >= CYCLE_LIMIT) {
          setPathState(pathOrder.next());
        }
        break;

      case PARK:
        robot.follower.followPath(parkPath);
        while (opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }
        setPathState(pathOrder.next());
        break;

      case STOP:
        robot.intake.setPower(0);
        robot.outtake.setTargetVelocity(0);
        robot.intake.setCyclePosition(FlapperState.LOCKED);
        break;

    }
  }

  private void intakeThree(PathChain shootToIntake, PathChain intake) {
    robot.intake.setPower(Intake.POWER_INTAKE);
    robot.follower.followPath(shootToIntake, true);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    robot.follower.followPath(intake, INTAKE_DRIVE_MAX_POWER, false);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }
    ElapsedTime intakeTimer = new ElapsedTime();
    while (opMode.opModeIsActive() && intakeTimer.milliseconds() <= INTAKE_TIMER) {
      robot.updateAutoControls();
    }
  }

  private void intakeThree(PathChain shootToIntake) {
    robot.intake.setPower(Intake.POWER_INTAKE);
    robot.follower.followPath(shootToIntake, true);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    ElapsedTime intakeTimer = new ElapsedTime();
    while (opMode.opModeIsActive() && intakeTimer.milliseconds() <= INTAKE_TIMER) {
      robot.updateAutoControls();
    }
  }

  private void shootThree(PathChain intakeToShoot) {
    ElapsedTime shootTimer = new ElapsedTime();
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    robot.follower.followPath(intakeToShoot, true);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    shootTimer.reset();
    robot.intake.setCyclePosition(FlapperState.SHOOT);
    while (opMode.opModeIsActive() && shootTimer.milliseconds() < SHOOT_TIME) {
      robot.updateAutoControls();
    }

    robot.intake.setCyclePosition(FlapperState.LOCKED);
  }


  public void run() {
    // INIT
    buildPaths();
    robot.initAuton();
    this.opMode.waitForStart();

    // START
    telemetry.addData("ALLIANCE", robot.getAllianceColor());
    telemetry.update();

    // START
    robot.follower.setStartingPose(poseFromArr(START_RED));
    robot.outtake.setTargetVelocity(Outtake.farSpeed);
    robot.intake.setPower(1);

    pathOrder = List.of(PathState.INTAKE, PathState.CYCLE, PathState.PARK, PathState.STOP).iterator();

    globalTimer.reset();

    // LOOP
    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Current Cycle", cycleCounter);
      telemetry.update();
    }
  }

}

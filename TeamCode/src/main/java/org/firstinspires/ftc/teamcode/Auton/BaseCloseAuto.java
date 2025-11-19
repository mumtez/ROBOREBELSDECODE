package org.firstinspires.ftc.teamcode.Auton;


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
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.opencv.core.Point;

@Configurable
public class BaseCloseAuto {

  public static double INTAKE_TIMER = 600;
  public int pattern = 0; //0 = gpp 1 = pgp 2 =ppg

  public static double CYCLE_TIMER = 1200;
  public static double TRANSFER_TIMER = 550; //500
  public static double[] START_RED = {113, 131, 180}; // initial rotation 37
  public static double[] SHOOT_CONTROl = {72, 48, 0};

  public static double[] INTAKE_PPG_START_RED = {83, 86, 0};
  public static double[] INTAKE_PPG_END_RED = {114, 86, 0};

  public static double[] INTAKE_PGP_START_RED = {86, 62, 0};
  public static double[] INTAKE_PGP_END_RED = {118, 62, 0};

  public static double[] INTAKE_GPP_START_RED = {88, 40, 0};
  public static double[] INTAKE_GPP_END_RED = {110, 40, 0};


  public static int OUTTAKE_SERVO_UP_MS = 400;
  public static int OUTTAKE_SERVO_DOWN_MS = 400;
  public static double INTAKE_DRIVE_SPEED = 0.3;

  public int currentTag = 21;

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
  private Iterator<PathState> pathOrder;

  private final Timer pathTimer = new Timer();
  private final double[] shootPos; // This is the one non mirrored point

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseCloseAuto(LinearOpMode opMode, Robot robot, double[] shootPos) {
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

  public Point pointFromArr(double[] arr) {
    return new Point(arr[0], arr[1]);
  }

  void buildPaths() {

    shootPreLoad = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(START_RED), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(START_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(500)
        .build();

    preIntakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PPG_START_RED)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_START_RED), poseFromArr(INTAKE_PPG_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(500)
        .build();
    shootPPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_END_RED), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PPG_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(500)
        .build();

    preIntakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PGP_START_RED)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PGP_START_RED).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PGP_START_RED), poseFromArr(INTAKE_PGP_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PGP_START_RED).getHeading())
        .setTimeoutConstraint(500)
        .build();
    shootPGP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_PGP_END_RED), poseFromArr(SHOOT_CONTROl),
            poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PGP_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(500)
        .build();

    preIntakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_GPP_START_RED)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_GPP_START_RED).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_GPP_START_RED), poseFromArr(INTAKE_GPP_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_GPP_START_RED).getHeading())
        .setTimeoutConstraint(500)
        .build();
    shootGPP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_GPP_END_RED), poseFromArr(SHOOT_CONTROl),
            poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_GPP_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(500)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case PRELOAD:

        if (pattern == 1) {
          cycle();
          cycle();
        }
        if (pattern == 2) {
          cycle();
        }
        robot.follower.followPath(shootPreLoad);
        shootThree();

        setPathState(pathOrder.next());
        break;

      case PPG:
        if (pattern != 2) {
          robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
        }
        intakeThree(preIntakePPG, intakePPG);
        if (pattern == 0) {
          cycle();
          cycle();
        }
        if (pattern == 1) {
          cycle();

        }
        shootThree(shootPPG);
        setPathState(pathOrder.next());
        break;

      case PGP:
        if (pattern != 1) {
          robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
        }
        intakeThree(preIntakePGP, intakePGP);
        if (pattern == 0) {
          cycle();
        }
        if (pattern == 2) {
          cycle();
          cycle();
        }
        shootThree(shootPGP);
        setPathState(pathOrder.next());
        break;

      case GPP:
        if (pattern != 0) {
          robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
        }
        intakeThree(preIntakeGPP, intakeGPP);
        if (pattern == 1) {
          cycle();
          cycle();
        }
        if (pattern == 2) {
          cycle();
        }
        shootThree(shootGPP);
        setPathState(pathOrder.next());
        break;

      case STOP:
        robot.intake.setPower(0);
        robot.outtake.setTargetVelocity(0);
        robot.outtake.setBase();
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

  private void shootThree() {
    ElapsedTime shootTimer = new ElapsedTime();

    robot.outtake.setBase();
    robot.intake.setPower(1);
    robot.outtake.setTargetVelocity(Outtake.medSpeed);

    while (opMode.opModeIsActive() && (robot.follower.isBusy()) || !robot.outtake.atTarget()) {
      robot.updateAutoControls();
    }

    shootAndWait(shootTimer);
    reloadAndWait(shootTimer);
    shootAndWait(shootTimer);
    reloadAndWait(shootTimer);
    shootAndWait(shootTimer);

    robot.intake.setPower(0);
    robot.outtake.setBase();
  }


  private void shootThree(PathChain intakeToShoot) {
    ElapsedTime shootTimer = new ElapsedTime();

    robot.outtake.setBase();
    robot.intake.setPower(1);
    robot.outtake.setTargetVelocity(Outtake.medSpeed);

    robot.follower.followPath(intakeToShoot, true); //TODO testing holdend
    while (opMode.opModeIsActive() && (robot.follower.isBusy()) || !robot.outtake.atTarget()) {
      robot.updateAutoControls();
    }

    shootAndWait(shootTimer);
    reloadAndWait(shootTimer);
    shootAndWait(shootTimer);
    reloadAndWait(shootTimer);
    shootAndWait(shootTimer);

    robot.intake.setPower(0);
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

  private void cycle() {
    ElapsedTime cycleTimer = new ElapsedTime();
    robot.outtake.setTargetVelocity(Outtake.cycleSpeed);

    while (opMode.opModeIsActive() && !robot.outtake.atTarget()) {
      // delay
      robot.updateAutoControls();
    }
    cycleTimer.reset();
    robot.outtake.setShoot();
    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < OUTTAKE_SERVO_UP_MS) {
      // delay
      robot.updateAutoControls();
    }
    robot.outtake.setBase();

    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < CYCLE_TIMER - OUTTAKE_SERVO_UP_MS) {
      // delay
      robot.updateAutoControls();
    }
    robot.intake.setPower(1); // TODO: this should be a constant in the intake class (Intake.POWER_IN)
    cycleTimer.reset();
    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < INTAKE_TIMER) {
      // delay
      robot.updateAutoControls();
    }
    cycleTimer.reset();
    robot.intake.setPowerVertical(-.6); // TODO: this should be a constant in the intake class (Intake.POWER_CYCLE?)
    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < TRANSFER_TIMER) {
      // delay
      robot.updateAutoControls();
    }

    robot.intake.setPower(1);
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

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      currentTag = robot.limelight.getPatternIdAuto();
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.addData("Tag", currentTag);

      telemetry.update();

    }

    // START
    robot.follower.setStartingPose(poseFromArr(START_RED));

    if (currentTag == 21) {
      pathOrder = List.of(PathState.GPP, PathState.PGP, PathState.PPG, PathState.STOP)
          .iterator();
      pattern = 0;

    }
    if (currentTag == 22) {
      pathOrder = List.of(PathState.PGP, PathState.PPG, PathState.GPP, PathState.STOP)
          .iterator();
      pattern = 1;
    }
    if (currentTag == 23) {
      pathOrder = List.of(PathState.PPG, PathState.PGP, PathState.GPP, PathState.STOP)
          .iterator();
      pattern = 2;
    }

    // LOOP
    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      robot.limelight.updateGoal();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.update();
    }
  }

}

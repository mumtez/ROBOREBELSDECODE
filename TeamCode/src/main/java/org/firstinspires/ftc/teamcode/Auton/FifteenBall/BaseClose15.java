package org.firstinspires.ftc.teamcode.Auton.FifteenBall;


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
public class BaseClose15 {

  private static double INTAKE_TIMER_GATE = 4000;
  private static double GATE_DRIVE_MAX_POWER = .8;

  private static double SHOOT_TIME = 1400;


  public static double[] START_RED = {114, 130, 39}; // 114.25, 130, 180
  public static double[] SHOOT_CONTROL = {70, 46, 0};

  public static double[] INTAKE_PPG_START_RED = {89, 86, 0};
  public static double[] INTAKE_PPG_END_RED = {122, 86, 0};

  public static double[] INTAKE_PGP_START_RED = {89, 60, 0};
  public static double[] INTAKE_PGP_END_RED = {125, 60, 0};

  public static double[] OPEN_GATE_START = {118, 74, 0};
  public static double[] OPEN_GATE_END = {122.75, 74, 0};

  public static double[] OPEN_GATE_CONTROL_PPG = {105, 72, 0};

  public static double[] OPEN_GATE_CONTROL_PGP = {105, 62, 0};


  public static double[] INTAKE_CLASSIFIER = {126.5, 60.5, 25};

  int cycleCounter = 0;

  public static int CYCLE_LIMIT = 2;

  public static double INTAKE_DRIVE_MAX_POWER = 1;

  public static double[] PARK_POS = {89, 60, 0};


  PathChain
      shootPreLoad,
      preIntakePPG, intakePPG, shootPPG,
      preIntakePGP, intakePGP, shootPGP,
      intakeClassifier, openGatePPG, openGatePGP, shootGate,
      parkPath, shootGateIntake;

  public enum PathState {
    PRELOAD, GATE, PGP, PPG, STOP, PARK
  }

  private PathState pathState = PathState.PRELOAD;
  private Iterator<PathState> pathOrder;

  private final Timer pathTimer = new Timer();
  private final double[] shootPos; // This is the one non mirrored point

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseClose15(LinearOpMode opMode, Robot robot, double[] shootPos) {
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
        .setLinearHeadingInterpolation(poseFromArr(START_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(300)
        .build();

    preIntakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PPG_START_RED)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();
    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_START_RED), poseFromArr(INTAKE_PPG_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();
    shootPPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_END_RED), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PPG_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(100)
        .build();

    preIntakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PGP_START_RED)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PGP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();
    intakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PGP_START_RED), poseFromArr(INTAKE_PGP_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PGP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();
    shootPGP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_PGP_END_RED), poseFromArr(SHOOT_CONTROL),
            poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PGP_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(100)
        .build();

    intakeClassifier = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(shootPos), poseFromArr(INTAKE_CLASSIFIER)))
        .setLinearHeadingInterpolation(poseFromArr(shootPos).getHeading(), poseFromArr(INTAKE_CLASSIFIER).getHeading())
        .setTimeoutConstraint(50)
        .build();

    parkPath = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(PARK_POS)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(PARK_POS).getHeading())
        .setTimeoutConstraint(50)
        .build();

    openGatePPG = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_PPG_END_RED), poseFromArr(OPEN_GATE_CONTROL_PPG),
            poseFromArr(OPEN_GATE_START)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PPG_END_RED).getHeading(),
            poseFromArr(OPEN_GATE_START).getHeading())
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_START), poseFromArr(OPEN_GATE_END)))
        .setLinearHeadingInterpolation(poseFromArr(OPEN_GATE_START).getHeading(),
            poseFromArr(OPEN_GATE_END).getHeading())
        .setTimeoutConstraint(3500)
        .build();

    openGatePGP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_PGP_END_RED), poseFromArr(OPEN_GATE_CONTROL_PGP),
            poseFromArr(OPEN_GATE_START)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PGP_END_RED).getHeading(),
            poseFromArr(OPEN_GATE_START).getHeading())
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_START), poseFromArr(OPEN_GATE_END)))
        .setLinearHeadingInterpolation(poseFromArr(OPEN_GATE_START).getHeading(),
            poseFromArr(OPEN_GATE_END).getHeading())
        .setTimeoutConstraint(3500)
        .build();

    shootGate = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_END),
            poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(OPEN_GATE_END).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(100)
        .build();

    shootGateIntake = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_CLASSIFIER), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_CLASSIFIER).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(100)
        .build();


  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case PRELOAD:

        shootThree(shootPreLoad);

        setPathState(pathOrder.next());
        break;
      case PGP:

        intakeThree(preIntakePGP, intakePGP);

        robot.intake.setPowerInverse(1); // TODO TEST ALL 2

        robot.follower.followPath(openGatePGP, GATE_DRIVE_MAX_POWER, true);

        shootThree(shootGate);
        setPathState(pathOrder.next());
        break;
      case PPG:

        intakeThree(preIntakePPG, intakePPG);
        robot.intake.setPowerInverse(1); // TODO TEST ALL 2

        robot.follower.followPath(openGatePPG, GATE_DRIVE_MAX_POWER, true);

        shootThree(shootGate);
        setPathState(pathOrder.next());
        break;

      case GATE:
        cycleCounter++;
        intakeGate(intakeClassifier);

        robot.intake.setPowerInverse(1);

        shootThree(shootGateIntake);

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
  }

  private void intakeGate(PathChain shootToIntake) {
    ElapsedTime gateIntakeTimer = new ElapsedTime();
    robot.intake.setPower(Intake.POWER_INTAKE);
    robot.follower.followPath(shootToIntake, true);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    while (opMode.opModeIsActive() && gateIntakeTimer.milliseconds() <= INTAKE_TIMER_GATE) {
      robot.updateAutoControls();
    }

  }

  private void shootThree(PathChain intakeToShoot) {
    ElapsedTime shootTimer = new ElapsedTime();
    // robot.intake.setPower(Intake.POWER_INTAKE); // TODO test without this
    while (opMode.opModeIsActive() && (robot.follower.isBusy())) {
      robot.updateAutoControls();
    }
    robot.follower.followPath(intakeToShoot, true);
    while (opMode.opModeIsActive() && (robot.follower.isBusy())) {
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

    // INIT LOOP
    while (this.opMode.opModeInInit()) {

    }

    telemetry.addData("ALLIANCE", robot.getAllianceColor());
    telemetry.update();

    // START
    robot.follower.setStartingPose(poseFromArr(START_RED));
    robot.outtake.setTargetVelocity(Outtake.medSpeed);
    robot.intake.setPower(1);

    pathOrder = List.of(PathState.PPG, PathState.PGP, PathState.GATE, PathState.PARK, PathState.STOP).iterator();

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

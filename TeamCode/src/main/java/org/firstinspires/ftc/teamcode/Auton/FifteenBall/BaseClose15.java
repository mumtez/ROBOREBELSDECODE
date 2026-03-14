package org.firstinspires.ftc.teamcode.Auton.FifteenBall;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.teamcode.Auton.BaseAuton;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseClose15 extends BaseAuton {

  public static double INTAKE_TIMER_GATE = 1200;
  public static double INTAKE_TIMER_MOVE = 200;
  public static int CYCLE_LIMIT = 2;

  public static double[] START_RED = {114, 130, 39}; // 114.25, 130, 180

  public static double[] INTAKE_PPG_START_RED = {89, 86, 0};
  public static double[] INTAKE_PPG_END_RED = {122, 86, 0};

  public static double[] INTAKE_PGP_START_RED = {89, 60, 0};
  public static double[] INTAKE_PGP_END_RED = {125, 60, 0};

  public static double[] OPEN_GATE_START = {118, 74, 0};
  public static double[] OPEN_GATE_END = {122.75, 74, 0};

  public static double[] OPEN_GATE_CONTROL_PPG = {105, 72, 0};
  public static double[] OPEN_GATE_CONTROL_PGP = {105, 62, 0};

  public static double[] INTAKE_CLASSIFIER = {126.5, 60.5, 25};
  public static double[] INTAKE_CLASSIFIER_TWO = {128.5, 52.5, 35}; //130.5

  public static double[] PARK_POS = {94, 65, 0};

  int cycleCounter = 0;

  PathChain
      shootPreLoad,
      intakePPG, openGatePPG,
      intakePGP, openGatePGP,
      intakeClassifier, intakeClassifierTwo,
      shootGate, shootGateIntake,
      parkPath;

  public BaseClose15(LinearOpMode opMode, Robot robot, double[] shootPos) {
    super(opMode, robot, shootPos);
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

    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PPG_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PPG_START_RED).getHeading()
        )
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_START_RED), poseFromArr(INTAKE_PPG_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    openGatePPG = robot.follower.pathBuilder()
        .addPath(new BezierCurve(
            poseFromArr(INTAKE_PPG_END_RED),
            poseFromArr(OPEN_GATE_CONTROL_PPG),
            poseFromArr(OPEN_GATE_START))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PPG_END_RED).getHeading())
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_START), poseFromArr(OPEN_GATE_END)))
        .setConstantHeadingInterpolation(poseFromArr(OPEN_GATE_START).getHeading())
        .build();

    intakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PGP_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PGP_START_RED).getHeading()
        )
        .addPath(new BezierLine(poseFromArr(INTAKE_PGP_START_RED), poseFromArr(INTAKE_PGP_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PGP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    openGatePGP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(
            poseFromArr(INTAKE_PGP_END_RED),
            poseFromArr(OPEN_GATE_CONTROL_PGP),
            poseFromArr(OPEN_GATE_START))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PGP_END_RED).getHeading())
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_START), poseFromArr(OPEN_GATE_END)))
        .setConstantHeadingInterpolation(poseFromArr(OPEN_GATE_START).getHeading())
        .build();

    intakeClassifier = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(shootPos), poseFromArr(INTAKE_CLASSIFIER)))
        .setLinearHeadingInterpolation(
            poseFromArr(shootPos).getHeading(),
            poseFromArr(INTAKE_CLASSIFIER).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    intakeClassifierTwo = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_CLASSIFIER), poseFromArr(INTAKE_CLASSIFIER_TWO)))
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_CLASSIFIER).getHeading(),
            poseFromArr(INTAKE_CLASSIFIER_TWO).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    parkPath = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(PARK_POS)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(PARK_POS).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    shootGate = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_END), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(
            poseFromArr(OPEN_GATE_END).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(300)
        .build();

    shootGateIntake = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_CLASSIFIER_TWO), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_CLASSIFIER_TWO).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(300)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case PRELOAD:
        shootThree(shootPreLoad, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
        setPathState(pathOrder.next());
        break;

      case PGP:
        intakeThree(null, intakePGP, INTAKE_DRIVE_MAX_POWER, INTAKE_SPIKE_TIME);
        robot.intake.setPowerInverse(1);
        robot.follower.followPath(openGatePGP, GATE_DRIVE_MAX_POWER, true);
        shootThree(shootGate, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
        setPathState(pathOrder.next());
        break;

      case PPG:
        intakeThree(null, intakePPG, INTAKE_DRIVE_MAX_POWER, INTAKE_SPIKE_TIME);
        robot.intake.setPowerInverse(1);
        robot.follower.followPath(openGatePPG, GATE_DRIVE_MAX_POWER, true);
        shootThree(shootGate, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
        setPathState(pathOrder.next());
        break;

      case GATE:
        cycleCounter++;
        intakeGate(intakeClassifier, intakeClassifierTwo);
        robot.intake.setPowerInverse(1);
        shootThree(shootGateIntake, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
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

  private void intakeGate(PathChain shootToIntake, PathChain intakeToIntakeTwo) {
    ElapsedTime gateIntakeTimer = new ElapsedTime();
    robot.intake.setPower(Intake.POWER_INTAKE);

    robot.follower.followPath(shootToIntake, true);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    gateIntakeTimer.reset();
    while (opMode.opModeIsActive() && gateIntakeTimer.milliseconds() <= INTAKE_TIMER_MOVE) {
      robot.updateAutoControls();
    }

    robot.follower.followPath(intakeToIntakeTwo, true);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    gateIntakeTimer.reset();
    while (opMode.opModeIsActive() && gateIntakeTimer.milliseconds() <= INTAKE_TIMER_GATE) {
      robot.updateAutoControls();
    }
  }

  public void run() {
    // INIT
    buildPaths();
    robot.initAuton();

    // INIT LOOP
    this.opMode.waitForStart();
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

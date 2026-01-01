package org.firstinspires.ftc.teamcode.Auton.TwelveBall;


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
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Pattern;

@Configurable
public class BaseClose12Indexed {

  // TODO: take care when naming variables that their names represent their usage properly.
  public static double INTAKE_TIMER = 600;
  public static double CYCLE_TIMER = 800;

  public static int TRANSFER_TIME_INTAKE_MS = 1000;

  public static double[] START_RED = {114.25, 130, 180};
  public static double[] SHOOT_CONTROL = {70, 46, 0};

  public static double[] INTAKE_PPG_START_RED = {83, 86, 0};
  public static double[] INTAKE_PPG_END_RED = {114, 86, 0};

  public static double[] OPEN_GATE_START = {110, 71, 90};
  public static double[] OPEN_GATE_END = {118, 71, 90};

  public static double[] OPEN_GATE_CONTROL = {72, 72, 0};


  public static double[] INTAKE_PGP_START_RED = {86, 60, 0};
  public static double[] INTAKE_PGP_END_RED = {121, 60, 0};

  public static double[] INTAKE_GPP_START_RED = {83, 36, 0};
  public static double[] INTAKE_GPP_END_RED = {120, 36, 0};

  public static int OUTTAKE_SERVO_UP_MS = 200;
  public static int OUTTAKE_SERVO_DOWN_MS = 400;
  public static double INTAKE_DRIVE_MAX_POWER = 1;

  private Pattern pattern = Pattern.GPP;
  public int currentTag = 21;

  PathChain
      shootPreLoad,
      preIntakePPG, intakePPG, shootPPG,
      preIntakePGP, intakePGP, shootPGP,
      preIntakeGPP, intakeGPP, shootGPP,
      openGate, shootOpenGate;

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

  public BaseClose12Indexed(LinearOpMode opMode, Robot robot, double[] shootPos) {
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
        .setTimeoutConstraint(100)
        .build();

    preIntakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PPG_START_RED)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(100)
        .build();
    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_START_RED), poseFromArr(INTAKE_PPG_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    openGate = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_PPG_END_RED), poseFromArr(OPEN_GATE_CONTROL),
            poseFromArr(OPEN_GATE_START)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PPG_END_RED).getHeading(),
            poseFromArr(OPEN_GATE_START).getHeading())
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_START), poseFromArr(OPEN_GATE_END)))
        .setLinearHeadingInterpolation(poseFromArr(OPEN_GATE_START).getHeading(),
            poseFromArr(OPEN_GATE_END).getHeading())
        .setTimeoutConstraint(3000)
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_END), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(OPEN_GATE_END).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(100)
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
        .setTimeoutConstraint(100)
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

    preIntakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_GPP_START_RED)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_GPP_START_RED).getHeading())
        .setTimeoutConstraint(100)
        .build();
    intakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_GPP_START_RED), poseFromArr(INTAKE_GPP_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_GPP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();
    shootGPP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_GPP_END_RED), poseFromArr(SHOOT_CONTROL),
            poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_GPP_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading())
        .setTimeoutConstraint(100)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case PRELOAD:

        // TODO: why make an entire separate shootThree method just to do the same thing?
        //  The other one just calls followPath a couple ms later in the cycle and with holdEnd=true.
        //  If you don't want to hold end add in a parameter for it in the method and get rid of the extra one.
        //  They are too similar to be separate.
        robot.follower.followPath(shootPreLoad);
        shootThree();

        setPathState(pathOrder.next());
        break;

      case PPG:
        ElapsedTime gateTimer = new ElapsedTime();
        if (pattern != Pattern.PPG) {
          robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
        }

        intakeThree(preIntakePPG, intakePPG);

        while (this.opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }

        robot.follower.followPath(openGate, INTAKE_DRIVE_MAX_POWER, true);

        if (pattern == Pattern.GPP) {
          cycle(TRANSFER_TIME_INTAKE_MS);
          cycle(TRANSFER_TIME_INTAKE_MS);
        }
        if (pattern == Pattern.PGP) {
          cycle(TRANSFER_TIME_INTAKE_MS);
        }

        while (this.opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }

        shootThree();
        setPathState(pathOrder.next());
        break;

      case PGP:
        if (pattern != Pattern.PGP) {
          robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
        }
        intakeThree(preIntakePGP, intakePGP);
        robot.follower.followPath(shootPGP);
        if (pattern == Pattern.GPP) {
          cycle(TRANSFER_TIME_INTAKE_MS);
        }
        if (pattern == Pattern.PPG) {
          cycle(TRANSFER_TIME_INTAKE_MS);
          cycle(TRANSFER_TIME_INTAKE_MS);
        }
        shootThree();
        setPathState(pathOrder.next());
        break;

      case GPP:
        if (pattern != Pattern.GPP) {
          robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
        }
        intakeThree(preIntakeGPP, intakeGPP);
        robot.follower.followPath(shootGPP);
        if (pattern == Pattern.PGP) {
          cycle(TRANSFER_TIME_INTAKE_MS);
          cycle(TRANSFER_TIME_INTAKE_MS);
        }
        if (pattern == Pattern.PPG) {
          cycle(TRANSFER_TIME_INTAKE_MS);
        }
        shootThree();
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

    robot.intake.setPower(Intake.POWER_INTAKE);
    robot.follower.followPath(intake, INTAKE_DRIVE_MAX_POWER, false);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }
  }

  // TODO: see above todo on line 170
  private void shootThree() {
    ElapsedTime shootTimer = new ElapsedTime();

    robot.outtake.setBase();
    robot.intake.setPower(Intake.POWER_INTAKE);
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
    robot.intake.setPower(Intake.POWER_INTAKE);
    robot.outtake.setTargetVelocity(Outtake.medSpeed);

    robot.follower.followPath(intakeToShoot, true);
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

  // TODO: we should be able to do this while driving.
  //  Make this a non-blocking routine in Robot controlled by a state machine.

  // TODO 2: This method should also take in the current and target Patterns.
  //  Try using the example in Pattern.java to simplify that implementation and above usages of cycle :)
  private void cycle(int transferTimeMs) {
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

    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < CYCLE_TIMER) {
      // delay
      robot.updateAutoControls();
    }
    robot.intake.setPower(Intake.POWER_INTAKE);
    cycleTimer.reset();
    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < INTAKE_TIMER) {
      // delay
      robot.updateAutoControls();
    }
    cycleTimer.reset();

    robot.intake.setPowerVertical(Intake.POWER_CYCLE_VERTICAL);
    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < transferTimeMs) {
      // delay
      robot.updateAutoControls();
    }

    robot.intake.setPower(Intake.POWER_INTAKE);
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

      switch (currentTag) {
        case 21:
          pattern = Pattern.GPP;
          break;
        case 22:
          pattern = Pattern.PGP;
          break;
        case 23:
          pattern = Pattern.PPG;
          break;
      }
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.addData("Tag", currentTag);
      telemetry.addData("Pattern", pattern);
      telemetry.update();

    }

    // START
    robot.follower.setStartingPose(poseFromArr(START_RED));

    //  If we can upgrade the JDK version to 21 (or kotlin) then we could use the even nicer switch syntax!
    switch (pattern) {
      case GPP:
        pathOrder = List.of(PathState.PPG, PathState.GPP, PathState.PGP, PathState.STOP).iterator();
        break;
      case PGP:
        pathOrder = List.of(PathState.PPG, PathState.PGP, PathState.GPP, PathState.STOP).iterator();
        break;
      case PPG:
        pathOrder = List.of(PathState.PPG, PathState.GPP, PathState.PGP, PathState.STOP).iterator();
        break;
    }

    // LOOP
    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Pattern", pattern);
      telemetry.update();
    }
  }

}

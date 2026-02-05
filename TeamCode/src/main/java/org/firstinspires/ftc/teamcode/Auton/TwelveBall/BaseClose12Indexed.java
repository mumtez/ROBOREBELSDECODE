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
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Pattern;

@Configurable
public class BaseClose12Indexed {

  private static double SHOOT_POWER = .8;

  // TODO: take care when naming variables that their names represent their usage properly.


  private static double SHOOT_TIME = 1800;


  public static double[] START_RED = {114.25, 130, 180};
  public static double[] SHOOT_CONTROL = {70, 46, 0};

  public static double[] INTAKE_PPG_START_RED = {89, 86, 0};
  public static double[] INTAKE_PPG_END_RED = {123, 86, 0};

  public static double[] OPEN_GATE_START = {118, 75, 0};
  public static double[] OPEN_GATE_END = {124, 75, 0};

  public static double[] OPEN_GATE_CONTROL = {95, 80, 0};


  public static double[] INTAKE_PGP_START_RED = {89, 60, 0};
  public static double[] INTAKE_PGP_END_RED = {131, 60, 0};

  public static double[] INTAKE_GPP_START_RED = {89, 36, 0};
  public static double[] INTAKE_GPP_END_RED = {125, 36, 0};

  public static double[] PARK_POS = {83, 36, 0};

  public static double INTAKE_DRIVE_MAX_POWER = .8;

  private Pattern pattern = Pattern.GPP;
  public int currentTag = 21;

  PathChain
      shootPreLoad,
      preIntakePPG, intakePPG, shootPPG,
      preIntakePGP, intakePGP, shootPGP,
      preIntakeGPP, intakeGPP, shootGPP,
      openGate, shootGate, parkPath;

  public enum PathState {
    PRELOAD, PPG, PGP, GPP, PARK, STOP
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

    parkPath = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(PARK_POS)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(PARK_POS).getHeading())
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
        .setTimeoutConstraint(4500)
        .build();
    shootGate = robot.follower.pathBuilder()
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

        shootThree(shootPreLoad);

        setPathState(pathOrder.next());
        break;

      case PPG:

        intakeThree(preIntakePPG, intakePPG);
        if (pattern == Pattern.GPP) {
          robot.intake.cycle(2);
        }
        if (pattern == Pattern.PGP) {
          robot.intake.cycle(1);
        }

        robot.follower.followPath(openGate, INTAKE_DRIVE_MAX_POWER, true);

        while (this.opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }

        shootThree(shootGate);
        setPathState(pathOrder.next());
        break;

      case PGP:
        intakeThree(preIntakePGP, intakePGP);
        if (pattern == Pattern.GPP) {
          robot.intake.cycle(1);
        }
        if (pattern == Pattern.PPG) {
          robot.intake.cycle(2);
        }
        shootThree(shootPGP);
        setPathState(pathOrder.next());
        break;

      case GPP:
        intakeThree(preIntakeGPP, intakeGPP);
        if (pattern == Pattern.PGP) {
          robot.intake.cycle(2);
        }
        if (pattern == Pattern.PPG) {
          robot.intake.cycle(1);
        }
        shootThree(shootGPP);
        setPathState(pathOrder.next());
        break;

      case STOP:
        robot.intake.setPower(0);
        robot.outtake.setTargetVelocity(0);
        robot.intake.setCyclePosition(FlapperState.LOCKED);
        break;

      case PARK:
        robot.follower.followPath(parkPath);
        break;
    }
  }

  private void intakeThree(PathChain shootToIntake, PathChain intake) {

    ElapsedTime intakeTimer = new ElapsedTime();
    robot.follower.followPath(shootToIntake);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    robot.intake.setPower(Intake.POWER_INTAKE);
    robot.follower.followPath(intake, INTAKE_DRIVE_MAX_POWER, false);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }
    intakeTimer.reset();
    while (opMode.opModeIsActive() && intakeTimer.milliseconds() <= 1000) {
      robot.updateAutoControls();
    }

  }


  private void shootThree(PathChain intakeToShoot) {
    ElapsedTime shootTimer = new ElapsedTime();
    while (opMode.opModeIsActive() && (robot.follower.isBusy())) {
      robot.updateAutoControls();
    }
    robot.follower.followPath(intakeToShoot, true);
    while (opMode.opModeIsActive() && (robot.follower.isBusy() || !robot.intake.isCycleFinished())) {
      robot.updateAutoControls();
    }
    shootTimer.reset();
    robot.intake.setPower(SHOOT_POWER);
    robot.intake.setCyclePosition(FlapperState.SHOOT);
    while (opMode.opModeIsActive() && shootTimer.milliseconds() < SHOOT_TIME) {
      robot.updateAutoControls();
    }
    robot.intake.setPower(1);
    robot.intake.setCyclePosition(FlapperState.LOCKED);
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
    robot.outtake.setTargetVelocity(Outtake.medSpeed);
    robot.intake.setPower(1);

    //  If we can upgrade the JDK version to 21 (or kotlin) then we could use the even nicer switch syntax!
    switch (pattern) {
      case GPP:
        pathOrder = List.of(PathState.PPG, PathState.GPP, PathState.PGP, PathState.PARK, PathState.STOP).iterator();
        break;
      case PGP:
        pathOrder = List.of(PathState.PPG, PathState.PGP, PathState.GPP, PathState.PARK, PathState.STOP).iterator();
        break;
      case PPG:
        pathOrder = List.of(PathState.PPG, PathState.GPP, PathState.PGP, PathState.PARK, PathState.STOP).iterator();
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

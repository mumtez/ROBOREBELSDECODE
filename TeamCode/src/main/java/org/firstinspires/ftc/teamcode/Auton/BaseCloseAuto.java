package org.firstinspires.ftc.teamcode.Auton;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Iterator;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.opencv.core.Point;

@Configurable
public class BaseCloseAuto {

  // TODO: implement limelight

  public static double[] START = {116, 131, 37};
  public static double[] SHOOT = {86, 86, 46};

  public static double[] INTAKE_ONE_LEFT = {90, 85, 0};
  public static double[] INTAKE_ONE_RIGHT = {120, 85, 0};

  public static double[] INTAKE_TWO_LEFT = {90, 62, 0};
  public static double[] INTAKE_TWO_RIGHT = {120, 62, 0};

  public static double[] INTAKE_THREE_LEFT = {90, 39, 0};
  public static double[] INTAKE_THREE_RIGHT = {120, 39, 0};

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
  private Iterator<PathState> pathOrder = List.of(PathState.PPG, PathState.PGP, PathState.GPP, PathState.STOP)
      .iterator();

  private final Timer pathTimer = new Timer();
  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseCloseAuto(LinearOpMode opMode, Robot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
  }

  Point pointFromArr(double[] arr) {
    return new Point(arr[0], arr[1]);
  }

  Pose poseFromArr(double[] arr) {
    return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
  }

  void setPathState(PathState pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  void buildPaths() {
    shootPreLoad = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(START), poseFromArr(SHOOT)))
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), Math.toRadians(SHOOT[2]))
        .setTimeoutConstraint(500)
        .build();

    preIntakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT), poseFromArr(INTAKE_ONE_LEFT)))
        .setLinearHeadingInterpolation(Math.toRadians(SHOOT[2]), Math.toRadians(INTAKE_ONE_LEFT[2]))
        .setTimeoutConstraint(500)
        .build();
    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_ONE_LEFT), poseFromArr(INTAKE_ONE_RIGHT)))
        .setConstantHeadingInterpolation(0)
        .setTimeoutConstraint(500)
        .build();
    shootPPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_ONE_RIGHT), poseFromArr(SHOOT)))
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_ONE_RIGHT[2]), Math.toRadians(SHOOT[2]))
        .setTimeoutConstraint(500)
        .build();

    preIntakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT), poseFromArr(INTAKE_TWO_LEFT)))
        .setLinearHeadingInterpolation(Math.toRadians(SHOOT[2]), Math.toRadians(INTAKE_TWO_LEFT[2]))
        .build();
    intakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_TWO_LEFT), poseFromArr(INTAKE_TWO_RIGHT)))
        .setConstantHeadingInterpolation(0)
        .build();
    shootPGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_TWO_RIGHT), poseFromArr(SHOOT)))
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_TWO_RIGHT[2]), Math.toRadians(SHOOT[2]))
        .build();

    preIntakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT), poseFromArr(INTAKE_THREE_LEFT)))
        .setLinearHeadingInterpolation(Math.toRadians(SHOOT[2]), Math.toRadians(INTAKE_THREE_LEFT[2]))
        .build();
    intakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_THREE_LEFT), poseFromArr(INTAKE_THREE_RIGHT)))
        .setConstantHeadingInterpolation(0)
        .build();
    shootGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_THREE_RIGHT), poseFromArr(SHOOT)))
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_THREE_RIGHT[2]), Math.toRadians(SHOOT[2]))
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
    robot.follower.setStartingPose(new Pose(START[0], START[1], Math.toRadians(START[2])));

    // TODO: read camera with limelight and update pathOrder

    // LOOP
    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      LLResult result = robot.limelight.getLatestResult(); //TODO demo limeliht code actually have to put real logic
      if (result != null) {
        if (result.isValid()) {
          Pose3D botpose = result.getBotpose();
          telemetry.addData("tx", result.getTx());
          telemetry.addData("ty", result.getTy());
          telemetry.addData("Botpose", botpose.toString());
        }
      }

      telemetry.addData("Path State", pathState);
      telemetry.update();
    }
  }

}

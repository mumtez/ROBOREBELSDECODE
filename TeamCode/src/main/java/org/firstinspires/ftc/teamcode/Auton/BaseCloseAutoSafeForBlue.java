package org.firstinspires.ftc.teamcode.Auton;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
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
public class BaseCloseAutoSafeForBlue {

  public static double INTAKE_TIMER = 600;
  public int pattern = 0; //0 = gpp 1 = pgp 2 =ppg

  public static double CYCLE_TIMER = 1200;
  public static double TRANSFER_TIMER = 500;
  public static double[] START_BLUE = {113, 131, 180}; // initial rotation 37
  public static double[] SHOOT_BLUE = {64, 88, 134}; // TODO This is the one non mirrored point

  public static double[] INTAKE_PPG_START_BLUE = {83, 88, 0};
  public static double[] INTAKE_PPG_END_BLUE = {114, 88, 0};

  public static double[] SHOOT_CONTROl = {72, 48, 0};


  public static double[] INTAKE_PGP_START_BLUE = {86, 64, 0};
  public static double[] INTAKE_PGP_END_BLUE = {116, 64, 0};


  public static double[] INTAKE_GPP_START_BLUE = {88, 41, 0};
  public static double[] INTAKE_GPP_END_BLUE = {110, 41, 0};


  public static int OUTTAKE_SERVO_UP_MS = 500;
  public static int OUTTAKE_SERVO_DOWN_MS = 1000;
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
  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseCloseAutoSafeForBlue(LinearOpMode opMode, Robot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
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
        .addPath(new BezierLine(poseFromArr(START_BLUE), poseFromArrNonMirror(SHOOT_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(START_BLUE).getHeading(),
            poseFromArrNonMirror(SHOOT_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();

    preIntakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(SHOOT_BLUE), poseFromArr(INTAKE_PPG_START_BLUE)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(SHOOT_BLUE).getHeading(),
            poseFromArr(INTAKE_PPG_START_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_START_BLUE), poseFromArr(INTAKE_PPG_END_BLUE)))
        .setConstantHeadingInterpolation(Math.toRadians(180))
        .setTimeoutConstraint(500)
        .build();
    shootPPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_END_BLUE), poseFromArrNonMirror(SHOOT_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PPG_END_BLUE).getHeading(),
            poseFromArrNonMirror(SHOOT_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();

    preIntakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(SHOOT_BLUE), poseFromArr(INTAKE_PGP_START_BLUE)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(SHOOT_BLUE).getHeading(),
            poseFromArr(INTAKE_PGP_START_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_PGP_START_BLUE), poseFromArr(INTAKE_PGP_END_BLUE)))
        .setConstantHeadingInterpolation(Math.toRadians(180))
        .setTimeoutConstraint(500)
        .build();
    shootPGP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_PGP_END_BLUE), poseFromArr(SHOOT_CONTROl),
            poseFromArrNonMirror(SHOOT_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_PGP_END_BLUE).getHeading(),
            poseFromArrNonMirror(SHOOT_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();

    preIntakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(SHOOT_BLUE), poseFromArr(INTAKE_GPP_START_BLUE)))
        .setLinearHeadingInterpolation(poseFromArrNonMirror(SHOOT_BLUE).getHeading(),
            poseFromArr(INTAKE_GPP_START_BLUE).getHeading())
        .setTimeoutConstraint(500)
        .build();
    intakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_GPP_START_BLUE), poseFromArr(INTAKE_GPP_END_BLUE)))
        .setConstantHeadingInterpolation(Math.toRadians(180))
        .setTimeoutConstraint(500)
        .build();
    shootGPP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(poseFromArr(INTAKE_GPP_END_BLUE), poseFromArr(SHOOT_CONTROl),
            poseFromArrNonMirror(SHOOT_BLUE)))
        .setLinearHeadingInterpolation(poseFromArr(INTAKE_GPP_END_BLUE).getHeading(),
            poseFromArrNonMirror(SHOOT_BLUE).getHeading())
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

  private void shootThree(PathChain intakeToShoot) {
    ElapsedTime shootTimer = new ElapsedTime();

    robot.outtake.setTargetVelocity(Outtake.medSpeed);
    robot.outtake.setBase();
    robot.intake.setPower(1);

    robot.follower.followPath(intakeToShoot, true); //TODO testing holdend
    while (opMode.opModeIsActive() && (robot.follower.isBusy() || !robot.outtake.atTarget())) {
      robot.updateAutoControls();
    }

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

  private void cycle() {
    ElapsedTime cycleTimer = new ElapsedTime();
    robot.outtake.setTargetVelocity(Outtake.cycleSpeed);

    while (opMode.opModeIsActive() && !robot.outtake.atTarget()) {
      // delay
      robot.updateAutoControls();

    }
    cycleTimer.reset();
    robot.outtake.setShoot();
    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < CYCLE_TIMER) {
      // delay
      robot.updateAutoControls();
    }
    robot.outtake.setBase();
    robot.intake.setPower(1);
    cycleTimer.reset();
    while (opMode.opModeIsActive() && cycleTimer.milliseconds() < INTAKE_TIMER) {
      // delay
      robot.updateAutoControls();
    }
    cycleTimer.reset();
    robot.intake.setPowerVertical(-.6);
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

    robot.limelight.start();

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
      robot.limelight.pipelineSwitch(1);
      LLResult result = robot.limelight.getLatestResult(); //TODO demo limelight code actually have to put real logic
      List<FiducialResult> fiducials = result.getFiducialResults();
      if (result != null) {
        if (result.isValid()) {
          Pose3D botpose = result.getBotpose();
          for (FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            currentTag = id;
            telemetry.addData("Tag ", id);
          }
        }
      }
    }

    // START
    robot.follower.setStartingPose(poseFromArr(START_BLUE));

    // TODO: read camera with limelight and update pathOrder
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
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.update();
    }
  }

}

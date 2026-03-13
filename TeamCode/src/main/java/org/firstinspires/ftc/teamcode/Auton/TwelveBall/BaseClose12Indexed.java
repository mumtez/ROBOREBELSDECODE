package org.firstinspires.ftc.teamcode.Auton.TwelveBall;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.teamcode.Auton.BaseAuton;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Pattern;

@Configurable
public class BaseClose12Indexed extends BaseAuton {

  public static double SHOOT_POWER = .7; // Testing this slightly slower

  public static double[] START_RED = {114.25, 130, 180};
  public static double[] SHOOT_CONTROL = {69, 50, 0}; //46 = y

  public static double[] INTAKE_PPG_START_RED = {89, 86, 0};
  public static double[] INTAKE_PPG_END_RED = {122, 86, 0};

  public static double[] OPEN_GATE_START = {118, 75, 0};
  public static double[] OPEN_GATE_CONTROL = {95, 80, 0};
  public static double[] OPEN_GATE_END = {121, 75, 0};

  public static double[] INTAKE_PGP_START_RED = {89, 60, 0};
  public static double[] INTAKE_PGP_END_RED = {125, 60, 0};

  public static double[] INTAKE_GPP_START_RED = {89, 36, 0};
  public static double[] INTAKE_GPP_END_RED = {125, 36, 0};

  public static double[] PARK_POS = {89, 60, 0};


  private Pattern pattern = Pattern.GPP;
  public int currentTag = 21;

  PathChain
      shootPreLoad,
      preIntakePPG, intakePPG, shootPPG,
      preIntakePGP, intakePGP, shootPGP,
      preIntakeGPP, intakeGPP, shootGPP,
      openGate, shootGate,
      parkPath;

  public BaseClose12Indexed(LinearOpMode opMode, Robot robot, double[] shootPos) {
    super(opMode, robot, shootPos);
  }

  void buildPaths() {
    shootPreLoad = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArr(START_RED),
            poseFromArrNonMirror(shootPos))
        )
        .setLinearHeadingInterpolation(
            poseFromArr(START_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(100)
        .build();

    preIntakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArrNonMirror(shootPos),
            poseFromArr(INTAKE_PPG_START_RED))
        )
        .setLinearHeadingInterpolation(poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(100)
        .build();

    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArr(INTAKE_PPG_START_RED),
            poseFromArr(INTAKE_PPG_END_RED))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    parkPath = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArrNonMirror(shootPos),
            poseFromArr(PARK_POS))
        )
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(PARK_POS).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    openGate = robot.follower.pathBuilder()
        .addPath(new BezierCurve(
            poseFromArr(INTAKE_PPG_END_RED),
            poseFromArr(OPEN_GATE_CONTROL),
            poseFromArr(OPEN_GATE_START))
        )
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_PPG_END_RED).getHeading(),
            poseFromArr(OPEN_GATE_START).getHeading()
        )
        .addPath(new BezierLine(
            poseFromArr(OPEN_GATE_START), poseFromArr(OPEN_GATE_END))
        )
        .setLinearHeadingInterpolation(
            poseFromArr(OPEN_GATE_START).getHeading(),
            poseFromArr(OPEN_GATE_END).getHeading()
        )
        .setTimeoutConstraint(200)
        .build();

    shootGate = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArr(OPEN_GATE_END),
            poseFromArrNonMirror(shootPos))
        )
        .setLinearHeadingInterpolation(
            poseFromArr(OPEN_GATE_END).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(100)
        .build();

    // Note: currently unused
    shootPPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArr(INTAKE_PPG_END_RED),
            poseFromArrNonMirror(shootPos))
        )
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_PPG_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(100)
        .build();

    preIntakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PGP_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PGP_START_RED).getHeading()
        )
        .setTimeoutConstraint(100)
        .build();

    intakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArr(INTAKE_PGP_START_RED),
            poseFromArr(INTAKE_PGP_END_RED))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PGP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    shootPGP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(
            poseFromArr(INTAKE_PGP_END_RED),
            poseFromArr(SHOOT_CONTROL),
            poseFromArrNonMirror(shootPos))
        )
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_PGP_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(100)
        .build();

    preIntakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArrNonMirror(shootPos),
            poseFromArr(INTAKE_GPP_START_RED))
        )
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_GPP_START_RED).getHeading()
        )
        .setTimeoutConstraint(100)
        .build();

    intakeGPP = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArr(INTAKE_GPP_START_RED),
            poseFromArr(INTAKE_GPP_END_RED))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_GPP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    shootGPP = robot.follower.pathBuilder()
        .addPath(new BezierCurve(
            poseFromArr(INTAKE_GPP_END_RED),
            poseFromArr(SHOOT_CONTROL),
            poseFromArrNonMirror(shootPos))
        )
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_GPP_END_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(100)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case PRELOAD:
        shootThree(shootPreLoad, 1, SHOOT_TIME_QUICK);
        setPathState(pathOrder.next());
        break;

      case PPG:
        intakeThree(preIntakePPG, intakePPG, INTAKE_DRIVE_MAX_POWER_SLOW, INTAKE_SPIKE_TIME);
        if (pattern == Pattern.GPP) {
          robot.intake.cycle(2);
        }
        if (pattern == Pattern.PGP) {
          robot.intake.cycle(1);
        }

        robot.follower.followPath(openGate, GATE_DRIVE_MAX_POWER, true);
        while (this.opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }

        ElapsedTime gateHoldTimer = new ElapsedTime();
        while (this.opMode.opModeIsActive() && gateHoldTimer.milliseconds() < 200) {
          robot.updateAutoControls();
        }

        shootThree(shootGate, SHOOT_POWER, SHOOT_TIME_SLOW);
        setPathState(pathOrder.next());
        break;

      case PGP:
        intakeThree(preIntakePGP, intakePGP, INTAKE_DRIVE_MAX_POWER_SLOW, INTAKE_SPIKE_TIME);
        if (pattern == Pattern.GPP) {
          robot.intake.cycle(1);
        }
        if (pattern == Pattern.PPG) {
          robot.intake.cycle(2);
        }
        shootThree(shootPGP, SHOOT_POWER, SHOOT_TIME_SLOW);
        setPathState(pathOrder.next());
        break;

      case GPP:
        intakeThree(preIntakeGPP, intakeGPP, INTAKE_DRIVE_MAX_POWER_SLOW, INTAKE_SPIKE_TIME);
        if (pattern == Pattern.PGP) {
          robot.intake.cycle(2);
        }
        if (pattern == Pattern.PPG) {
          robot.intake.cycle(1);
        }
        shootThree(shootGPP, SHOOT_POWER, SHOOT_TIME_SLOW);
        setPathState(pathOrder.next());
        break;

      case STOP:
        robot.intake.setPower(0);
        robot.outtake.setTargetVelocity(0);
        robot.intake.setCyclePosition(FlapperState.LOCKED);
        break;

      case PARK:
        robot.follower.followPath(parkPath);
        while (opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }
        setPathState(pathOrder.next());
        break;
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
    robot.outtake.setTargetVelocity(Outtake.medSpeed - 40);
    robot.intake.setPower(1);

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

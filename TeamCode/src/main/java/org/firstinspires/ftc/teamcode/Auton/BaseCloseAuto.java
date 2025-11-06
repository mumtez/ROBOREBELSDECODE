package org.firstinspires.ftc.teamcode.Auton;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.opencv.core.Point;

@Configurable
public class BaseCloseAuto {

  // TODO: implement limelight
  public Limelight3A limelight;

  public static double[] START = {116, 131, 37};
  public static double[] SHOOT = {86, 86, 40};
  public static double[] INTAKE_ONE_LEFT = {100, 83, 0};
  public static double[] INTAKE_ONE_RIGHT = {100, 83, 0};

  public static double[] INTAKE_TWO_LEFT = {100, 60, 0};
  public static double[] INTAKE_TWO_RIGHT = {100, 60, 0};

  public static int OUTTAKE_SERVO_UP_MS = 500;
  public static int OUTTAKE_SERVO_DOWN_MS = 2000;
  public static int velConst = 1;

  // TODO: implement intakeThreeShootThree, park
  PathChain
      shootPreLoad,
      intakeOneShootOne, intakeTwoShootTwo, intakeThreeShootThree,
      park;
  private int pathState = 0;

  private final Timer pathTimer = new Timer();
  private final ElapsedTime intakeTimer = new ElapsedTime();
  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseCloseAuto(LinearOpMode opMode, Robot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
  }

  public Point pointFromArr(double[] arr) {
    return new Point(arr[0], arr[1]);
  }

  public Pose poseFromArr(double[] arr) {
    return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void buildPaths() {
    shootPreLoad = robot.follower
        .pathBuilder()
        .addPath(new BezierLine(poseFromArr(START), poseFromArr(SHOOT)))
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), Math.toRadians(SHOOT[2]))
        .build();

    intakeOneShootOne = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT), poseFromArr(INTAKE_ONE_LEFT)))
        .setLinearHeadingInterpolation(Math.toRadians(SHOOT[2]), INTAKE_ONE_LEFT[2])
        .addParametricCallback(.5, () -> robot.intake.setPower(1))

        .addPath(new BezierLine(poseFromArr(INTAKE_ONE_LEFT), poseFromArr(INTAKE_ONE_RIGHT)))
        .setConstantHeadingInterpolation(0)
        .setVelocityConstraint(velConst)

        .addPath(new BezierLine(poseFromArr(INTAKE_ONE_RIGHT), poseFromArr(SHOOT)))
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_ONE_RIGHT[2]), Math.toRadians(SHOOT[2]))
        .build();

    intakeTwoShootTwo = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT), poseFromArr(INTAKE_TWO_LEFT)))
        .setLinearHeadingInterpolation(Math.toRadians(SHOOT[2]), INTAKE_TWO_LEFT[2])
        .addParametricCallback(.5, () -> robot.intake.setPower(1))

        .addPath(new BezierLine(poseFromArr(INTAKE_TWO_LEFT), poseFromArr(INTAKE_TWO_RIGHT)))
        .setConstantHeadingInterpolation(0)
        .setVelocityConstraint(5)

        .addPath(new BezierLine(poseFromArr(INTAKE_TWO_RIGHT), poseFromArr(SHOOT)))
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_TWO_RIGHT[2]), Math.toRadians(SHOOT[2]))
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        robot.outtake.setBase();
        robot.outtake.setTargetVelocity(Outtake.medSpeed);
        robot.follower.followPath(shootPreLoad);
        setPathState(1);
        break;

      case 1:
        if (!robot.follower.isBusy() && robot.outtake.atTarget()) {
          shootThree(2, intakeOneShootOne);
        }
        break;

      case 2:
        if (!robot.follower.isBusy() && robot.outtake.atTarget()) {
          shootThree(3, intakeTwoShootTwo);
        }
        break;

      case 3:
        if (!robot.follower.isBusy() && robot.outtake.atTarget()) {
          shootThree(4, intakeThreeShootThree);
        }
        break;

      // TODO: implement
//      case 4:
//        if (!robot.follower.isBusy() && robot.outtake.atTarget()) {
//          robot.follower.followPath(park);
//        }
//        break;
    }
  }

  public void shootThree(int pState, PathChain next) {
    ElapsedTime shootTimer = new ElapsedTime();
    robot.intake.setPower(1);

    shootAndWait(shootTimer);
    reloadAndWait(shootTimer);
    shootAndWait(shootTimer);
    reloadAndWait(shootTimer);
    shootAndWait(shootTimer);

    robot.outtake.setBase();
    robot.follower.followPath(next);
    setPathState(pState);
  }

  private void shootAndWait(ElapsedTime shootTimer) {
    shootTimer.reset();
    robot.outtake.setShoot();
    while (opMode.opModeIsActive() && shootTimer.milliseconds() < OUTTAKE_SERVO_UP_MS) {
      // delay
      robot.updateAutoControls();
    }
  }

  private void reloadAndWait(ElapsedTime shootTimer) {
    robot.outtake.setBase();
    shootTimer.reset();
    while (opMode.opModeIsActive() && shootTimer.milliseconds() < OUTTAKE_SERVO_DOWN_MS && !robot.outtake.atTarget()) {
      // delay
      robot.updateAutoControls();
    }
  }

  public void run() {
    buildPaths();
    robot.initAuton();
    robot.follower.setStartingPose(new Pose(START[0], START[1], Math.toRadians(START[2])));

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.update();
    }
  }

}

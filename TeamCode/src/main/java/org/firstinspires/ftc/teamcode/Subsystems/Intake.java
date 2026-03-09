package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Intake {

  // Color Sensor tuning vars


  public static final double POWER_INTAKE = 1;

  public static Direction intakeMotorDirection = Direction.FORWARD;
  public static Direction intakeMotorAltDirection = Direction.FORWARD;
  public final DcMotor intakeMotor;
  public final DcMotor intakeMotorAlt;


  private enum CycleState {PENDING, OPEN, CLOSE, CLOSE_DELAY}

  public static int OPEN_DELAY = 400;

  public static int CLOSE_DELAY = 500;

  private CycleState cycleState = CycleState.PENDING;

  ElapsedTime cycleTimer = new ElapsedTime();

  private int remainingCycles = 0;

  public static double SHOOT_BASE = 1;
  public static double SHOOT_CYCLE = .52;

  public static double SHOOT_POS = 0.39;


  public static double CYCLE_BASE = 1;
  public static double CYCLE_DEPLOY = 0;


  public ServoImplEx gate;
  public ServoImplEx cycler;

  public ServoImplEx intakeServo;

  public enum FlapperState {CYCLE, SHOOT, LOCKED}

  public Intake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    intakeMotorAlt = hardwareMap.get(DcMotorEx.class, "intakealt");
    intakeMotor.setDirection(intakeMotorDirection);
    intakeMotorAlt.setDirection(intakeMotorAltDirection);
    intakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    intakeMotorAlt.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    intakeMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    intakeMotorAlt.setMode(RunMode.RUN_WITHOUT_ENCODER);

    gate = hardwareMap.get(ServoImplEx.class, "flapper");
    cycler = hardwareMap.get(ServoImplEx.class, "cycler");

    intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");


  }


  public void setCyclePosition(FlapperState state) {
    switch (state) {
      case CYCLE:
        gate.setPosition(SHOOT_CYCLE);
        cycler.setPosition(CYCLE_DEPLOY);
        break;
      case SHOOT:
        gate.setPosition(SHOOT_POS);
        cycler.setPosition(CYCLE_BASE);
        break;
      case LOCKED:
        gate.setPosition(SHOOT_BASE);
        cycler.setPosition(CYCLE_BASE);
        break;
    }
  }

  public void setFlapperPos(double pos) {
    gate.setPosition(pos);
  }

  public void setCyclerPos(double pos) {
    cycler.setPosition(pos);
  }

  public void setPower(double pow) {
    intakeMotor.setPower(pow);
    intakeMotorAlt.setPower(pow);
  }

  public void setPowerInverse(double pow) {
    intakeMotor.setPower(-pow);
    intakeMotorAlt.setPower(pow);
  }

  public void cycle(int num) {
    remainingCycles = num;
  }

  public void cycleIncrementByNum(int num) {
    remainingCycles += num;
  }

  public boolean isCycleFinished() {
    return this.cycleState == CycleState.PENDING;
  }


  public void updateAutoCycle() {
    switch (cycleState) {
      case PENDING:
        if (remainingCycles > 0) {
          this.cycleState = CycleState.OPEN;
        }
        break;
      case OPEN:
        remainingCycles -= 1;
        this.setCyclePosition(FlapperState.CYCLE);
        this.cycleTimer.reset();
        this.cycleState = CycleState.CLOSE;
        break;
      case CLOSE:
        if (this.cycleTimer.milliseconds() >= OPEN_DELAY) {
          setCyclePosition(FlapperState.LOCKED);
          this.cycleTimer.reset();
          this.cycleState = CycleState.CLOSE_DELAY;
        }
        break;
      case CLOSE_DELAY:
        if (this.cycleTimer.milliseconds() >= CLOSE_DELAY) {
          if (remainingCycles == 0) {
            this.cycleState = CycleState.PENDING;
          } else {
            this.cycleState = CycleState.OPEN;
          }

        }

        break;
    }
  }

  public void cancelCycle() {
    this.remainingCycles = 0;
    this.cycleState = CycleState.PENDING;
    setCyclePosition(FlapperState.LOCKED);
  }


}

package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Outtake {

  // --- PID constants (tune in Panels) ---

  public static int medSpeed = 1340; // 1340
  public static int farSpeed = 1600;

  public static int cycleSpeed = 300;
  public static double kP = 0.002; // .0025 //m001
  public static double kV = 0.00039; // .00037


  public static double SHOOT_POS = 0.7;
  public static double SHOOT_BASE = 0.95;

  public static double CYCLE_BASE = 0.7;
  public static double CYCLE_BALL = 0.95; //TODO TUNE

  public static Direction flywheelMotorDirection = Direction.REVERSE;

  // --- Variables ---
  private double targetVelocity = 0; // ticks/sec

  // --- Hardware ---
  public DcMotorEx flywheel1;
  public DcMotorEx flywheel2;
  public ServoImplEx flapper;
  public ServoImplEx cycler;
  public final Servo rgb;
  private double currentVelocity;

  // --- Constructor ---
  public Outtake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    rgb = hardwareMap.servo.get("rgb");
    rgb.setPosition(.5);
    flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
    flywheel2.setDirection(flywheelMotorDirection);
    flywheel2.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel2.setMode(RunMode.RUN_WITHOUT_ENCODER);
    flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
    flywheel1.setDirection(flywheelMotorDirection);
    flywheel1.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel1.setMode(RunMode.RUN_WITHOUT_ENCODER);
    flapper = hardwareMap.get(ServoImplEx.class, "flapper");
    cycler = hardwareMap.get(ServoImplEx.class, "cycler");

  }

  public void setPower(double pow) {
    this.flywheel1.setPower(pow);
    this.flywheel2.setPower(pow); // TODO CHANGE THIS BEFORE TESTING
  }

  // --- Set target velocity ---
  public void setTargetVelocity(double targetTicksPerSec) {
    targetVelocity = Math.max(targetTicksPerSec, 0);
  }

  public void setShoot() {
    flapper.setPosition(SHOOT_POS);
  }

  public void setBase() {
    flapper.setPosition(SHOOT_BASE);
    cycler.setPosition(CYCLE_BASE);
  }

  public void setCycle() {
    cycler.setPosition(CYCLE_BALL);
    flapper.setPosition(SHOOT_POS);
  }

  public void setServoPos(double pos) {
    flapper.setPosition(pos);
  }

  // --- Main PID update loop ---
  public double updatePIDControl() {
    if (this.atTarget(20) && targetVelocity != 0) {
      rgb.setPosition(.5);
    } else if (this.atTarget(100) && targetVelocity != 0) {
      rgb.setPosition(.375);
    } else {
      rgb.setPosition(.3);
    }
    this.currentVelocity = this.flywheel1.getVelocity(); // ticks/sec
    double error = this.targetVelocity - this.currentVelocity;

    double output = (kV * this.targetVelocity) + (kP * error);

    // limit power range
    output = Range.clip(output, -0.2, 1.0);

    this.setPower(output);
    return output;
  }

  public double getCurrentVelocity() {
    return this.currentVelocity;
  }

  public boolean atTarget() {
    return atTarget(40);
  }

  public boolean atTarget(double threshold) {
    return Math.abs(this.getCurrentVelocity() - this.targetVelocity) < threshold;
  }

  public double getTargetVelocity() {
    return this.targetVelocity;
  }

  public void stop() {
    this.setTargetVelocity(0);
  }
}

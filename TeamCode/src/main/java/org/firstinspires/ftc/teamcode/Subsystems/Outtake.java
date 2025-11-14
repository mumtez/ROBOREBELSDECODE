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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Outtake {

  // --- PID constants (tune in Panels) ---

  public static int medSpeed = 1320;
  public static int farSpeed = 1600;

  public static int cycleSpeed = 500;
  public static double kP = .028; //TODO tune
  public static double kV = 1 / 2500; //TODO tune


  public static double SHOOT_POS = 0.7;
  public static double SHOOT_BASE = 0.95;

  public static Direction flywheelMotorDirection = Direction.REVERSE;

  // --- Variables ---
  private double targetVelocity = 0; // ticks/sec
  private final ElapsedTime timer = new ElapsedTime();

  // --- Hardware ---
  public DcMotorEx flywheel;
  public ServoImplEx flapper;
  public final Servo rgb;

  // --- Constructor ---
  public Outtake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    rgb = hardwareMap.servo.get("rgb");
    rgb.setPosition(.5);
    flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    flywheel.setDirection(flywheelMotorDirection);
    flywheel.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel.setMode(RunMode.RUN_WITHOUT_ENCODER);
    flapper = hardwareMap.get(ServoImplEx.class, "flapper");

  }

  public void setPower(double pow) {
    this.flywheel.setPower(pow);
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
  }

  public void setServoPos(double pos) {
    flapper.setPosition(pos);
  }

  // --- Main PID update loop ---
  public double updatePIDControl() {
    if (this.atTarget(40) && targetVelocity != 0) {
      rgb.setPosition(.5);
    } else {
      rgb.setPosition(.3);
    }

    double currentVelocity = this.getCurrentVelocity(); // ticks/sec
    double error = this.targetVelocity - currentVelocity;

    double output = (kV * this.targetVelocity) + (kP * error);

    // limit power range
    output = Range.clip(output, -0.2, 1.0);

    this.setPower(output);
    return output;
  }

  public double getCurrentVelocity() {
    return flywheel.getVelocity();
  }

  public boolean atTarget() {
    return atTarget(60);
  }

  public boolean atTarget(double threshold) {
    return Math.abs(this.getCurrentVelocity() - this.targetVelocity) < threshold;
  }

  public double getTargetVelocity() {
    return this.targetVelocity;
  }

  public void stop() {
    targetVelocity = 0;
  }
}

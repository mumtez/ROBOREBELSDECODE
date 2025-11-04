package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {

  public DcMotor flywheel;
  public static double kP = 0.00;
  public static double kI = 0.000;
  public static double kD = 0.000;

  public static double targetVelocity = 0;
  public static double lastError = 0;
  public static double integralSum = 0;
  public static long lastTime = 0;
  public static int lastPosition = 0;

  public Outtake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    flywheel = hardwareMap.dcMotor.get("flywheel");
    flywheel.setDirection(Direction.FORWARD);
    flywheel.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel.setMode(RunMode.RUN_WITHOUT_ENCODER);

    // initialize encoder state
    lastPosition = flywheel.getCurrentPosition();
    lastTime = System.nanoTime();
  }

  public void setTargetVelocity(double target) {
    targetVelocity = target; // ticks per second
  }

  public void updatePID() {
    // get current time and position
    long currentTime = System.nanoTime();
    int currentPosition = flywheel.getCurrentPosition();

    // time step (s)
    double deltaTime = (currentTime - lastTime) / 1e9;
    if (deltaTime <= 0) deltaTime = 1e-6;

    // velocity in ticks/sec
    double currentVelocity = (currentPosition - lastPosition) / deltaTime;

    // PID calculations
    double error = targetVelocity - currentVelocity;
    integralSum += error * deltaTime;
    double derivative = (error - lastError) / deltaTime;

    double output = kP * error + kI * integralSum + kD * derivative;
    output = Math.max(-1.0, Math.min(1.0, output));

    flywheel.setPower(output);

    // update stored values
    lastError = error;
    lastPosition = currentPosition;
    lastTime = currentTime;
  }

  public double getCurrentVelocity() {
    long currentTime = System.nanoTime();
    double deltaTime = (currentTime - lastTime) / 1e9;
    if (deltaTime <= 0) deltaTime = 1e-6;
    int currentPosition = flywheel.getCurrentPosition();
    return (currentPosition - lastPosition) / deltaTime;
  }
}

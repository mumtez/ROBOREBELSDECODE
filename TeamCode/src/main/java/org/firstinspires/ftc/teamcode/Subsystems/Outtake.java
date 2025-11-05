
package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Outtake {

  // --- PID constants (tune in Panels) ---
  public static double kP = 0.02;
  public static double kI = 0.000;
  public static double kD = 0.000;

  public static double SHOOT_POS =0;

  public static double SHOOT_BASE =0;

  public static Direction flywheelMotorDirection = Direction.REVERSE;

  // --- Variables ---
  private double targetVelocity = 0; // ticks/sec
  private double lastError = 0;
  private double integralSum = 0;
  private final ElapsedTime timer = new ElapsedTime();

  // --- Hardware ---
  public DcMotorEx flywheel;
  public ServoImplEx flapper;

  // --- Constructor ---
  public Outtake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    flywheel.setDirection(flywheelMotorDirection);
    flywheel.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel.setMode(RunMode.RUN_WITHOUT_ENCODER);
    flapper = hardwareMap.get(ServoImplEx.class, "flapper");
    timer.reset();
  }

  public void setPower(double pow) {
    this.flywheel.setPower(pow);
  }

  // --- Set target velocity ---
  public void setTargetVelocity(double targetTicksPerSec) {
    targetVelocity = Math.max(targetTicksPerSec, 0);
    integralSum = 0;
    lastError = 0;
    timer.reset();
  }
  public void setShoot(){
    flapper.setPosition(SHOOT_POS);
  }
  public void setBase(){
    flapper.setPosition(SHOOT_BASE);
  }
  public void setServoPos(double pos){
    flapper.setPosition(pos);
  }



  // --- Main PID update loop ---
  public double updatePIDControl() {
    double dt = timer.seconds();
    timer.reset();
    if (dt == 0) return 0;

    double currentVelocity = this.getCurrentVelocity(); // ticks/sec
    double error = targetVelocity - currentVelocity;

    // PID calculations
    integralSum += error * dt;
    double derivative = (error - lastError) / dt;
    lastError = error;

    double output = (kP * error) + (kI * integralSum) + (kD * derivative);

    // limit power range
    output = Range.clip(output, 0.0, 1.0);

    this.setPower(output);
    return output;
  }

  public double getCurrentVelocity() {
    return flywheel.getVelocity();
  }
  public boolean atSpeed(double speed){
    if (this.getCurrentVelocity() >= speed) {
      return true;
    }
    return false;
  }
  public boolean atTarget() {
    return atTarget(5);
  }
  public boolean atTarget(double threshold) {
    return Math.abs(this.getCurrentVelocity() - this.targetVelocity) < threshold;
  }
  public double getTargetVelocity(){
    return this.targetVelocity;
  }

  public void stop() {
    flywheel.setPower(0);
    targetVelocity = 0;
    integralSum = 0;
    lastError = 0;
  }
}

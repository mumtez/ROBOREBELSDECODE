package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
  public static Direction intakeMotorDirection = Direction.FORWARD;
  public static DcMotor intakeMotor;
  public Intake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    intakeMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
    intakeMotor.setDirection(intakeMotorDirection);
    intakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    intakeMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
  }

  public void setPower(double pow){
    intakeMotor.setPower(pow);
  }

}

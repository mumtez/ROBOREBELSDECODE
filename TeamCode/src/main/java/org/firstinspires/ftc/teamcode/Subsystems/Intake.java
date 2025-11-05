package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {

  public static Direction intakeMotorDirection = Direction.FORWARD;
  public static Direction intakeMotorAltDirection = Direction.REVERSE;
  public final DcMotor intakeMotor;
  public final DcMotor intakeMotorAlt;

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
  }

  public void setPower(double pow) {
    intakeMotor.setPower(pow);
    intakeMotorAlt.setPower(pow);
  }

}

package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@Configurable
public class Intake {

  private static final double DIST_THRESHOLD_CM = 3;
  private static final float GREEN_THRESHOLD = .025f;

  private static final float RED_THRESHOLD = .014f;

  private static final float BLUE_THRESHOLD = .2f;

  public enum BallColor {
    GREEN, PURPLE, NONE
  }

  public static Direction intakeMotorDirection = Direction.FORWARD;
  public static Direction intakeMotorAltDirection = Direction.REVERSE;
  public final DcMotor intakeMotor;
  public final DcMotor intakeMotorAlt;
  private BallColor ballColor = BallColor.NONE;

  public final NormalizedColorSensor colorSensor;


  public Intake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    intakeMotorAlt = hardwareMap.get(DcMotorEx.class, "intakealt");
    colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
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

  public void setPowerVertical(double pow) {
    intakeMotor.setPower(pow);
    intakeMotorAlt.setPower(-pow);
  }


  public void updateSampleColor() {
    if (colorSensor.getNormalizedColors().green > GREEN_THRESHOLD
        && colorSensor.getNormalizedColors().red < RED_THRESHOLD) {
      ballColor = BallColor.GREEN;

    } else if (colorSensor.getNormalizedColors().green < GREEN_THRESHOLD
        && colorSensor.getNormalizedColors().red > RED_THRESHOLD) {
      ballColor = BallColor.PURPLE;

    } else if (colorSensor.getNormalizedColors().green < GREEN_THRESHOLD
        && colorSensor.getNormalizedColors().red < RED_THRESHOLD) {
      ballColor = BallColor.NONE;

    }
  }

  public BallColor getColor() {
    return ballColor;
  }
}

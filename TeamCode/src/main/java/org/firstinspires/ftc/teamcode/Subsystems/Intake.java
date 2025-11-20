package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Intake {

  // Color Sensor tuning vars
  public static final float COLOR_GAIN = 2;
  public static final double DIST_THRESHOLD_CM = 3;
  public static final float GREEN_THRESHOLD = .022f;
  public static final float RED_THRESHOLD = .014f;

  public static final double POWER_CYCLE_VERTICAL = -.6;

  public static final double POWER_INTAKE = 1;

  public static Direction intakeMotorDirection = Direction.FORWARD;
  public static Direction intakeMotorAltDirection = Direction.REVERSE;
  public final DcMotor intakeMotor;
  public final DcMotor intakeMotorAlt;
  public final NormalizedColorSensor cs1, cs2;

  private BallColor currentBallColor = BallColor.NONE;

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

    cs1 = hardwareMap.get(NormalizedColorSensor.class, "color");
    cs1.setGain(COLOR_GAIN);
    if (cs1 instanceof SwitchableLight) {
      ((SwitchableLight) cs1).enableLight(true);
    }
    cs2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
    cs2.setGain(COLOR_GAIN);
    if (cs2 instanceof SwitchableLight) {
      ((SwitchableLight) cs2).enableLight(true);
    }

  }

  public void setPower(double pow) {
    intakeMotor.setPower(pow);
    intakeMotorAlt.setPower(pow);
  }

  public void setPowerVertical(double pow) {
    intakeMotor.setPower(pow);
    intakeMotorAlt.setPower(-pow);
  }

  public BallColor updateSampleColor() {
    if (this.readDistance(cs1) < DIST_THRESHOLD_CM) {
      this.currentBallColor = readBallColor(cs1);
    } else {
      this.currentBallColor = readBallColor(cs2);
    }

    return this.currentBallColor;
  }

  public double readDistance(NormalizedColorSensor cs) {
    if (cs instanceof DistanceSensor) {
      return ((DistanceSensor) cs).getDistance(DistanceUnit.CM);
    }
    return 0;
  }

  public BallColor readBallColor(NormalizedColorSensor cs) {
    NormalizedRGBA colors = cs.getNormalizedColors();
    if (colors.green > GREEN_THRESHOLD && colors.red < RED_THRESHOLD) {
      return BallColor.GREEN;
    } else if (colors.green < GREEN_THRESHOLD && colors.red > RED_THRESHOLD) {
      return BallColor.PURPLE;
    } else {
      return BallColor.NONE;
    }
  }
}

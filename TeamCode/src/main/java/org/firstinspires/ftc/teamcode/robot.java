package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.List;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class robot {
  public enum AllianceColor {
    RED, BLUE
  }
  public Follower follower;
  public DcMotor fr, fl, br, bl;
  public IMU imu;

  public Outtake outtake;

  private AllianceColor allianceColor;  //0 red 1 blue

  public robot(LinearOpMode opMode) {
    this(opMode, AllianceColor.RED);
  }

  public robot(LinearOpMode opMode, AllianceColor allianceColor) {
    this.allianceColor = allianceColor;

    HardwareMap hardwareMap = opMode.hardwareMap;

    // From https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
    fl = hardwareMap.dcMotor.get("fl");
    fr = hardwareMap.dcMotor.get("fr");
    bl = hardwareMap.dcMotor.get("bl");
    br = hardwareMap.dcMotor.get("br");

    fl.setDirection(Direction.REVERSE);
    fr.setDirection(Direction.FORWARD);
    bl.setDirection(Direction.REVERSE);
    br.setDirection(Direction.FORWARD);

    fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(RunMode.RUN_WITHOUT_ENCODER);

    fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        LogoFacingDirection.RIGHT,
        UsbFacingDirection.UP));
    imu.initialize(parameters);
    opMode.telemetry.addData("IMU Initialized", true);
    opMode.telemetry.update();
    outtake = new Outtake(opMode);
  }
  public void initAuton() {
    // when start auto will do this
  }
  public AllianceColor getAllianceColor() {
    return this.allianceColor;
  }

  public void updateAutoControls() {
    follower.update();
    outtake.updatePID();
  }
}

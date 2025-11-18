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
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {

  public final ElapsedTime stateTimer = new ElapsedTime();

  public Follower follower;
  public DcMotor fr, fl, br, bl;
  public IMU imu;

  public Outtake outtake;
  public Intake intake;
  public Limelight limelight;


  private final AllianceColor allianceColor;

  public Robot(LinearOpMode opMode) {
    this(opMode, AllianceColor.RED);
  }

  public Robot(LinearOpMode opMode, AllianceColor allianceColor) {
    this.allianceColor = allianceColor;

    HardwareMap hardwareMap = opMode.hardwareMap;
    follower = Constants.createFollower(hardwareMap);

    // From https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    opMode.telemetry.setMsTransmissionInterval(11);

    bl = hardwareMap.dcMotor.get("bl");
    br = hardwareMap.dcMotor.get("br");
    fl = hardwareMap.dcMotor.get("fl");
    fr = hardwareMap.dcMotor.get("fr");

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

    // Init Subsystems
    outtake = new Outtake(opMode);
    intake = new Intake(opMode);

    limelight = new Limelight(opMode, this.getAllianceColor());
  }

  public void initAuton() {
    this.outtake.setBase();
  }

  public AllianceColor getAllianceColor() {
    return this.allianceColor;
  }

  public void updateAutoControls() {
    follower.update();
    outtake.updatePIDControl();
  }
}

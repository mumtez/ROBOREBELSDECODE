package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public enum AllianceColor {
  RED {
    @Override
    public Pose poseFromArray(double[] arr) {
      return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
    }

    @Override
    public int getLLPipelineTeleOP() {
      return 1;
    }

    @Override
    public int getAimPose() {
      return -4;
    } //-3
  },
  BLUE {
    @Override
    public Pose poseFromArray(double[] arr) {

      //TODO: latest pedro made mirror use 141.5 field length
      // overwriting to 144 which was the pre-patch value to retain your previous tuning
      return new Pose(arr[0], arr[1], Math.toRadians(arr[2])).mirror(144);
    }

    @Override
    public int getLLPipelineTeleOP() {
      return 2; // TODO: make these in LL
    }

    @Override
    public int getAimPose() {
      return -2;
    } //-1
  };

  public abstract Pose poseFromArray(double[] arr);

  public abstract int getLLPipelineTeleOP();

  public abstract int getAimPose();
}
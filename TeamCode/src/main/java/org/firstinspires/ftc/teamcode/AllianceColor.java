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
    }
  },
  BLUE {
    @Override
    public Pose poseFromArray(double[] arr) {
      return new Pose(arr[0], arr[1], Math.toRadians(arr[2])).mirror();
    }

    @Override
    public int getLLPipelineTeleOP() {
      return 2; //TODO make these in LL
    }

    @Override
    public int getAimPose() {
      return -3;
    }
  };

  public abstract Pose poseFromArray(double[] arr);

  public abstract int getLLPipelineTeleOP();

  public abstract int getAimPose();
}
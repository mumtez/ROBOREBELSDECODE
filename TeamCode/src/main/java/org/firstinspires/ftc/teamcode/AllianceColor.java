package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public enum AllianceColor {
  RED {
    @Override
    public Pose poseFromArray(double[] arr) {
      return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
    }

    @Override
    public int getLLPipelineAuto(){
      return 0;
    }
    @Override
    public int getLLPipelineTeleOP(){
      return 2;
    }
  },
  BLUE {
    @Override
    public Pose poseFromArray(double[] arr) {
      return new Pose(arr[0], arr[1], Math.toRadians(arr[2])).mirror();
    }

    @Override
    public int getLLPipelineAuto(){
      return 1; // TODO add Tag filtering
    }
    @Override
    public int getLLPipelineTeleOP(){
      return 3; //TODO make these in LL
    }
  };

  public abstract Pose poseFromArray(double[] arr);
  public abstract int getLLPipelineAuto();
  public abstract int getLLPipelineTeleOP();
}
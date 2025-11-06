package org.firstinspires.ftc.teamcode.Subsystems;

public enum BallColor {
  GREEN(0.5),
  PURPLE(0.7),
  NONE(0.0);

  private final double rgbPos;

  BallColor(double rgbPos) {
    this.rgbPos = rgbPos;
  }

  public double getRgbPos() {
    return rgbPos;
  }
}

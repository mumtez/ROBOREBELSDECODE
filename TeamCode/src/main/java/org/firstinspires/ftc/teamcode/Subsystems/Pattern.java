package org.firstinspires.ftc.teamcode.Subsystems;

public enum Pattern {
  GPP, PGP, PPG;

  int numCyclesToGPP() {
    switch (this) {
      case GPP: return 0;
      case PGP: return 1;
      case PPG: return 2;
    }
    return 0;
  }

  int numCyclesToPGP() {
    switch (this) {
      case GPP: return 2;
      case PGP: return 0;
      case PPG: return 1;
    }
    return 0;
  }

  int numCyclesToPPG() {
    switch (this) {
      case GPP: return 1;
      case PGP: return 2;
      case PPG: return 0;
    }
    return 0;
  }

  public int numCyclesFromPattern(Pattern p) {
    switch (this) {
      case GPP: return p.numCyclesToGPP();
      case PGP: return p.numCyclesToPGP();
      case PPG: return p.numCyclesToPPG();
    }
    return 0;
  }

}
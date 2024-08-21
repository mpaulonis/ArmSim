package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int MOTOR_PORT = 0;
  public static final int ENCODER_A_CHAN = 0;
  public static final int ENCODER_B_CHAN = 1;
  public static final int JOYSTICK_PORT = 0;

  public static final String ARM_POSITION_KEY = "ArmPosition";

  public static final String ARM_P_KEY = "ArmP";
  public static final String ARM_D_KEY = "ArmD";
  public static final String ARM_TRAP_V_KEY = "ArmTrapV";
  public static final String ARM_TRAP_A_KEY = "ArmTrapA";

  public static final double ARM_KP_INIT = 50.0;
  public static final double ARM_KD_INIT = 0.0;
  public static final double ARM_TRAP_VMAX_INIT = 1.8; // rad/sec
  public static final double ARM_TRAP_AMAX_INIT = 4.0; // rad/sec*sec

  // https://www.reca.lc/arm?armMass=%7B%22s%22%3A10%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A20%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A30%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%20Vortex%2A%22%7D&ratio=%7B%22magnitude%22%3A355.56%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-31%2C%22u%22%3A%22deg%22%7D
  public static final double ARM_FFG_INIT = 0.19; // volts
  public static final double ARM_FFV_INIT = 6.01; // volt*sec/rad
  public static final double ARM_FFA_INIT = 0.01; // volt*sec*sec/rad

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double ARM_ENCODER_DIST_PER_PULSE = 2.0 * Math.PI / 4096;

  // arm gear reduction - planetary with 5/5/4 stages
  // and chain sprockets of 64 driven, 18 driving
  public static final double ARM_REDUCTION = 5 * 5 * 4 * (64 / 18);
  public static final double ARM_MASS = Units.lbsToKilograms(10);
  public static final double ARM_LENGTH = Units.inchesToMeters(20);
  public static final double MIN_ANGLE_RAD = Units.degreesToRadians(-34);
  public static final double MAX_ANGLE_RAD = Units.degreesToRadians(100);
  public static final double INIT_ANGLE_DEG = -34.0;
  public static final double LOAD_ANGLE_DEG = -31.0;
  public static final double SHOOT_ANGLE_DEG = 16.0;
  public static final double AMP_ANGLE_DEG = 86.0;
  public static final double ARM_ANGLE_TOLERANCE_DEG = 0.25;
}

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public Boolean armClosedLoop = false;
  private Double angleGoalRad = Units.degreesToRadians(Constants.LOAD_ANGLE_DEG);
  private Double pidOutput = 0.0;
  private Double feedforwardOutput = 0.0;

  // Initial tuning is hardcoded as constants,
  // but some can be changed via the dashboard
  private double armKp = Constants.ARM_KP_INIT;
  private double armKd = Constants.ARM_KD_INIT;
  private double armVmax = Constants.ARM_TRAP_VMAX_INIT;
  private double armAmax = Constants.ARM_TRAP_AMAX_INIT;
  private double armFFG = Constants.ARM_FFG_INIT;
  private double armFFV = Constants.ARM_FFV_INIT;
  private double armFFA = Constants.ARM_FFA_INIT;

  // Control the arm with a trapezoidal profile PID controller plus feedforward
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          armKp, 0.0, armKd, new TrapezoidProfile.Constraints(armVmax, armAmax));
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, armFFG, armFFV, armFFA);

  // Drive the arm with a NEO Vortex
  private final DCMotor armGearbox = DCMotor.getNeoVortex(1);
  private final PWMSparkFlex motor = new PWMSparkFlex(Constants.MOTOR_PORT);

  // Create a single-jointed arm as defined by all the constants
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armGearbox,
          Constants.ARM_REDUCTION,
          SingleJointedArmSim.estimateMOI(Constants.ARM_LENGTH, Constants.ARM_MASS),
          Constants.ARM_LENGTH,
          Constants.MIN_ANGLE_RAD,
          Constants.MAX_ANGLE_RAD,
          true,
          Units.degreesToRadians(Constants.INIT_ANGLE_DEG),
          // Add noise with a std-dev of 1/2 tick
          VecBuilder.fill(Constants.ARM_ENCODER_DIST_PER_PULSE / 2.0));

  // Simulate an encoder for measuring the arm angle
  private final Encoder encoder = new Encoder(Constants.ENCODER_A_CHAN, Constants.ENCODER_B_CHAN);
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d arm =
      armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  // Create an arm
  public Arm() {
    encoder.setDistancePerPulse(Constants.ARM_ENCODER_DIST_PER_PULSE);
    pid.setTolerance(Units.degreesToRadians(Constants.ARM_ANGLE_TOLERANCE_DEG));

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", mech2d);
    armTower.setColor(new Color8Bit(Color.kBlue));

    // Set tuning values to preferences
    Preferences.initDouble(Constants.ARM_P_KEY, armKp);
    Preferences.initDouble(Constants.ARM_D_KEY, armKd);
    Preferences.initDouble(Constants.ARM_TRAP_V_KEY, armVmax);
    Preferences.initDouble(Constants.ARM_TRAP_A_KEY, armAmax);
  }

  @Override
  public void periodic() {
    // Run controllers
    if (armClosedLoop) {
      pidOutput = pid.calculate(encoder.getDistance(), angleGoalRad);
      feedforwardOutput =
          feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
      motor.setVoltage(pidOutput + feedforwardOutput);

    } else {
      pid.reset(encoder.getDistance());
    }

    // Log outputs from feedforward and PID controllers
    SmartDashboard.putNumber("FFOutput", feedforwardOutput);
    SmartDashboard.putNumber("PIDOutput", pidOutput);
    SmartDashboard.putNumber("atGoal", (pid.atGoal()) ? 1.0 : -1.0);
  }

  @Override
  public void simulationPeriodic() {
    // Provide the simulation with the voltage being requested by the controllers
    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());

    // Do a little logging
    SmartDashboard.putNumber("MotorVoltage", motor.get() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("MotorCurrent", armSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("internalSPDeg", Units.radiansToDegrees(pid.getSetpoint().position));
    SmartDashboard.putNumber("internalSPVel", pid.getSetpoint().velocity);

    // Integrate the simulation model at the
    // default 0.02 second timestep (50Hz) used by the roboRio
    armSim.update(0.020);

    // Update the simulated encoder and the battery voltage under load
    encoderSim.setDistance(armSim.getAngleRads());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    // Update the 2d mechanism
    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  // Set the goal for the arm angle. Put controller in auto if not already
  public void setGoalDeg(double setpointDeg) {
    armClosedLoop = true;
    angleGoalRad =
        Units.degreesToRadians(
            MathUtil.clamp(
                setpointDeg,
                Units.radiansToDegrees(Constants.MIN_ANGLE_RAD),
                Units.radiansToDegrees(Constants.MAX_ANGLE_RAD)));
  }

  // Stop the arm
  public void stop() {
    motor.set(0.0);
    armClosedLoop = false;
  }

  // Load tuning constants from preferences
  public void loadPreferences() {
    if (armKp != Preferences.getDouble(Constants.ARM_P_KEY, armKp)) {
      armKp = Preferences.getDouble(Constants.ARM_P_KEY, armKp);
      pid.setP(armKp);
    }
    if (armKd != Preferences.getDouble(Constants.ARM_D_KEY, armKd)) {
      armKd = Preferences.getDouble(Constants.ARM_D_KEY, armKd);
      pid.setD(armKd);
    }
    if (armVmax != Preferences.getDouble(Constants.ARM_TRAP_V_KEY, armVmax)) {
      armVmax = Preferences.getDouble(Constants.ARM_TRAP_V_KEY, armVmax);
      pid.setConstraints(new TrapezoidProfile.Constraints(armVmax, armAmax));
    }
    if (armAmax != Preferences.getDouble(Constants.ARM_TRAP_A_KEY, armAmax)) {
      armAmax = Preferences.getDouble(Constants.ARM_TRAP_A_KEY, armAmax);
      pid.setConstraints(new TrapezoidProfile.Constraints(armVmax, armAmax));
    }
  }
}

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  private final Arm arm = new Arm();

  private final CommandXboxController controller =
      new CommandXboxController(Constants.JOYSTICK_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    controller.a().onTrue(Commands.runOnce(() -> arm.setGoalDeg(Constants.LOAD_ANGLE_DEG)));

    controller.b().onTrue(Commands.runOnce(() -> arm.setGoalDeg(Constants.SHOOT_ANGLE_DEG)));

    controller.y().onTrue(Commands.runOnce(() -> arm.setGoalDeg(Constants.AMP_ANGLE_DEG)));

    controller.x().onTrue(Commands.runOnce(() -> arm.stop()));
  }

  public void disableInitialize() {
    arm.armClosedLoop = false;
  }

  public void autoInitialize() {
    arm.armClosedLoop = true;
  }

  public void teleopInitialize() {
    arm.loadPreferences();
    arm.armClosedLoop = true;
  }
}

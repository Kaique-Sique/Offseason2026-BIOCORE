// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord7563.Robot;

import br.megazord7563.Robot.Constants.RobotConstants;
import br.megazord7563.Robot.subsystems.swerve.GyroIO;
import br.megazord7563.Robot.subsystems.swerve.ModuleIOSim;
import br.megazord7563.Robot.subsystems.swerve.SwerveModule;
import br.megazord7563.Robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  public static SwerveSubsystem m_swerveDrive;

  public static final CommandXboxController driverJoystick = new CommandXboxController(0);

  public RobotContainer() {
    switch (RobotConstants.robotMode) {
      case SIM:
        m_swerveDrive = new SwerveSubsystem(new SwerveModule(new ModuleIOSim(), "FL"), 
                                            new SwerveModule(new ModuleIOSim(), "FR"), 
                                            new SwerveModule(new ModuleIOSim(), "BL"),  
                                            new SwerveModule(new ModuleIOSim(), "BR"),  
                                            new GyroIO() {});
        break;
      default:
        break;
    }

    m_swerveDrive.setDefaultCommand(new RunCommand(
        () -> m_swerveDrive.driveFieldOriented(
            () -> MathUtil.applyDeadband(driverJoystick.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(driverJoystick.getLeftX(), 0.1),
            () -> -MathUtil.applyDeadband(driverJoystick.getRightX(), 0.1),
            () -> driverJoystick.rightStick().getAsBoolean()),
        m_swerveDrive));

    configureBindings();
  }

  /** Use this method to define your trigger->command mappings. Triggers can be created via the
   */
  private void configureBindings() 
  {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

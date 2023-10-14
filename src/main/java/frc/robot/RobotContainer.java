// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  public static Drivetrain m_drivetrain = new Drivetrain();

  public RobotContainer() {
    configureBindings();
    CommandXboxController m_driverController = new CommandXboxController(0);
    
    // m_drivetrain.setDefaultCommand(
    //   new RunCommand(() -> m_drivetrain.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX()), m_drivetrain));
    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> m_drivetrain.driveVoltsTank(-m_driverController.getLeftY(), -m_driverController.getRightX()), m_drivetrain));
  }
  

  private void configureBindings() {} 

  public Command getAutonomousCommand() {
    return m_drivetrain.followTrajectoryCommand("Test");

  }
}

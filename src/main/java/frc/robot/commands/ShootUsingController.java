// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootUsingController extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private CommandXboxController m_controller;
  
  public static final double DEADBAND = 0.1;
  private static final double SHOOTERSPEED = 1.0;
  /** Creates a new ShootUsingController. */
  public ShootUsingController(ShooterSubsystem subsystem, CommandXboxController controller) {
    m_shooterSubsystem = subsystem;
    m_controller = controller;
    addRequirements(m_shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -m_controller.getHID().getLeftY();
    speed = MathUtil.applyDeadband(speed * SHOOTERSPEED, DEADBAND);
    double voltage = speed * Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
    m_shooterSubsystem.runFeedMotor(voltage);
    m_shooterSubsystem.runLaunchMotor(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

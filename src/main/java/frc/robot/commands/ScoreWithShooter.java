// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreWithShooter extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private final double SHOOTER_SPEED;

  /** Creates a new ShootUsingController. */
  public ScoreWithShooter(ShooterSubsystem subsystem, double speed) {
    m_shooterSubsystem = subsystem;
    this.SHOOTER_SPEED = speed;
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
    double voltage = SHOOTER_SPEED * Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
    m_shooterSubsystem.runFeedMotor(voltage);
    m_shooterSubsystem.runLaunchMotor(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

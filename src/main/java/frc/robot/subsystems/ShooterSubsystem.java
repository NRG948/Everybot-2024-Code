// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ShootUsingController;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */

  private boolean isEnabled = false;
  private double feedMotorPower;
  private double launchMotorPower;
  private double shooterPower; // % output

  static final double FEEDER_OUT_SPEED = 1.0; // % output
  static final double FEEDER_IN_SPEED = -.4; // % output
  static final double LAUNCHER_SPEED = 1.0; // % output

  private final CANSparkBase launchWheel = new CANSparkMax(Constants.RobotConstants.CAN.SparkMax.SHOOTER_PORT, MotorType.kBrushless);
  private final CANSparkBase feedWheel = new CANSparkMax(Constants.RobotConstants.CAN.SparkMax.INDEXER_PORT, MotorType.kBrushless);

  public ShooterSubsystem() {
    launchWheel.setIdleMode(IdleMode.kBrake);
    feedWheel.setIdleMode(IdleMode.kBrake);

    feedWheel.setInverted(true); // TODO check spin direction
    launchWheel.setInverted(true); // TODO check spin direction
  }

  /**
   * Runs the launch wheel motor
   * 
   * @param power
   */
  public void runLaunchMotor(double power) {
    isEnabled = true;
    launchWheel.setVoltage(power);
  }

  /**
   * Runs the motor
   * 
   * @param power
   */
  public void runFeedMotor(double power) {
    isEnabled = true;
    shooterPower = power / Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
    feedWheel.set(power);
  }

  /**
   * Stops the launch wheel motor
   */
  public void stopLaunchMotor() {
    launchWheel.stopMotor();
  }

  /**
   * Stops the launch wheel motor
   */
  public void stopFeedMotor() {
    feedWheel.stopMotor();
  }

  /**
   * Stops ALL motors
   */
  public void stopAllMotors() {
    stopFeedMotor();
    stopLaunchMotor();
  }

  /**
   * Disable the intake.
   */
  public void disable() {
    isEnabled = false;
    stopAllMotors();
  }

  @Override
  public void periodic() {
    if (isEnabled) {
      runFeedMotor(feedMotorPower);
      runLaunchMotor(launchMotorPower);
    }
  }

  public void addShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    
    ShuffleboardLayout layout = tab.getLayout("Shooter", BuiltInLayouts.kList)
      .withPosition(0, 0)
      .withSize(2, 3);

    layout.addDouble("Current Power", () -> shooterPower);
    layout.addDouble("Feeder In Power", () -> FEEDER_IN_SPEED);
    layout.addDouble("Feeder Out Power", () -> FEEDER_OUT_SPEED);
    layout.addDouble("Launcher Power", () -> LAUNCHER_SPEED);
    layout.addDouble("Deadband", () -> ShootUsingController.DEADBAND);
  }
}

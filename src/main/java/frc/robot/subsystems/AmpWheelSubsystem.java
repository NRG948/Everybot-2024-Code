// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpWheelSubsystem extends SubsystemBase {
  private boolean isEnabled = false;
  private CANSparkFlex wheel = new CANSparkFlex(Constants.RobotConstants.CAN.SparkMax.INTAKE_PORT, MotorType.kBrushless);

  /** Creates a new AmpWheel. */
  public AmpWheelSubsystem() {
    wheel.setIdleMode(IdleMode.kBrake);

  }

  /**
   * Runs the launch wheel motor
   * 
   * @param power
   */
  public void runLaunchMotor(double power) {
    isEnabled = true;
    wheel.setVoltage(power);
  }

  /**
   * Runs the motor
   * 
   * @param power
   */
  public void runMotor(double power) {
    isEnabled = true;
    wheel.setVoltage(power);
  }

  /**
   * Stops the launch wheel motor
   */
  public void stopMotor() {
    wheel.stopMotor();
  }

  /**
   * Disable the intake.
   */
  public void disable() {
    isEnabled = false;
    stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isEnabled) {
      runMotor(1.0);
    }
  }
}

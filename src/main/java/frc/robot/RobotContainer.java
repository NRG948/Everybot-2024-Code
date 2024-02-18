// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.OperatorConstants;
import frc.robot.commands.DriveUsingController;
import frc.robot.drive.SwerveDrive;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public class RobotContainer {

    public static final Subsystems m_subsystems = new Subsystems();

    // Robot autonomous must be initialized after the subsystems
    private final RobotAutonomous m_autonomous = new RobotAutonomous(m_subsystems);

    // Instance variables for controllers
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.XboxControllerPort.DRIVER);

    private final CommandXboxController m_manipulatorController = new CommandXboxController(
            OperatorConstants.XboxControllerPort.MANIPULATOR);

    public RobotContainer() {

        m_subsystems.drivetrain.setDefaultCommand(new DriveUsingController(m_subsystems, m_driverController));
        m_subsystems.drivetrain.setDefaultCommand(new DriveUsingController(m_subsystems, m_manipulatorController));

        // Configure the trigger bindings
        configureBindings();

        initShuffleboard();
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return m_autonomous.getAutonomousCommand();
    }

    public void periodic() {
        m_subsystems.periodic();
    }

    private void initShuffleboard() {
        ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");
        m_autonomous.addShuffleboardLayout(operatorTab);
        m_subsystems.drivetrain.addShuffleboardTab();
        m_subsystems.shooterSubsystem.addShuffleboardTab();
    }
}

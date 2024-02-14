// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public class RobotContainer {

    private final Subsystems m_subsystems = new Subsystems();

    // Instance variables for controllers
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.XboxControllerPort.DRIVER);

    private final CommandXboxController m_manipulatorController = new CommandXboxController(
            OperatorConstants.XboxControllerPort.MANIPULATOR);

    public RobotContainer() {

        m_subsystems.drivetrain.setDefaultCommand(new DriveUsingController(m_subsystems, m_driverController));

        // Configure the trigger bindings
        configureBindings();

        initShuffleboard();
    }
    private void ConfigureBindings()



    
}

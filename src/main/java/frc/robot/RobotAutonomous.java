// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.nrg948.preferences.RobotPreferences.BooleanValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.OperatorConstants;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ScoreWithShooter;
import frc.robot.subsystems.Subsystems;

/**
 * This class creates and manages the user interface operators used to select
 * and configure autonomous routines.
 */
public class RobotAutonomous {
    public static BooleanValue enableTesting = new BooleanValue("Autonomous", "enableTesting", false);

    private static SendableChooser<ChooseAutoPath> chooseAutoPath = new SendableChooser<>();

    public static enum ChooseAutoPath {
        NONE,
        RIGHT_SUBWOOFER_SHOT_DRIVE_OUT
    }

    /**
     * Creates a new RobotAutonomous.
     * 
     * @param subsystems The subsystems container.
     */
    public RobotAutonomous(Subsystems subsystems) {
        AutoBuilder.configureHolonomic(
                subsystems.drivetrain::getPosition,
                subsystems.drivetrain::resetPosition,
                subsystems.drivetrain::getChassisSpeeds,
                subsystems.drivetrain::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(1.0, 0, 0),
                        new PIDConstants(1.0, 0, 0),
                        subsystems.drivetrain.getMaxSpeed(),
                        subsystems.drivetrain.getWheelBaseRadius(),
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                subsystems.drivetrain);
    }

    private static Translation2d RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_FIRST_POINT = new Translation2d(0.76, 4.53);
    private static Translation2d RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_SECOND_POINT = new Translation2d(2.47, 0.55);

    private static Rotation2d RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_FIRST_ORIENTATION = Rotation2d.fromDegrees(1.90);
    private static Rotation2d RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_SECOND_ORIENTATION = Rotation2d.fromDegrees(-78.41);

    private static Pose2d START_POINT = new Pose2d(RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_FIRST_POINT,
            RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_FIRST_ORIENTATION);
    private static Pose2d END_POINT = new Pose2d(RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_SECOND_POINT,
            RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_SECOND_ORIENTATION);

    public static CommandXboxController mCommandXboxController = new CommandXboxController(
            OperatorConstants.XboxControllerPort.DRIVER);

    public static Command getAutonomousCommand() {
        Command autoCommand = getSelectedAutonomousCommand();
        return autoCommand;
    }

    private static Command getSelectedAutonomousCommand() {
        switch (chooseAutoPath.getSelected()) {
            case NONE:
                return new InstantCommand(() -> System.out.println("NO AUTO PATH SELECTED"));

            case RIGHT_SUBWOOFER_SHOT_DRIVE_OUT:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> RobotContainer.m_subsystems.drivetrain.resetOrientation()),
                        new InstantCommand(
                                () -> System.out.println(RobotContainer.m_subsystems.drivetrain.getOrientation())),
                        new WaitCommand(3),
                        new PrintCommand("hello"),
                        // new ScoreWithShooter(RobotContainer.m_subsystems.shooterSubsystem, 0.8), // TODO: Find actual speed.
                        new DriveStraight(RobotContainer.m_subsystems.drivetrain,
                                RIGHT_SUBWOOFER_SHOT_DRIVE_OUT_SECOND_POINT, 1.0) // TODO: Find actual max speed.
                );

            default:
                return new InstantCommand(() -> System.out.println("Error"));
        }
    }

    /**
     * Adds the autonomous layout to the shuffleboard tab.
     * 
     * @param tab The tab to add the layout.
     */
    public void addShuffleboardLayout(ShuffleboardTab tab) {

        ShuffleboardLayout autoLayout = tab.getLayout("Autonomous", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 2);

        chooseAutoPath.setDefaultOption("None", ChooseAutoPath.NONE);

        chooseAutoPath.addOption("Right Subwoofer Shot Drive Out", ChooseAutoPath.RIGHT_SUBWOOFER_SHOT_DRIVE_OUT);

        autoLayout.add("Routine", chooseAutoPath).withWidget(BuiltInWidgets.kComboBoxChooser);
    }
}

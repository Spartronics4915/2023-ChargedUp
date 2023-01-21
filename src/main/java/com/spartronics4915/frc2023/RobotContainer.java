// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023;

import com.spartronics4915.frc2023.commands.Autos;
//import com.spartronics4915.frc2023.Constants.Swerve;
import com.spartronics4915.frc2023.commands.SwerveCommands;
import com.spartronics4915.frc2023.subsystems.Swerve;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static com.spartronics4915.frc2023.Constants.OI.*;
import static com.spartronics4915.frc2023.Constants.Autos.*;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController mController;

    // The robot's subsystems and commands are defined here...
    // private final com.spartronics4915.frc2023.subsystems.Swerve mSwerve;
    // private final SwerveCommands mSwerveCommands; 
    private final Command mAutonomousCommand;
    // private final Command mTeleopInitCommand;
    // private final Command mTeleopCommand;
    // private final Command mTestingCommand;
        private final Command mAutos;
        private final Swerve mSwerve;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        mController = new XboxController(kControllerID);

        // mTestSubsystem = new TestSubsystem();

        mSwerve = new Swerve();

        // mTestCommand = new TestCommands(mController, mTestSubsystem);

        // mSwerveCommands = new SwerveCommands(mController, mSwerve);

        mAutonomousCommand = Autos.driveStraight(mSwerve, 1.0);

        // mTeleopInitCommand = mSwerveCommand.new TeleopInitCommand();
        // mTeleopCommand = mSwerveCommands.new TeleopCommand();
        // mTestingCommand = mSwerveCommands.new TestCommand();

        

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // new JoystickButton(mController, kToggleFieldRelativeButton)
            // .whenPressed(mSwerveCommands.new ToggleFieldRelative());

        // new JoystickButton(mController, kResetYawButton)
            // .whenPressed(mSwerveCommands.new ResetYaw());

        // new JoystickButton(mController, kResetOdometryButton)
            // .whenPressed(mSwerveCommands.new ResetOdometry());

        new JoystickButton(mController, 1)
            .onTrue(mAutonomousCommand); //why not work???
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return mAutonomousCommand;
    }

    // public Command getTeleopInitCommand() {
        // return mTeleopInitCommand;
    // }

    // public Command getTeleopCommand() {
        // return mTeleopCommand;
    }

    // public Command getTestingCommand() {
        // return mTestingCommand;
    // }
// }

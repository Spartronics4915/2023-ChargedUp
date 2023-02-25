// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023;

import com.spartronics4915.frc2023.commands.ArmCommands;
import com.spartronics4915.frc2023.commands.Autos;
import com.spartronics4915.frc2023.commands.ChargeStationCommands;
import com.spartronics4915.frc2023.commands.DebugTeleopCommands;
import com.spartronics4915.frc2023.commands.SwerveCommands;
import com.spartronics4915.frc2023.commands.SwerveTrajectoryFollowerCommands;
import com.spartronics4915.frc2023.commands.ChargeStationCommands.AutoChargeStationClimb.ClimbState;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem;
import com.spartronics4915.frc2023.subsystems.Intake;
import com.spartronics4915.frc2023.subsystems.Swerve;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static com.spartronics4915.frc2023.Constants.OI.*;

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
    // private final XboxController mDriverController;
    // private final XboxController mOperatorController;
    
    private final CommandXboxController mDriverController;
    private final CommandXboxController mOperatorController;
    
    private final SwerveTrajectoryFollowerCommands mSwerveTrajectoryFollowerCommands;
    
    // The robot's subsystems and commands are defined here...
    private final Swerve mSwerve;
    private final SwerveCommands mSwerveCommands;
    
    private final ArmSubsystem mArm;
    private final ArmCommands mArmCommands;
    private final Intake mIntake;
    
    // private final Intake mIntake;
    // private final IntakeCommands mIntakeCommands;
    
    private final Autos mAutos;
    
    private final Command mAutonomousCommand;
    private final Command mTeleopInitCommand;
    
    private final boolean useJoystick = true;
    private final boolean useSwerveChassis = false;
    private final boolean useArm = true;
    // private final Command mTestingCommand;
    
    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
        mDriverController = useJoystick ? new CommandXboxController(kDriverControllerID) : null;
        mOperatorController = useJoystick ? new CommandXboxController(kOperatorControllerID) : null;
        
        if (useSwerveChassis) {
            mSwerve = Swerve.getInstance();
            mSwerveCommands = new SwerveCommands(mDriverController);
            mSwerve.setDefaultCommand(mSwerveCommands.new TeleopCommand());
            
            mSwerveTrajectoryFollowerCommands = new SwerveTrajectoryFollowerCommands();
            mAutos = new Autos(mSwerveTrajectoryFollowerCommands);
            
            mAutonomousCommand = new SequentialCommandGroup(
            mSwerveCommands.new ResetCommand(),
            mAutos.new MoveForwardCommandFancy()
            );
            
            mTeleopInitCommand = mSwerveCommands.new ResetCommand();
        }
        else {
            mAutonomousCommand = null;
            mTeleopInitCommand = null;
            mAutos = null;
            mSwerve = null;
            mSwerveCommands = null;
            mSwerveTrajectoryFollowerCommands = null;
        }
        
        if (useArm) {
            mArm = ArmSubsystem.getInstance();
            mArmCommands = new ArmCommands(mArm);
            mIntake = Intake.getInstance();
            
        }
        // mIntake = Intake.getInstance();
        // mIntakeCommands = new IntakeCommands(mIntake);
        
        
        // Configure the button bindings
        configureButtonBindings();
    }
    
    private void configureButtonBindings() {
        if (useJoystick) {
            // DRIVER CONTROLS
            
            if(useSwerveChassis) {
            mDriverController.a()
            .onTrue(mSwerveCommands.new ToggleFieldRelative());
            
            mDriverController.b()
            .onTrue(mSwerveCommands.new ResetYaw());
            
            mDriverController.y()
            .onTrue(mSwerveCommands.new ResetOdometry());
            
            mDriverController.rightTrigger(kTriggerDeadband)
            .onTrue(mSwerveCommands.new EnableSprintMode())
            .onFalse(mSwerveCommands.new DisableSprintMode());
            
            mDriverController.rightBumper()
            .whileTrue(new ChargeStationCommands.AutoChargeStationClimb(mSwerve));
            
            mDriverController.leftBumper()
            .whileTrue(new ChargeStationCommands.AutoChargeStationClimb(mSwerve, ClimbState.LEVEL_ROBOT_SETUP));
            }
            mDriverController.povRight().whileTrue(mArm.getExtender().getExtendCommand());
            mDriverController.povLeft().whileFalse(mArm.getExtender().getRetractCommand());

            mOperatorController.rightTrigger(kTriggerDeadband).whileTrue(Commands.runEnd(()->mIntake.setState(IntakeState.OUT),
            ()->mIntake.setState(IntakeState.OFF)));

            // OPERATOR CONTROLS
            // mOperatorController.povUp()
            //     .onTrue(mArmCommands.new SetArmState(ArmState.GRAB_UPRIGHT));
            
            // mOperatorController.povDown()
            //     .onTrue(mArmCommands.new SetArmState(ArmState.GRAB_FALLEN));
            
            // mOperatorController.b()
            //     .onTrue(mArmCommands.new SetArmState(ArmState.RETRACTED));
            
            // mOperatorController.a()
            //     .onTrue(mArmCommands.new SetArmState(ArmState.LEVEL_1));
            
            // mOperatorController.x()
            //     .onTrue(mArmCommands.new SetArmState(ArmState.LEVEL_2));
            
            // mOperatorController.y()
            //     .onTrue(mArmCommands.new SetArmState(ArmState.LEVEL_3));
            
            // mOperatorController.rightTrigger(kTriggerDeadband)
            //     .onTrue(mIntakeCommands.new SetIntakeState(IntakeState.OUT))
            //     .onFalse(mIntakeCommands.new SetIntakeState(IntakeState.OFF));
            
            // mOperatorController.leftTrigger(kTriggerDeadband)
            //     .onTrue(mIntakeCommands.new SetIntakeState(IntakeState.IN))
            //     .onFalse(mIntakeCommands.new SetIntakeState(IntakeState.OFF));
        }
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return mAutonomousCommand;
    }
    
    public Command getTeleopInitCommand() {
        return mTeleopInitCommand;
    }
    
    public Command getTestingCommand() {
        return null;
    }
    
    public void initRobot() {
        Command shuffleboard_update_command = new DebugTeleopCommands.ShuffleboardUpdateCommand(mArm, mArmCommands);
        shuffleboard_update_command.schedule();
        
        if (mSwerve != null) {
            mSwerve.resetToAbsolute();
            mSwerve.setFieldRelative(true);
        }
    }
}

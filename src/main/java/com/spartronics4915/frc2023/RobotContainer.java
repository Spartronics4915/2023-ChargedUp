// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023;

import static com.spartronics4915.frc2023.Constants.OI.kDriverControllerID;
import static com.spartronics4915.frc2023.Constants.OI.kOperatorControllerID;
import static com.spartronics4915.frc2023.Constants.OI.kTriggerDeadband;
import static com.spartronics4915.frc2023.Constants.OI.kWindowButtonId;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import com.pathplanner.lib.PathPoint;
import com.spartronics4915.frc2023.Constants.Arm;

import static com.spartronics4915.frc2023.Constants.OI.kMenuButtonId;

import com.spartronics4915.frc2023.Constants.OI;
import com.spartronics4915.frc2023.Constants.Trajectory;
import static com.spartronics4915.frc2023.Constants.Swerve.*;
import com.spartronics4915.frc2023.commands.ArmCommands;
import com.spartronics4915.frc2023.commands.Autos;
import com.spartronics4915.frc2023.commands.ChargeStationCommands;
import com.spartronics4915.frc2023.commands.ChargeStationCommands.AutoChargeStationClimb.ClimbState;
import com.spartronics4915.frc2023.commands.DebugTeleopCommands;
import com.spartronics4915.frc2023.commands.ExtenderCommands;
import com.spartronics4915.frc2023.commands.IntakeCommands;
import com.spartronics4915.frc2023.commands.SimpleAutos;
import com.spartronics4915.frc2023.commands.SwerveCommands;
import com.spartronics4915.frc2023.commands.SwerveTrajectoryFollowerCommands;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem;
import com.spartronics4915.frc2023.subsystems.Intake;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;
import com.spartronics4915.frc2023.subsystems.Swerve;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;

import com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    private final ExtenderCommands mExtenderCommands;

    private final IntakeCommands mIntakeCommands;
    
    private final Autos mAutos;
    
	private final SendableChooser<Pose2d> mInitialPoseSelector = new SendableChooser<>();
	private final SendableChooser<Function<Pose2d, CommandBase>> mAutoSelector = new SendableChooser<>();
	private final Command mTeleopInitCommand;
    
    private final boolean useJoystick = true;
    private final boolean useSwerveChassis = true;
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
            mAutos = new Autos(mSwerveCommands, mSwerveTrajectoryFollowerCommands);
            
            mTeleopInitCommand = mSwerveCommands.new ResetCommand(new Pose2d()); // remove before comp as pose will be unknown after auto
        } else {
            mTeleopInitCommand = null;
            mAutos = null;
            mSwerve = null;
            mSwerveCommands = null;
            mSwerveTrajectoryFollowerCommands = null;
        }
        
        if (useArm) {
            mArm = ArmSubsystem.getInstance();
            mExtenderCommands = new ExtenderCommands(mArm.getExtender());
            mIntake = Intake.getInstance();
            mIntakeCommands = new IntakeCommands(mIntake);
            mArmCommands = new ArmCommands(mArm, mIntakeCommands);
        } else {
            mArm = null;
            mExtenderCommands = null;
            mIntake = null;
            mIntakeCommands = null;
            mArmCommands = null;
        }

        configureAutoSelector();
		configureInitialPoseSelector();
        
        // Configure the button bindings
        configureButtonBindings();
    }

	private void configureInitialPoseSelector() {
		for (InitialPose entry : kInitialPoses)
			mInitialPoseSelector.addOption(entry.name, entry.pose);
			
		mInitialPoseSelector.setDefaultOption(
			kInitialPoses[kDefaultInitialPoseIndex].name,
			kInitialPoses[kDefaultInitialPoseIndex].pose
		);

		SmartDashboard.putData("Initial Pose", mInitialPoseSelector);
	}

	private void configureAutoSelector() {
		Autos.Strategy[] autoStrategies = {
			mAutos.new Strategy(
				"Drop, Balance",
				(Pose2d initialPose) -> new SequentialCommandGroup(		
					mArmCommands.new ReleasePiece(ArmState.FLOOR_POS),
					new ChargeStationCommands.AutoChargeStationClimb()
				)
			),
            // mAutos.new Strategy(
            //     "Drop, Leave, Pick-up",
            //     (Pose2d initialPose) -> new SequentialCommandGroup(
			// 		mArmCommands.new ReleasePiece(ArmState.FLOOR_POS),
			// 		mSwerveTrajectoryFollowerCommands.new FollowStaticTrajectory(
			// 			new ArrayList<>(List.of(
			// 				new PathPoint(initialPose.getTranslation(), new Rotation2d(), initialPose.getRotation()),
			// 				new PathPoint(initialPose.getTranslation().plus(new Translation2d(Trajectory.kBackUpDistance, 0)), new Rotation2d(), new Rotation2d())
			// 			))
			// 		),
			// 		mArmCommands.new GrabPiece(ArmState.FLOOR_POS)
            //     )
            // ),
			mAutos.new Strategy(
				"Drop, Leave, Pick-up",
				(Pose2d initialPose) -> new SequentialCommandGroup(
					mArmCommands.new ReleasePiece(ArmState.FLOOR_POS),
					mSwerveTrajectoryFollowerCommands.new FollowStaticTrajectory(
						new ArrayList<>(List.of(
							new PathPoint(
								initialPose.getTranslation(),
								new Rotation2d(),
								initialPose.getRotation()
							),
							new PathPoint(
								initialPose.getTranslation().plus(new Translation2d(Trajectory.kBackUpDistance, 0)),
								new Rotation2d(),
								new Rotation2d()
							)
						))
					),
					mArmCommands.new GrabPiece(ArmState.FLOOR_POS)
				)

			)//,
			// mAutos.new Strategy(
			// 	"Move Forward Static (Test)",
			// 	(Pose2d initialPose) -> mAutos.new MoveForwardCommandFancy()
			// ),
			// mAutos.new Strategy(
			// 	"Move Forward Dynamic (Test)", (Pose2d initialPose) -> new InstantCommand(() -> {
			// 		mAutos.new MoveForwardCommandDynamic().schedule();
			// 	})
			// )
		};
		for (Autos.Strategy strat : autoStrategies) {
			mAutoSelector.addOption(strat.getName(), strat::getCommand);
		}

		mAutoSelector.setDefaultOption(
			autoStrategies[OI.kDefaultAutoIndex].getName(),
			autoStrategies[OI.kDefaultAutoIndex]::getCommand
		);
		SmartDashboard.putData("Auto Strategies", mAutoSelector);        
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
                
                mDriverController.rightBumper() // TODO: remove before comp
                .whileTrue(new ChargeStationCommands.AutoChargeStationClimb());
                
                mDriverController.leftBumper() // TODO: remove before comp
                .whileTrue(new ChargeStationCommands.AutoChargeStationClimb(ClimbState.LEVEL_ROBOT_SETUP));    
            }
            
            // OPERATOR CONTROLS
            // mOperatorController.button(7) //window
            //     .onTrue(mArmCommands.new SetArmState(ArmState.GRAB_UPRIGHT));
            
            // mOperatorController.povDown() //menu
            //     .onTrue(mArmCommands.new SetArmState(ArmState.GRAB_FALLEN));
            
           
            if (useArm){
                    /**
                    * presets button bindings:
                    * Extend to floor position		    Window
                    * Extend to Double substation		Menu
                    * Extend to Middle tier - cube		A
                    * Extend to Middle tier - cone		X
                    * Extend to High tier - cube		B
                    * Extend to High tier - cone		Y   
                    * */
                mOperatorController.button(kWindowButtonId) //should be window
                    .onTrue(mArmCommands.new SetArmLocalStateSimple(ArmState.FLOOR_POS));

                mOperatorController.button(kMenuButtonId) //should be menu
                    .onTrue(mArmCommands.new SetArmLocalStateSimple(ArmState.DOUBLE_SUBSTATION));
                
                mOperatorController.a()
                    .onTrue(mArmCommands.new SetArmLocalStateSimple(ArmState.CUBE_LEVEL_1));
                
                mOperatorController.x()
                    .onTrue(mArmCommands.new SetArmLocalStateSimple(ArmState.CONE_LEVEL_1));
                
                mOperatorController.b()
                    .onTrue(mArmCommands.new SetArmLocalStateSimple(ArmState.CUBE_LEVEL_2));
                
                mOperatorController.y()
                    .onTrue(mArmCommands.new SetArmLocalStateSimple(ArmState.CONE_LEVEL_2));

                /**
                 * Eject game piece		    RT
                 * Intake game piece		LT
                 */
                mOperatorController.rightTrigger(kTriggerDeadband)
                    .onTrue(mIntakeCommands.new SetIntakeState(IntakeState.CUBE_OUT))
                    .onFalse(mIntakeCommands.new SetIntakeState(IntakeState.OFF));
                
                mOperatorController.rightBumper()
                    .onTrue(mIntakeCommands.new SetIntakeState(IntakeState.CONE_OUT))
                    .onFalse(mIntakeCommands.new SetIntakeState(IntakeState.OFF));

                mOperatorController.leftTrigger(kTriggerDeadband)
                    .onTrue(mIntakeCommands.new SetIntakeState(IntakeState.IN))
                    .onFalse(mIntakeCommands.new SetIntakeState(IntakeState.OFF));

                /**
                 * Relative button bindings:
                 * Manual Pivot up		    D pad up
                 * Manual Pivot down		D pad down
                 * Manual Extend out		D pad left
                 * Manual Extend In		    D pad right
                 * Manual wrist up		    LS in
                 * Manual wrist down		RS in
                 */

                mOperatorController.povUp()
                    .whileTrue(mArmCommands.new TransformArmState(0, Arm.kTransformAmount, Rotation2d.fromDegrees(0)));

                mOperatorController.povDown()
                    .whileTrue(mArmCommands.new TransformArmState(0, Arm.kTransformAmount.unaryMinus(), Rotation2d.fromDegrees(0)));
                    
                mOperatorController.povLeft()
                    .whileTrue(mExtenderCommands.new Extend());

                mOperatorController.povRight()
                    .whileTrue(mExtenderCommands.new Retract());

                mOperatorController.leftStick()
                    .whileTrue(mArmCommands.new TransformArmState(0, Rotation2d.fromDegrees(0), Arm.kTransformAmount));

                mOperatorController.rightStick()
                    .whileTrue(mArmCommands.new TransformArmState(0, Rotation2d.fromDegrees(0), Arm.kTransformAmount.unaryMinus()));
                
            }

        }
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return mAutoSelector.getSelected().apply(mInitialPoseSelector.getSelected());
    }
    
    public Command getTeleopInitCommand() {
        return mTeleopInitCommand;
    }
    
    public Command getTestingCommand() {
        return null;
    }
    
    public void initRobot() {
        Command shuffleboard_update_command = new DebugTeleopCommands.ShuffleboardUpdateCommand(useArm, useSwerveChassis, mArm, mArmCommands, mSwerve, mSwerveCommands);
        shuffleboard_update_command.schedule();
        
        if (mSwerve != null) {
            mSwerve.resetToAbsolute();
            mSwerve.setFieldRelative(true);
        }
    }
}

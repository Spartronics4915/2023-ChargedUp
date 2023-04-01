// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023;

import static com.spartronics4915.frc2023.Constants.OI.kDriverControllerID;
import static com.spartronics4915.frc2023.Constants.OI.kOperatorControllerID;
import static com.spartronics4915.frc2023.Constants.OI.kTriggerDeadband;
import static com.spartronics4915.frc2023.Constants.OI.kWindowButtonId;

import java.util.function.Function;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.spartronics4915.frc2023.Constants.Arm;

import static com.spartronics4915.frc2023.Constants.OI.kMenuButtonId;
// import static com.spartronics4915.frc2023.commands.Autos.autoBuilder;

import com.spartronics4915.frc2023.Constants.OI;
import com.spartronics4915.frc2023.bling.CustomLEDPattern;

import static com.spartronics4915.frc2023.Constants.Swerve.*;
import static com.spartronics4915.frc2023.Constants.Bling.*;
import com.spartronics4915.frc2023.commands.ArmCommands;
import com.spartronics4915.frc2023.commands.Autos;
import com.spartronics4915.frc2023.commands.ChargeStationCommands;
import com.spartronics4915.frc2023.commands.DebugTeleopCommands;
import com.spartronics4915.frc2023.commands.ExtenderCommands;
import com.spartronics4915.frc2023.commands.IntakeCommands;
import com.spartronics4915.frc2023.commands.SwerveCommands;
import com.spartronics4915.frc2023.commands.SwerveTrajectoryFollowerCommands;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem;
import com.spartronics4915.frc2023.subsystems.BlingSubsystem;
import com.spartronics4915.frc2023.subsystems.Intake;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;
import com.spartronics4915.frc2023.subsystems.Swerve;

import com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    
    // The robot's subsystems and commands are defined here...
    private final BlingSubsystem mBlingSubsystem;
    
    private final Swerve mSwerve;
    private final SwerveCommands mSwerveCommands;
    private final SwerveTrajectoryFollowerCommands mSwerveTrajectoryFollowerCommands;
    
    private final ArmSubsystem mArm;
    private final ArmCommands mArmCommands;
    private final Intake mIntake;

    private final IntakeCommands mIntakeCommands;
    
    private final Autos mAutos;
    
	private final SendableChooser<Pose2d> mInitialPoseSelector = new SendableChooser<>();
	private final SendableChooser<Function<Pose2d, CommandBase>> mAutoSelector = new SendableChooser<>();
	private final Command mTeleopInitCommand;
    
    private final boolean useJoystick = true;
    private final boolean useSwerveChassis = true;
    private final boolean useArm = true;
    // private final Command mTestingCommand;

    private final Trigger mDSAttachedTrigger;
    
    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
        mDriverController = useJoystick ? new CommandXboxController(kDriverControllerID) : null;
        mOperatorController = useJoystick ? new CommandXboxController(kOperatorControllerID) : null;

        mBlingSubsystem = BlingSubsystem.getInstance();
        mDSAttachedTrigger = new Trigger(() -> DriverStation.isDSAttached());
        
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
            mIntake = Intake.getInstance();
            mIntakeCommands = new IntakeCommands(mIntake);
            mArmCommands = new ArmCommands(mArm, mIntakeCommands);
        } else {
            mArm = null;
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
			kInitialPoses[kDefaultInitialPoseIndex].pose);

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
            mAutos.new Strategy(
				"High cube, Balance",
				(Pose2d initialPose) -> new SequentialCommandGroup(		
					mArmCommands.new ReleasePiece(ArmState.SHOOT_HIGH_CUBE),
                    mArmCommands.getGoToPresetArmStateExtendFirstCommand(ArmState.TUCK_INTERMEDIATE, true),
					new ChargeStationCommands.AutoChargeStationClimb()
				)
			),
			mAutos.new Strategy(
                "do nothing",
                (Pose2d initialPose) -> new CommandBase() {}
            ),
            mAutos.new Strategy(
                "place, do nothing",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.new ReleasePiece(ArmState.FLOOR_POS)
                )
            ),
            mAutos.new Strategy(
                "high cube, do nothing",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.new ReleasePiece(ArmState.SHOOT_HIGH_CUBE)
                )
            ),
            mAutos.new Strategy(
                "place, leave community",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.new ReleasePiece(ArmState.FLOOR_POS),
                    mArmCommands.new SetArmPivotWristLocalState(ArmState.TUCK_INTERMEDIATE),
                    new WaitCommand(1),
                    mSwerve.driveCommand(new ChassisSpeeds(-2, 0, 0), false, true),
                    new WaitCommand(2.5),
                    mSwerve.driveCommand(new ChassisSpeeds(), false, true)
                )
            ),
            mAutos.new Strategy(
                "cube high, leave community",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.new ReleasePiece(ArmState.SHOOT_HIGH_CUBE),
                    mArmCommands.new SetArmPivotWristLocalState(ArmState.TUCK_INTERMEDIATE),
                    new WaitCommand(1),
                    mSwerve.driveCommand(new ChassisSpeeds(-2, 0, 0), false, true),
                    new WaitCommand(2.5),
                    mSwerve.driveCommand(new ChassisSpeeds(), false, true)
                )
            ),
            mAutos.new Strategy(
                "cone high, balance",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.getDunkCommand(),
                    mArmCommands.getTuckCommand(),
                    new ChargeStationCommands.AutoChargeStationClimb(false)
                )
            ),
            mAutos.new Strategy(
                "cone high, do nothing",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.getDunkCommand(),
                    mArmCommands.getTuckCommand()
                )
            ),
            mAutos.new Strategy(
                "cone high, leave community",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.getDunkCommand(),
                    mArmCommands.getTuckCommand(),
                    new WaitCommand(1),
                    mSwerve.driveCommand(new ChassisSpeeds(-2, 0, 0), false, true),
                    new WaitCommand(2.5),
                    mSwerve.driveCommand(new ChassisSpeeds(), false, true)
                )
            ),
            mAutos.new Strategy(
                "cube high (test)",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.new ReleasePiece(ArmState.SHOOT_HIGH_CUBE)
                )
            ),
            mAutos.new Strategy(
                "place, move short (test)",
                (Pose2d initialPose) -> new SequentialCommandGroup(
                    mArmCommands.new ReleasePiece(ArmState.FLOOR_POS),
                    mArmCommands.new SetArmPivotWristLocalState(ArmState.TUCK_INTERMEDIATE),
                    new WaitCommand(1),
                    mSwerve.driveCommand(new ChassisSpeeds(-0.5, 0, 0), false, true),
                    new WaitCommand(2),
                    mSwerve.driveCommand(new ChassisSpeeds(-1, 0, 0), false, true),
                    new WaitCommand(1),
                    mSwerve.driveCommand(new ChassisSpeeds(), false, true)
                )
            ),
            // mAutos.new Strategy(
            //     "follow test trajectory",
            //     (Pose2d initialPose) -> {
            //         initialPose = new Pose2d();
            //         var trajectory = PathPlanner.generatePath(
            //             new PathConstraints(2.5, 2.5), 
            //             new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d()),
            //             new PathPoint(new Translation2d(1, 1), new Rotation2d(), Rotation2d.fromDegrees(-90))
            //         );
            //         return new SequentialCommandGroup(
            //             new FollowSingleTrajectoryCommand(trajectory),
            //             mSwerve.driveCommand(new ChassisSpeeds(), true, true)
            //         );
            //     }
            // ),
            // mAutos.new Strategy(
            //     "follow straight 2m test trajectory",
            //     (Pose2d initialPose) -> {
            //         initialPose = Swerve.getInstance().getPose();
            //         var trajectory = PathPlanner.generatePath(
            //             new PathConstraints(2.5, 2.5),
            //             new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d()),
            //             new PathPoint(new Translation2d(2, 0), new Rotation2d(), Rotation2d.fromDegrees(90))
            //         );
            //         return new SequentialCommandGroup(
            //             new FollowSingleTrajectoryCommand(trajectory),
            //             mSwerve.driveCommand(new ChassisSpeeds(), true, true)
            //         );
            //     }
            // ),
            // mAutos.new Strategy(
            //     "2-piece test", 
            //     (Pose2d initialPose) -> autoBuilder.fullAuto(Autos.test2PieceTrajectory)
            // ),
            // mAutos.new Strategy(
            //     "2-piece test, trajectory only",
            //     (Pose2d initialPose) -> {
            //         var trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(k2pTestTraj, Alliance.Red);
            //         final var startPose = trajectory.getInitialHolonomicPose();
            //         return new SequentialCommandGroup(
            //             new InstantCommand(() -> Swerve.getInstance().resetPose(startPose), Swerve.getInstance()),
            //             new SwerveTrajectoryFollowerCommands.FollowSingleTrajectoryCommand(
            //                 trajectory
            //             )
            //         );
            //     }
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
                .onTrue(mSwerveCommands.new ResetPose());
                
                mDriverController.rightTrigger(kTriggerDeadband)
                .onTrue(mSwerveCommands.new EnableSprintMode())
                .onFalse(mSwerveCommands.new DisableSprintMode());
                
                mDriverController.rightBumper() // TODO: remove before comp
                .whileTrue(new ChargeStationCommands.AutoChargeStationClimb(false));

                mDriverController.leftBumper()
                .onTrue(mArmCommands.getTuckCommand());
 
                // Disabled for now with the belt extender.
                // mDriverController.rightBumper().whileTrue(mArm.getExtender().extendToTarget());
                
                // This is to tuck into stow
                mDriverController.povDown().onTrue(Commands.runOnce(()->mArm.stopPivot()));
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
                    .onTrue(mArmCommands.getGoToFloorCommand());

                mOperatorController.button(kMenuButtonId) //should be menu
                    .onTrue(mArmCommands.getGoToPresetArmStatePivotFirstCommand(ArmState.FRONT_DOUBLE_SUBSTATION, false));

                mOperatorController.rightStick()
                    .onTrue(mArmCommands.getGoToPresetArmStatePivotFirstCommand(ArmState.BACK_DOUBLE_SUBSTATION, false));
                
                mOperatorController.a()
                    .onTrue(mArmCommands.getGoToPresetArmStatePivotFirstCommand(ArmState.CUBE_LEVEL_1, false));
                
                mOperatorController.x()
                    .onTrue(mArmCommands.getGoToPresetArmStatePivotFirstCommand(ArmState.CONE_LEVEL_1, false));
                
                mOperatorController.b()
                    .onTrue(mArmCommands.getGoToPresetArmStatePivotFirstCommand(ArmState.SHOOT_HIGH_CUBE, false));
                
                mOperatorController.y()
                    .onTrue(mArmCommands.getRaiseToHighConeOverTopCommand());

                /**
                 * Eject game piece		    RT
                 * Intake game piece		LT
                 */
                mOperatorController.rightTrigger(kTriggerDeadband)
                    .onTrue(mIntakeCommands.new SetIntakeState(IntakeState.OUT))
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
                    
                final double extensionIncrementPerTic = 5. / 50; // 3 inches/sec at 50Hz
                mOperatorController.povRight()
                    .whileTrue(mArm.getExtender().modifyTargetCommandRepeat(extensionIncrementPerTic));

                mOperatorController.povLeft()
                .whileTrue(mArm.getExtender().modifyTargetCommandRepeat(-extensionIncrementPerTic));

                mOperatorController.leftBumper()
                    .whileTrue(mArmCommands.new TransformArmState(0, Rotation2d.fromDegrees(0), Arm.kTransformAmount));

                mOperatorController.rightBumper()
                    .whileTrue(mArmCommands.new TransformArmState(0, Rotation2d.fromDegrees(0), Arm.kTransformAmount.unaryMinus()));
                

                
                mOperatorController.leftStick()
                    .onTrue(mArmCommands.getTuckCommand());
            }

        }
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return mArm.getExtender().zeroExtenderCommand().andThen(mAutoSelector.getSelected().apply(mInitialPoseSelector.getSelected()));
        // return null;
    }
    
    public Command getTeleopInitCommand() {
        return mTeleopInitCommand;
    }
    
    public Command getTestingCommand() {
        return null;
    }
    
    public void initRobot() {
        mBlingSubsystem.startUnderglow();
        Command shuffleboard_update_command = new DebugTeleopCommands.ShuffleboardUpdateCommand(useArm, useSwerveChassis, mArm, mArmCommands, mSwerve, mSwerveCommands);
        shuffleboard_update_command.schedule();
        mSwerve.resetYaw();
        if (mSwerve != null) {
            mSwerve.resetToAbsolute();
            mSwerve.setFieldRelative(true);
        }
    }

    // Be careful, there is also reset code in the command itself
    public void resetForTeleop() {
        if (mSwerve != null) {
            mSwerve.resetYaw();
        }

        System.out.println("TeleopInit Called");
        if(mArm != null) {
            mArm.clearReference();
            System.out.println("Reference reset");

        }
    }

    public void resetForAuto() {
        if (mSwerve != null) {
            mSwerve.resetYaw();
        }
    }
}

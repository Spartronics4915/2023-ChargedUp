package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;
import com.spartronics4915.frc2023.subsystems.SwerveModule;
import com.spartronics4915.frc2023.commands.SwerveCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Map;


public final class DebugTeleopCommands {
    
    public static void teleopInit() {
		Swerve swerve_subsystem = Swerve.getInstance();
        swerve_subsystem.resetToAbsolute();
        swerve_subsystem.resetYaw();
        swerve_subsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))); // for odometry testing
        swerve_subsystem.stop();
        swerve_subsystem.alignModules();
        System.out.println("Teleopinit called");
        
    }
    
    public static class ChassisWidget {
        private GenericEntry yawEntry;
        private GenericEntry pitchEntry;
        private GenericEntry rollEntry;
        
        private GenericEntry vxEntry;
        private GenericEntry vyEntry;
        private GenericEntry omegaEntry;

        ChassisWidget(ShuffleboardTab tab) {
            ShuffleboardLayout chassisLayout = tab.getLayout("Chassis", BuiltInLayouts.kList)
            .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));
            
            yawEntry = chassisLayout.add("yaw (deg)", 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", -180, "Max", 180)).getEntry();
            pitchEntry = chassisLayout.add("pitch (deg)", 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", -180, "Max", 180)).getEntry();
            rollEntry = chassisLayout.add("roll (deg)", 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", -180, "Max", 180)).getEntry();

            vxEntry = chassisLayout.add("vx (m per s)", 0).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Min", -5, "Max", 5)).getEntry();
            vyEntry = chassisLayout.add("vy (m per s)", 0).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Min", -5, "Max", 5)).getEntry();
            omegaEntry = chassisLayout.add("omega (rad/s)", 0).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Min", -12, "Max", 12)).getEntry();
        }

        public void update() {
			Swerve swerve = Swerve.getInstance();

            yawEntry.setDouble(swerve.getIMU().getYaw());
            pitchEntry.setDouble(swerve.getIMU().getPitch());
            rollEntry.setDouble(swerve.getIMU().getRoll());
            
            vxEntry.setDouble(swerve.getChassisSpeeds().vxMetersPerSecond);
            vyEntry.setDouble(swerve.getChassisSpeeds().vyMetersPerSecond);
            omegaEntry.setDouble(swerve.getChassisSpeeds().omegaRadiansPerSecond);
        }
    }

    public static class ChargeWidget {

        private GenericEntry rollEntry;
        public ChargeWidget(ShuffleboardTab tab) {

        
            ShuffleboardLayout layout = tab.getLayout("ChargeStation", BuiltInLayouts.kList)
            .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));
            
            rollEntry = layout.add("Roll (Degrees)", 0).getEntry();
        }

        public void update(Swerve swerveSubsystem) {
            rollEntry.setDouble(swerveSubsystem.getYaw().getDegrees());
        }

    }

    public static class SwerveModuleWidget {
        private final GenericEntry mDesiredAngleEntry;
        private final GenericEntry mCurrentAngleEntry;
        private final GenericEntry mAbsoluteEncoderEntry;
        private final GenericEntry mRelativeEncoderEntry;
        private final GenericEntry mShiftedAbsoluteEncoderEntry;

        SwerveModuleWidget(ShuffleboardTab tab, String name) {
            ShuffleboardLayout swerve_module = tab.getLayout(name, BuiltInLayouts.kList)
            .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

            mDesiredAngleEntry = swerve_module.add("desired angle (deg)", 0).getEntry();
            mCurrentAngleEntry = swerve_module.add("current angle (deg)", 0).getEntry();
            mAbsoluteEncoderEntry = swerve_module.add("abs encoder (deg)", 0).getEntry();
            mRelativeEncoderEntry = swerve_module.add("rel encoder (deg)", 0).getEntry();
            mShiftedAbsoluteEncoderEntry = swerve_module.add("shifted abs encoder (deg)", 0).getEntry();
        }
        
        public void update(SwerveModule module) {
            SwerveModuleState current = module.getState();
            SwerveModuleState desired = module.getDesiredState();
            
            mDesiredAngleEntry.setDouble(desired.angle.getDegrees()); 
            mCurrentAngleEntry.setDouble(current.angle.getDegrees());
            mAbsoluteEncoderEntry.setDouble(module.getAbsoluteEncoderRotation().getDegrees()); 
            mRelativeEncoderEntry.setDouble(module.getRelativeEncoderRotation().getDegrees());
            mShiftedAbsoluteEncoderEntry.setDouble(module.getShiftedAbsoluteEncoderRotation().getDegrees());

        }
    }
    public static class SwerveTab {
        SwerveModuleWidget module0, module1, module2, module3;
        ChassisWidget chassisWidget;
        ShuffleboardTab tab;
        Swerve swerve_subsystem;
        SwerveCommands mSwerveCommands;

        SwerveTab(SwerveCommands swerveCommands) {
            mSwerveCommands = swerveCommands;
            tab = Shuffleboard.getTab("Swerve");
            module0 = new SwerveModuleWidget(tab, "Module 0");
            module1 = new SwerveModuleWidget(tab, "Module 1");
            module2 = new SwerveModuleWidget(tab, "Module 2");
            module3 = new SwerveModuleWidget(tab, "Module 3");
            chassisWidget = new ChassisWidget(tab);

            swerve_subsystem = Swerve.getInstance();
            ShuffleboardLayout commands = 
            tab.getLayout("Orientation", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
            
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(0)).withName("Orientation 0"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(90)).withName("Orientation 90"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(180)).withName("Orientation 180"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(270)).withName("Orientation 270"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(360)).withName("Orientation 360"));
            
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(-90)).withName("Orientation -90"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(-180)).withName("Orientation -180"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(-270)).withName("Orientation -270"));
            commands.add(Commands.runOnce(() -> swerve_subsystem.resetToAbsolute()).withName("Reset to Absolute"));
            commands.add(mSwerveCommands.new RotateToYaw(Rotation2d.fromDegrees(45)).withName("Rotate to 45"));

            commands.add(mSwerveCommands.new RotateYaw(Rotation2d.fromDegrees(45)).withName("Rotate 45"));
        }
        
        public void update(){
            
            var swerve_modules = swerve_subsystem.getSwerveModules();
            
            module0.update(swerve_modules[0]);
            module1.update(swerve_modules[1]);
            module2.update(swerve_modules[2]);
            module3.update(swerve_modules[3]);

            chassisWidget.update();
        }
    }
    
    public static class ShuffleboardUpdateCommand extends CommandBase {
        
        Swerve m_swerve_subsystem;
        SwerveTab m_swerve_tab;
        SwerveCommands mSwerveCommands;

        public ShuffleboardUpdateCommand(SwerveCommands swerveCommands) {
            m_swerve_subsystem = Swerve.getInstance();
            mSwerveCommands = swerveCommands;
        }
        // Called when the command is initially scheduled.
        
        @Override
        public void initialize() {
            
            m_swerve_tab = new SwerveTab(mSwerveCommands);
        }
        
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            m_swerve_tab.update();
        }

        // Lets this command run even when disabled
        @Override
        public boolean runsWhenDisabled() {

            return true;
        }


        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {}
        
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }
        
    }
}
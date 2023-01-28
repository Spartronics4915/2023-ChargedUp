package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;
import com.spartronics4915.frc2023.subsystems.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Map;


public final class DebugTeleopCommands {
    
    public static void teleopInit(Swerve swerve_subsystem) {
        swerve_subsystem.resetToAbsolute();
        swerve_subsystem.resetYaw();
        swerve_subsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))); // for odometry testing
        swerve_subsystem.stop();
        swerve_subsystem.alignModules();
        System.out.println("Teleopinit called");
        
    }
    
    public static class SwerveModuleWidget {
        private GenericEntry angleEntry;
        private GenericEntry state_angle, abs_encoder, rel_encoder, rel_encoder_deg, shifted_abs_encoder;
        SwerveModuleWidget(ShuffleboardTab tab, String name) {
            ShuffleboardLayout swerve_module = tab.getLayout(name, BuiltInLayouts.kList)
            .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

            angleEntry = swerve_module.add("desired.angle", 0).getEntry();
            state_angle = swerve_module.add("current.angle", 0).getEntry();
            abs_encoder = swerve_module.add("abs_encoder", 0).getEntry();
            rel_encoder = swerve_module.add("rel_encoder", 0).getEntry();
            rel_encoder_deg = swerve_module.add("rel_encoder (degrees)", 0).getEntry();
            shifted_abs_encoder = swerve_module.add("shifted abs_encoder", 0).getEntry();
        }
        
        public void update(SwerveModule module) {
            SwerveModuleState current = module.getState();
            SwerveModuleState desired = module.getDesiredState();
            
            angleEntry.setDouble(desired.angle.getDegrees()); 
            state_angle.setDouble(current.angle.getDegrees());
            abs_encoder.setDouble(module.getAbsoluteEncoderValue()); 
            rel_encoder.setDouble(module.getRelativeEncoderValue());
            rel_encoder_deg.setDouble(Rotation2d.fromRadians(module.getRelativeEncoderValue()).getDegrees());
            shifted_abs_encoder.setDouble(module.getShiftedAbsoluteEncoderRotations());

        }
    }
    public static class SwerveTab {
        SwerveModuleWidget module0, module1, module2, module3;
        ShuffleboardTab tab;
        Swerve swerve_subsystem;
        
        SwerveTab(Swerve swerve) {
            tab = Shuffleboard.getTab("Swerve");
            module0 = new SwerveModuleWidget(tab, "Module 0");
            module1 = new SwerveModuleWidget(tab, "Module 1");
            module2 = new SwerveModuleWidget(tab, "Module 2");
            module3 = new SwerveModuleWidget(tab, "Module 3");
            swerve_subsystem = swerve;
            ShuffleboardLayout elevatorCommands = 
            tab.getLayout("Orientation", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
            
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(0)).withName("Orientation 0"));
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(90)).withName("Orientation 90"));
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(180)).withName("Orientation 180"));
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(270)).withName("Orientation 270"));
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(360)).withName("Orientation 360"));
            
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(-90)).withName("Orientation -90"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(-180)).withName("Orientation -180"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(-270)).withName("Orientation -270"));
            elevatorCommands.add(Commands.runOnce(() -> swerve.zeroPIDP()).withName("Zero PID P"));
            elevatorCommands.add(Commands.runOnce(() -> swerve_subsystem.resetToAbsolute()).withName("Reset to Absolute"));
        }
        
        public void update(){
            
            var swerve_modules = swerve_subsystem.getSwerveModules();
            
            module0.update(swerve_modules[0]);
            module1.update(swerve_modules[1]);
            module2.update(swerve_modules[2]);
            module3.update(swerve_modules[3]);
        }
    }
    
    public static class ShuffleboardUpdateCommand extends CommandBase {
        
        Swerve m_swerve_subsystem;
        SwerveTab m_swerve_tab;

        public ShuffleboardUpdateCommand(Swerve swerve_subsystem) {
            m_swerve_subsystem = swerve_subsystem;
        }
        // Called when the command is initially scheduled.
        
        @Override
        public void initialize() {
            
            m_swerve_tab = new SwerveTab(m_swerve_subsystem);
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
package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class SimpleAutos {
  /** Example static factory for an autonomous command. */
  public static CommandBase forceOrientation(Swerve swerve_subsystem, Rotation2d orientation) {

    return Commands.runOnce(
      () -> swerve_subsystem.forceModuleOrientations(orientation, false), swerve_subsystem);
  }

  public static CommandBase driveStraightIndefinite(Swerve swerve_subsystem, double speed_m_per_s)
  {

    Translation2d straightVelocity = new Translation2d(speed_m_per_s, 0);
        return Commands.runOnce(() -> swerve_subsystem.drive(straightVelocity, 0.0, true), swerve_subsystem);

  }

  public static CommandBase stop(Swerve swerve_subsystem) {
    return Commands.runOnce(() -> swerve_subsystem.drive(new Translation2d(0.0, 0.0), 
      0.0, true), swerve_subsystem);

  }

  public static CommandBase driveStraightTimed(Swerve swerve_subsystem, double duration_seconds, double speed_m_per_s)
  {
    return Commands.sequence(
      driveStraightIndefinite(swerve_subsystem, speed_m_per_s),
      Commands.waitSeconds(duration_seconds),
      stop(swerve_subsystem));

  }
  private SimpleAutos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

    public static CommandBase driveStraight(Swerve swerve, double kSpeedMPerS) {
        Translation2d straightVelocity = new Translation2d(kSpeedMPerS, 0);
        return Commands.runOnce(() -> swerve.drive(straightVelocity, 0.0, true), swerve);
    }

    public static CommandBase stop(Swerve swerve) {
        return Commands.runOnce(() -> swerve.drive(new Translation2d(0.0, 0.0),
                0.0, true), swerve);
    }

    public static CommandBase leaveCommunity(Swerve swerve, double duration_seconds, double kSpeedMPerS) {
        return Commands.sequence(
                driveStraight(swerve, kSpeedMPerS),
                Commands.waitSeconds(duration_seconds),
                stop(swerve));

    }

    private Autos() {};
}

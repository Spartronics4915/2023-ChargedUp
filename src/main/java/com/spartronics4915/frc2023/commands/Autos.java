// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.PhotonCameraWrapper;
import com.spartronics4915.frc2023.commands.PrintPos;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static CommandBase exampleAuto(PhotonCameraWrapper camera) {
        return Commands.sequence(new PrintPos(camera));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

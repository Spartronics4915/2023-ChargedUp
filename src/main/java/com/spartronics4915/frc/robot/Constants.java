// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

 
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    static class FieldConstants {
        static final double length = Units.feetToMeters(54);
        static final double width = Units.feetToMeters(27);
    }

    static class VisionConstants {
        static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(0.5, 0.0, 0.5),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        static final String cameraName = "YOUR CAMERA NAME";
    }
}

// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Utility functions for mirroring poses and other field elements based on the current alliance color. */
public class FieldMirroringUtils {

    private static final double FIELD_LENGTH_METERS = 16.5417;
    private static final double FIELD_WIDTH_METERS = 8.0136;

    /**
     * Returns the alliance-specific pose of the robot.
     *
     * @param pose The pose for the blue alliance.
     * @return The pose for the current alliance.
     */
    public static Pose2d toCurrentAlliancePose(Pose2d pose) {
        return isRedAlliance()
                ? new Pose2d(
                        FIELD_LENGTH_METERS - pose.getX(),
                        pose.getY(),
                        new Rotation2d(Math.PI).minus(pose.getRotation()))
                : pose;
    }

    /**
     * Returns the alliance-specific translation of the robot.
     *
     * @param translation The translation for the blue alliance.
     * @return The translation for the current alliance.
     */
    public static Translation2d toCurrentAllianceTranslation(Translation2d translation) {
        return isRedAlliance()
                ? new Translation2d(FIELD_LENGTH_METERS - translation.getX(), translation.getY())
                : translation;
    }

    /**
     * Returns the rotation of the current alliance's driver station wall.
     *
     * @return 0 degrees for Blue, 180 degrees for Red.
     */
    public static Rotation2d getCurrentAllianceDriverStationFacing() {
        return isRedAlliance() ? Rotation2d.fromDegrees(180.0) : new Rotation2d();
    }

    /** @return Whether the current alliance is the red alliance. */
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }
}

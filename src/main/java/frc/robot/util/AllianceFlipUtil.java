// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and poses in
 * {@link FieldConstants} are stored with the origin at the rightmost point on the blue alliance wall.
 */
public class AllianceFlipUtil {
    public static final double fieldLength = 1755.0 / 100.0;
    public static final double fieldWidth = 805.0 / 100.0;

    public static enum FieldFlipType {
        CenterPointFlip,
        MirrorFlip,
    }

    public static final FieldFlipType defaultFlipType = FieldFlipType.CenterPointFlip;

    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d apply(Translation2d translation) {
        return apply(translation, defaultFlipType);
    }

    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d apply(Translation2d translation, FieldFlipType flipType) {
        if (!shouldFlip()) return translation;
        switch (flipType) {
            default:
            case CenterPointFlip:
                return new Translation2d(fieldLength - translation.getX(), fieldWidth - translation.getY());
            case MirrorFlip:
                return new Translation2d(fieldLength - translation.getX(), translation.getY());
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        return apply(rotation, defaultFlipType);
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation, FieldFlipType flipType) {
        if (!shouldFlip()) return rotation;
        switch (flipType) {
            default:
            case CenterPointFlip:
                return rotation.rotateBy(Rotation2d.fromRotations(0.5));
            case MirrorFlip:
                return new Rotation2d(-rotation.getCos(), rotation.getSin());
        }
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose) {
        return apply(pose, defaultFlipType);
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose, FieldFlipType flipType) {
        if (!shouldFlip()) return pose;
        return new Pose2d(apply(pose.getTranslation(), flipType), apply(pose.getRotation(), flipType));
    }

    /**
     * Flips a pose to the specified field side.
     *
     * @param pose The pose to flip.
     * @param isRed If true, the pose is flipped for the Red alliance; otherwise, for the Blue alliance.
     */
    public static Pose2d apply(Pose2d pose, boolean isRed) {
        return apply(pose, isRed, defaultFlipType);
    }

    /**
     * Flips a pose to the specified field side using a specified flip type.
     *
     * @param pose The pose to flip.
     * @param isRed If true, the pose is flipped for the Red alliance; otherwise, for the Blue alliance.
     * @param flipType The type of flip to perform.
     */
    public static Pose2d apply(Pose2d pose, boolean isRed, FieldFlipType flipType) {
        if (!isRed) return pose; // The pose is stored as if it's on the Blue side (isRed = false)

        // If isRed is true, we flip the pose, regardless of the current DS alliance.
        // This logic is based on the assumption that 'apply' functions flip an
        // *always-blue-side* value to the *desired* side.

        // We'll use a local helper to perform the flip logic from blue to red,
        // mirroring the logic of the original 'apply' functions that use shouldFlip().
        return flipPoseToRed(pose, flipType);
    }

    /** Helper to perform the Pose2d flip logic from Blue to Red. */
    private static Pose2d flipPoseToRed(Pose2d pose, FieldFlipType flipType) {
        Translation2d newTranslation;
        Rotation2d newRotation;

        switch (flipType) {
            default:
            case CenterPointFlip:
                // Translation flip
                newTranslation = new Translation2d(fieldLength - pose.getX(), fieldWidth - pose.getY());
                // Rotation flip (rotate by 180 degrees)
                newRotation = pose.getRotation().rotateBy(Rotation2d.fromRotations(0.5));
                break;
            case MirrorFlip:
                // Translation flip
                newTranslation = new Translation2d(fieldLength - pose.getX(), pose.getY());
                // Rotation flip (mirror across the X-axis of the field center line)
                newRotation = new Rotation2d(
                        -pose.getRotation().getCos(), pose.getRotation().getSin());
                break;
        }

        return new Pose2d(newTranslation, newRotation);
    }

    // --- End of New Methods ---

    public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds) {
        return applyFieldRelative(speeds, defaultFlipType);
    }

    public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
        if (!shouldFlip()) return speeds;
        switch (flipType) {
            default:
            case CenterPointFlip:
                return new ChassisSpeeds(
                        -speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
            case MirrorFlip:
                return new ChassisSpeeds(
                        -speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
    }

    public static ChassisSpeeds applyRobotRelative(ChassisSpeeds speeds, Rotation2d robotRotation) {
        return applyRobotRelative(speeds, robotRotation, defaultFlipType);
    }

    public static ChassisSpeeds applyRobotRelative(
            ChassisSpeeds speeds, Rotation2d robotRotation, FieldFlipType flipType) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                applyFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotRotation)), robotRotation);
    }

    /** Flips a trajectory state to the correct side of the field based on the current alliance color. */
    // public static Trajectory.State apply(Trajectory.State state) {
    // if (shouldFlip()) {
    // return new Trajectory.State(
    // state.timeSeconds,
    // state.velocityMetersPerSecond,
    // state.accelerationMetersPerSecondSq,
    // new Pose2d(
    // FieldConstants.fieldLength - state.poseMeters.getX(),
    // state.poseMeters.getY(),
    // new Rotation2d(
    // -state.poseMeters.getRotation().getCos(),
    // state.poseMeters.getRotation().getSin())),
    // -state.curvatureRadPerMeter);
    // } else {
    // return state;
    // }
    // }

    /** Flips a rotation sequence state based on the current alliance color. */
    // public static RotationSequence.State apply(RotationSequence.State state) {
    // if (shouldFlip()) {
    // return new RotationSequence.State(
    // new Rotation2d(-state.position.getCos(), state.position.getSin()),
    // -state.velocityRadiansPerSec);
    // } else {
    // return state;
    // }
    // }

    /** Returns true if the robot is on the red alliance. */
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }
}

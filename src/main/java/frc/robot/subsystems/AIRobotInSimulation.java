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

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.mapleSimConfig;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.util.FieldMirroringUtils;
import frc.robot.util.RobotModeTriggers;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;

public class AIRobotInSimulation extends SubsystemBase {

    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
    };

    public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
    };

    private final SwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;
    private final IntakeIOSim intake;
    private final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

    private final boolean isOpponent;

    public AIRobotInSimulation(int id, boolean isOpponent) {
        this.id = id;
        this.isOpponent = isOpponent;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        this.driveSimulation =
                new SwerveDriveSimulation(mapleSimConfig, queeningPose);
        this.intake = new IntakeIOSim(driveSimulation);

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    }

    public void buildBehaviorChooser(Command autoCycle) {
        SendableChooser<Command> behaviorChooser = new SendableChooser<>();
        final Supplier<Command> disable =
                () -> Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(queeningPose), this)
                        .andThen(Commands.runOnce(() -> driveSimulation.setRobotSpeeds(
                                new ChassisSpeeds())))
                        .ignoringDisable(true);

        behaviorChooser.setDefaultOption("Disable", disable.get());
        behaviorChooser.addOption("Auto Cycle", autoCycle);
        behaviorChooser.onChange((Command::schedule));
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));
        RobotModeTriggers.disabled().onTrue(disable.get());

        SmartDashboard.putData(
                "AIRobotBehaviors/" + (isOpponent ? "Opponent" : "Teammate") + " Robot " + id + " Behavior",
                behaviorChooser);
    }

    public Command intake() {
        return Commands.none();
    }

    public static AIRobotInSimulation[] instances = new AIRobotInSimulation[5];

    public static void startOpponentRobotSimulations() {
        try {
            // Teammates
            instances[0] = new AIRobotInSimulation(3, false);
            instances[0].buildBehaviorChooser(Commands.none());
            instances[1] = new AIRobotInSimulation(4, false);
            instances[1].buildBehaviorChooser(Commands.none());

            // Opponents
            instances[2] = new AIRobotInSimulation(0, true);
            instances[2].buildBehaviorChooser(Commands.none());
            instances[3] = new AIRobotInSimulation(1, true);
            instances[3].buildBehaviorChooser(Commands.none());
            instances[4] = new AIRobotInSimulation(2, true);
            instances[4].buildBehaviorChooser(Commands.none());

        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
        }
    }
}

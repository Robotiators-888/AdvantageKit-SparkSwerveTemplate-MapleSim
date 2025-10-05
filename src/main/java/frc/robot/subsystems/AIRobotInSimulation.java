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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CMD_PathfindCloseReefAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.util.RobotModeTriggers;
import static frc.robot.subsystems.drive.DriveConstants.*;
import java.util.Arrays;
import java.util.Optional;
import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class AIRobotInSimulation extends SubsystemBase {
    private static final Random random = new Random();
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
    private final DriverStation.Alliance alliance;
    public AIRobotInSimulation(int id, boolean isOpponent, DriverStation.Alliance alliance) {
        this.id = id;
        this.isOpponent = isOpponent;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        this.driveSimulation = new SwerveDriveSimulation(mapleSimConfig, queeningPose);
        this.intake = new IntakeIOSim(driveSimulation);
        this.alliance = alliance;
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    }

    public void buildBehaviorChooser(Command autoCycle) {
        SendableChooser<Command> behaviorChooser = new SendableChooser<>();
        final Supplier<Command> disable =
                () -> Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(queeningPose), this)
                        .andThen(Commands.runOnce(() -> driveSimulation.setRobotSpeeds(new ChassisSpeeds())))
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
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            Optional<DriverStation.Alliance> OpAlliance;
            if (alliance.get() == DriverStation.Alliance.Red) {
                OpAlliance = Optional.of(DriverStation.Alliance.Blue);
            } else {
                OpAlliance = Optional.of(DriverStation.Alliance.Red);
            }
            PathConstraints constraints =
                    new PathConstraints(3.0, 2.1, Units.degreesToRadians(540), Units.degreesToRadians(720));
            PathPlannerPath leftFeederStation = PathPlannerPath.fromPathFile("Left Feeder Station");
            PathPlannerPath rightFeederStation = PathPlannerPath.fromPathFile("Right Feeder Station");

            // Teammates
            instances[0] = new AIRobotInSimulation(3, false,alliance.get());
            Command botCommand = new SequentialCommandGroup(
                    AutoBuilder.pathfindThenFollowPath(
                            random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
                    Commands.runOnce(() -> instances[0].intake.intakeCoralStation()),
                    CMD_PathfindCloseReefAlign.pathfindToRandomPose(alliance),
                    CMD_PathfindCloseReefAlign.pathfindingCommand(
                            instances[0].driveSimulation.getSimulatedDriveTrainPose(),
                            random.nextDouble() < 0.5,
                            alliance),
                    Commands.runOnce(() -> {
                        if (random.nextDouble() < 0.5) {
                            instances[0].intake.launchCoralLevel3();
                        } else {
                            instances[0].intake.launchCoralLevel4();
                        }
                    }));
            instances[0].buildBehaviorChooser(botCommand);

            instances[1] = new AIRobotInSimulation(4, false,alliance.get());
            botCommand = new SequentialCommandGroup(
                    AutoBuilder.pathfindThenFollowPath(
                            random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
                    Commands.runOnce(() -> instances[1].intake.intakeCoralStation()),
                    CMD_PathfindCloseReefAlign.pathfindToRandomPose(alliance),
                    CMD_PathfindCloseReefAlign.pathfindingCommand(
                            instances[1].driveSimulation.getSimulatedDriveTrainPose(),
                            random.nextDouble() < 0.5,
                            alliance),
                    Commands.runOnce(() -> {
                        if (random.nextDouble() < 0.5) {
                            instances[1].intake.launchCoralLevel3();
                        } else {
                            instances[1].intake.launchCoralLevel4();
                        }
                    }));
            instances[1].buildBehaviorChooser(botCommand);

            // Opponents
            instances[2] = new AIRobotInSimulation(0, true,OpAlliance.get());
            botCommand = new SequentialCommandGroup(
                    AutoBuilder.pathfindThenFollowPath(
                            random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
                    Commands.runOnce(() -> instances[2].intake.intakeCoralStation()),
                    CMD_PathfindCloseReefAlign.pathfindToRandomPose(OpAlliance),
                    CMD_PathfindCloseReefAlign.pathfindingCommand(
                            instances[2].driveSimulation.getSimulatedDriveTrainPose(),
                            random.nextDouble() < 0.5,
                            OpAlliance),
                    Commands.runOnce(() -> {
                        if (random.nextDouble() < 0.5) {
                            instances[2].intake.launchCoralLevel3();
                        } else {
                            instances[2].intake.launchCoralLevel4();
                        }
                    }));
            instances[2].buildBehaviorChooser(botCommand);

            instances[3] = new AIRobotInSimulation(1, true,OpAlliance.get());
            botCommand = new SequentialCommandGroup(
                    AutoBuilder.pathfindThenFollowPath(
                            random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
                    Commands.runOnce(() -> instances[3].intake.intakeCoralStation()),
                    CMD_PathfindCloseReefAlign.pathfindToRandomPose(OpAlliance),
                    CMD_PathfindCloseReefAlign.pathfindingCommand(
                            instances[3].driveSimulation.getSimulatedDriveTrainPose(),
                            random.nextDouble() < 0.5,
                            OpAlliance),
                    Commands.runOnce(() -> {
                        if (random.nextDouble() < 0.5) {
                            instances[3].intake.launchCoralLevel3();
                        } else {
                            instances[3].intake.launchCoralLevel4();
                        }
                    }));
            instances[3].buildBehaviorChooser(botCommand);

            instances[4] = new AIRobotInSimulation(2, true,OpAlliance.get());
            botCommand = new SequentialCommandGroup(
                    AutoBuilder.pathfindThenFollowPath(
                            random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
                    Commands.runOnce(() -> instances[4].intake.intakeCoralStation()),
                    CMD_PathfindCloseReefAlign.pathfindToRandomPose(OpAlliance),
                    CMD_PathfindCloseReefAlign.pathfindingCommand(
                            instances[4].driveSimulation.getSimulatedDriveTrainPose(),
                            random.nextDouble() < 0.5,
                            OpAlliance),
                    Commands.runOnce(() -> {
                        if (random.nextDouble() < 0.5) {
                            instances[4].intake.launchCoralLevel3();
                        } else {
                            instances[4].intake.launchCoralLevel4();
                        }
                    }));
            instances[4].buildBehaviorChooser(botCommand);

        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
            throw new RuntimeException(e);
        }
    }
    public static Pose2d[] getOpponentRobotPoses() {
        return new Pose2d[] {
            instances[2].driveSimulation.getSimulatedDriveTrainPose(),
            instances[3].driveSimulation.getSimulatedDriveTrainPose(),
            instances[4].driveSimulation.getSimulatedDriveTrainPose()
        };
    }
    
    public static Pose2d[] getAlliancePartnerRobotPoses() {
        return new Pose2d[] {
            instances[0].driveSimulation.getSimulatedDriveTrainPose(),
            instances[1].driveSimulation.getSimulatedDriveTrainPose()
        };
    }

    public Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints,int instanceIndex) {
        return PathfindThenFollowPath(
            path, 
            constraints,
            instances[instanceIndex].driveSimulation::getSimulatedDriveTrainPose, 
            instances[instanceIndex].driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative, 
            instances[instanceIndex].driveSimulation::setRobotSpeeds, 
            driveController,
            new RobotConfig(
                robotMassKg,
                robotMOI,
                new ModuleConfig(
                        wheelRadiusMeters,
                        maxSpeedMetersPerSec,
                        wheelCOF,
                        driveGearbox.withReduction(driveMotorReduction),
                        driveMotorCurrentLimit,
                        1),
                moduleTranslations),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );
        // return new PathfindThenFollowPath()
    }
}

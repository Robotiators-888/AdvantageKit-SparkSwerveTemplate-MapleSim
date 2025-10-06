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

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CMD_PathfindCloseReefAlign;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.RobotModeTriggers;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class AIRobotInSimulation extends SubsystemBase {
    private static final double FIELD_LENGTH_M = 17.55; // X-dimension (57 ft. 6⅞ in.)
    private static final double FIELD_WIDTH_M = 8.05; // Y-dimension (26 ft. 5 in.)
    private static final double MIN_X_BLUE = 0.5; // Start of Blue Alliance half
    private static final double MAX_X_BLUE =
            FIELD_LENGTH_M / 2.0 - 0.2; // End of Blue Alliance half, slightly offset (~8.575m)
    private static final double MIN_Y = 0.5;
    private static final double MAX_Y = FIELD_WIDTH_M - 0.5;
    private static final Random random = new Random();
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-1, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d())
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
    private final Pose2d startingPose;
    private final int id;
    private final IntakeIOSim intake;
    private final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(1.5,0.02, 0.2), new PIDConstants(0, 0));

    private final boolean isOpponent;
    private final DriverStation.Alliance alliance;
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public AIRobotInSimulation(int id, boolean isOpponent, DriverStation.Alliance alliance) {
        this.id = id;
        this.isOpponent = isOpponent;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        this.startingPose = ROBOTS_STARTING_POSITIONS[id];
        this.driveSimulation = new SwerveDriveSimulation(mapleSimConfig, queeningPose);
        this.intake = new IntakeIOSim(driveSimulation);
        this.alliance = alliance;
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    }

    public void buildBehaviorChooser(Command autoCycle) {
        SendableChooser<Command> behaviorChooser = new SendableChooser<>();
        final Supplier<Command> disable =
                () -> Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(this.queeningPose), this)
                        .andThen(Commands.runOnce(() -> driveSimulation.setRobotSpeeds(new ChassisSpeeds())))
                        .ignoringDisable(true);

        behaviorChooser.setDefaultOption("Disable", disable.get());
        behaviorChooser.addOption(
                "Auto Cycle",
                Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(this.startingPose), this)
                        .andThen(autoCycle.repeatedly()));
        behaviorChooser.onChange((Command::schedule));
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));
        RobotModeTriggers.disabled().onTrue(disable.get());

        SmartDashboard.putData(
                "AIRobotBehaviors/" + (isOpponent ? "Opponent" : "Teammate") + " Robot " + id + " Behavior",
                behaviorChooser);
    }

    public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        ChassisSpeeds flipped = new ChassisSpeeds(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,0*speeds.omegaRadiansPerSecond);
        this.driveSimulation.setRobotSpeeds(flipped);
    }

    public static AIRobotInSimulation[] instances = new AIRobotInSimulation[5];

    public static void startOpponentRobotSimulations() {
        try {
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            Optional<DriverStation.Alliance> OpAlliance;
            if (alliance.isEmpty()) {
                alliance = Optional.of(DriverStation.Alliance.Red);
            }
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
            instances[0] = new AIRobotInSimulation(3, false, alliance.get());
            Command botCommand = new SequentialCommandGroup(
                    instances[0].PFThenFollowPath(
                            random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
                    Commands.runOnce(() -> instances[0].intake.intakeCoralStation()),
                    pathfindToRandomPose(alliance, instances[0]),
                    pathfindingCommand(
                            instances[0].driveSimulation.getSimulatedDriveTrainPose(),
                            random.nextDouble() < 0.5,
                            alliance,
                            instances[0]),
                    Commands.runOnce(() -> {
                        if (random.nextDouble() < 0.5) {
                            instances[0].intake.launchCoralLevel3();
                        } else {
                            instances[0].intake.launchCoralLevel4();
                        }
                    }));
            instances[0].buildBehaviorChooser(botCommand);

            // instances[1] = new AIRobotInSimulation(4, false, alliance.get());
            // botCommand = new SequentialCommandGroup(
            //         instances[1].PFThenFollowPath(
            //                 random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
            //         Commands.runOnce(() -> instances[1].intake.intakeCoralStation()),
            //         CMD_PathfindCloseReefAlign.pathfindToRandomPose(alliance, instances[1]),
            //         CMD_PathfindCloseReefAlign.pathfindingCommand(
            //                 instances[1].driveSimulation.getSimulatedDriveTrainPose(),
            //                 random.nextDouble() < 0.5,
            //                 alliance,
            //                 instances[1]),
            //         Commands.runOnce(() -> {
            //             if (random.nextDouble() < 0.5) {
            //                 instances[1].intake.launchCoralLevel3();
            //             } else {
            //                 instances[1].intake.launchCoralLevel4();
            //             }
            //         }));
            // instances[1].buildBehaviorChooser(botCommand);

            // // Opponents
            // instances[2] = new AIRobotInSimulation(0, true, OpAlliance.get());
            // botCommand = new SequentialCommandGroup(
            //         instances[2].PFThenFollowPath(
            //                 random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
            //         Commands.runOnce(() -> instances[2].intake.intakeCoralStation()),
            //         CMD_PathfindCloseReefAlign.pathfindToRandomPose(OpAlliance, instances[2]),
            //         CMD_PathfindCloseReefAlign.pathfindingCommand(
            //                 instances[2].driveSimulation.getSimulatedDriveTrainPose(),
            //                 random.nextDouble() < 0.5,
            //                 OpAlliance,
            //                 instances[2]),
            //         Commands.runOnce(() -> {
            //             if (random.nextDouble() < 0.5) {
            //                 instances[2].intake.launchCoralLevel3();
            //             } else {
            //                 instances[2].intake.launchCoralLevel4();
            //             }
            //         }));
            // instances[2].buildBehaviorChooser(botCommand);

            // instances[3] = new AIRobotInSimulation(1, true, OpAlliance.get());
            // botCommand = new SequentialCommandGroup(
            //         instances[3].PFThenFollowPath(
            //                 random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
            //         Commands.runOnce(() -> instances[3].intake.intakeCoralStation()),
            //         CMD_PathfindCloseReefAlign.pathfindToRandomPose(OpAlliance, instances[3]),
            //         CMD_PathfindCloseReefAlign.pathfindingCommand(
            //                 instances[3].driveSimulation.getSimulatedDriveTrainPose(),
            //                 random.nextDouble() < 0.5,
            //                 OpAlliance,
            //                 instances[3]),
            //         Commands.runOnce(() -> {
            //             if (random.nextDouble() < 0.5) {
            //                 instances[3].intake.launchCoralLevel3();
            //             } else {
            //                 instances[3].intake.launchCoralLevel4();
            //             }
            //         }));
            // instances[3].buildBehaviorChooser(botCommand);

            // instances[4] = new AIRobotInSimulation(2, true, OpAlliance.get());
            // botCommand = new SequentialCommandGroup(
            //         instances[4].PFThenFollowPath(
            //                 random.nextDouble() < 0.5 ? rightFeederStation : leftFeederStation, constraints),
            //         Commands.runOnce(() -> instances[4].intake.intakeCoralStation()),
            //         CMD_PathfindCloseReefAlign.pathfindToRandomPose(OpAlliance, instances[4]),
            //         CMD_PathfindCloseReefAlign.pathfindingCommand(
            //                 instances[4].driveSimulation.getSimulatedDriveTrainPose(),
            //                 random.nextDouble() < 0.5,
            //                 OpAlliance,
            //                 instances[4]),
            //         Commands.runOnce(() -> {
            //             if (random.nextDouble() < 0.5) {
            //                 instances[4].intake.launchCoralLevel3();
            //             } else {
            //                 instances[4].intake.launchCoralLevel4();
            //             }
            //         }));
            // instances[4].buildBehaviorChooser(botCommand);

        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
            throw new RuntimeException(e);
        }
    }

    public boolean isRedAlliance() {
        return this.isOpponent;
    }

    public static Pose2d[] getOpponentRobotPoses() {
        return new Pose2d[] {
            // instances[2].driveSimulation.getSimulatedDriveTrainPose(),
            // instances[3].driveSimulation.getSimulatedDriveTrainPose(),
            // instances[4].driveSimulation.getSimulatedDriveTrainPose()
        };
    }

    public static Pose2d[] getAlliancePartnerRobotPoses() {
        return new Pose2d[] {
            instances[0].driveSimulation.getSimulatedDriveTrainPose(),
            // instances[1].driveSimulation.getSimulatedDriveTrainPose()
        };
    }

    public Command PFThenFollowPath(PathPlannerPath path, PathConstraints constraints) {
        return new PathfindThenFollowPath(
                path,
                constraints,
                this.driveSimulation::getSimulatedDriveTrainPose,
                this.driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(0, 0), new PIDConstants(0, 0)), //5,7
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
                this::isRedAlliance,
                this);
        // return new PathfindThenFollowPath()
    }

    public Command PFToPose(Pose2d pose, PathConstraints constraints) {
        return new PathfindingCommand(
                pose,
                constraints,
                0,
                this.driveSimulation::getSimulatedDriveTrainPose,
                this.driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(0,0, 0), new PIDConstants(0, 0)),
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
                this);
    }

    public static Pose2d generateRandomTargetPose(Optional<DriverStation.Alliance> alliance) {
        double rangeX = MAX_X_BLUE - MIN_X_BLUE;
        double randomX = MIN_X_BLUE + (rangeX * random.nextDouble());

        // Y range is the full width of the field
        double rangeY = MAX_Y - MIN_Y;
        double randomY = MIN_Y + (rangeY * random.nextDouble());

        // Random Holonomic Rotation (from -180 to 180 degrees)
        double randomDegrees = random.nextDouble() * 360.0 - 180.0;
        Rotation2d randomRotation = Rotation2d.fromDegrees(randomDegrees);

        Pose2d targetPose = new Pose2d(randomX, randomY, randomRotation);

        // Mirror the pose if on the Red Alliance
        Logger.recordOutput("RandomPoseLoc",AllianceFlipUtil.apply(targetPose, alliance.get() == DriverStation.Alliance.Blue));
        return AllianceFlipUtil.apply(targetPose, alliance.get() == DriverStation.Alliance.Blue);
    }

    public static Command pathfindToRandomPose(
            Optional<DriverStation.Alliance> alliance, AIRobotInSimulation instance) {
        Pose2d pose = generateRandomTargetPose(alliance);
        PathConstraints constraints =
                new PathConstraints(3.0, 2.1, Units.degreesToRadians(540), Units.degreesToRadians(720));
        return instance.PFToPose(pose, constraints);
    }

    public static Command pathfindingCommand(
            Pose2d drivePose,
            boolean isLeftAlign,
            Optional<DriverStation.Alliance> alliance,
            AIRobotInSimulation instance) {
        List<Integer> targetTagSets;
        Command pathfindingCommand;
        Pose2d tagPose = new Pose2d();
        Integer targetId = 7;
        HashMap<Integer, Translation2d> redLeft = new HashMap<>();
        HashMap<Integer, Translation2d> redRight = new HashMap<>();
        HashMap<Integer, Translation2d> blueLeft = new HashMap<>();
        HashMap<Integer, Translation2d> blueRight = new HashMap<>();
        redRight.put(7, new Translation2d(14.341348, 4.2116375));
        redLeft.put(7, new Translation2d(14.341348, 3.8401625));
        redRight.put(8, new Translation2d(13.539017606564588, 5.228798303296214));
        redLeft.put(8, new Translation2d(13.860724393435412, 5.043060803296214));
        redRight.put(9, new Translation2d(12.257079606564588, 5.043060803296214));
        redLeft.put(9, new Translation2d(12.578786393435411, 5.228798303296214));
        redRight.put(10, new Translation2d(11.776455999999998, 3.8401625));
        redLeft.put(10, new Translation2d(11.776455999999998, 4.2116375));
        redRight.put(11, new Translation2d(12.578786393435411, 2.8230016967037854));
        redLeft.put(11, new Translation2d(12.257079606564588, 3.0087391967037855));
        redRight.put(6, new Translation2d(13.860724393435412, 3.0087391967037855));
        redLeft.put(6, new Translation2d(13.539017606564588, 2.8230016967037854));
        blueRight.put(21, new Translation2d(5.771896, 4.2116375));
        blueLeft.put(21, new Translation2d(5.771896, 3.8401625));
        blueRight.put(20, new Translation2d(4.969311606564587, 5.228798303296214));
        blueLeft.put(20, new Translation2d(5.2910183934354125, 5.043060803296214));
        blueRight.put(19, new Translation2d(3.687627606564587, 5.043060803296214));
        blueLeft.put(19, new Translation2d(4.009334393435411, 5.228798303296214));
        blueRight.put(18, new Translation2d(3.20675, 3.8401625));
        blueLeft.put(18, new Translation2d(3.20675, 4.2116375));
        blueRight.put(17, new Translation2d(4.00933439343541, 2.8230016967037854));
        blueLeft.put(17, new Translation2d(3.6876276065645865, 3.0087391967037855));
        blueRight.put(22, new Translation2d(5.2910183934354125, 3.0087391967037855));
        blueLeft.put(22, new Translation2d(4.969311606564588, 2.8230016967037854));
        redRight.put(7, new Translation2d(14.341348, 4.2116375));
        redLeft.put(7, new Translation2d(14.341348, 3.8401625));
        redRight.put(8, new Translation2d(13.539017606564588, 5.228798303296214));
        redLeft.put(8, new Translation2d(13.860724393435412, 5.043060803296214));
        redRight.put(9, new Translation2d(12.257079606564588, 5.043060803296214));
        redLeft.put(9, new Translation2d(12.578786393435411, 5.228798303296214));
        redRight.put(10, new Translation2d(11.776455999999998, 3.8401625));
        redLeft.put(10, new Translation2d(11.776455999999998, 4.2116375));
        redRight.put(11, new Translation2d(12.578786393435411, 2.8230016967037854));
        redLeft.put(11, new Translation2d(12.257079606564588, 3.0087391967037855));
        redRight.put(6, new Translation2d(13.860724393435412, 3.0087391967037855));
        redLeft.put(6, new Translation2d(13.539017606564588, 2.8230016967037854));
        blueRight.put(21, new Translation2d(5.771896, 4.2116375));
        blueLeft.put(21, new Translation2d(5.771896, 3.8401625));
        blueRight.put(20, new Translation2d(4.969311606564587, 5.228798303296214));
        blueLeft.put(20, new Translation2d(5.2910183934354125, 5.043060803296214));
        blueRight.put(19, new Translation2d(3.687627606564587, 5.043060803296214));
        blueLeft.put(19, new Translation2d(4.009334393435411, 5.228798303296214));
        blueRight.put(18, new Translation2d(3.20675, 3.8401625));
        blueLeft.put(18, new Translation2d(3.20675, 4.2116375));
        blueRight.put(17, new Translation2d(4.00933439343541, 2.8230016967037854));
        blueLeft.put(17, new Translation2d(3.6876276065645865, 3.0087391967037855));
        blueRight.put(22, new Translation2d(5.2910183934354125, 3.0087391967037855));
        blueLeft.put(22, new Translation2d(4.969311606564588, 2.8230016967037854));
        if (alliance.isPresent()) {
            targetTagSets = alliance.get() == DriverStation.Alliance.Red
                    ? Arrays.asList(10, 11, 6, 7, 8, 9)
                    : Arrays.asList(21, 20, 19, 18, 17, 22);
            // int[] targetTagSet = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new
            // int[]{10,11,6,7,8,9} : new int[]{21, 20, 19,18, 17, 22};
        } else {
            return Commands.none();
        }
        double minDistance = Double.MAX_VALUE;
        for (int tag : targetTagSets) {
            Pose2d pose = fieldLayout.getTagPose(tag).orElse(new Pose3d()).toPose2d();
            Translation2d translate = pose.minus(drivePose).getTranslation();
            double distance = translate.getNorm();

            if (distance < minDistance) {
                tagPose = pose;
                targetId = tag;
                minDistance = distance;
            }
        }
        int target = targetId;
        // int path = pathId.get();
        int path = targetTagSets.indexOf(target);
        HashMap<Integer, Translation2d> selectedMap;
        if (alliance.isPresent()) {
            if (isLeftAlign) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    selectedMap = redLeft;
                } else {
                    selectedMap = blueLeft;
                }
            } else {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    selectedMap = redRight;
                } else {
                    selectedMap = blueRight;
                }
            }
        } else {
            return Commands.none();
        }

        tagPose = fieldLayout.getTagPose(target).orElse(new Pose3d()).toPose2d();
        PathConstraints constraints =
                new PathConstraints(3.0, 2.1, Units.degreesToRadians(540), Units.degreesToRadians(720));
        Translation2d translate = selectedMap.get(target);
        if (translate != null) {
            Pose2d pose = new Pose2d(
                    translate.getX(), translate.getY(), tagPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
            // drivetrain.selectPosePublisher.set(pose);
            Logger.recordOutput("FieldSimulation/AlignPose", pose);
            List<List<String>> characterLists = Arrays.asList(
                    Arrays.asList("G", "H"),
                    Arrays.asList("I", "J"),
                    Arrays.asList("K", "L"),
                    Arrays.asList("A", "B"),
                    Arrays.asList("C", "D"),
                    Arrays.asList("E", "F"));

            String selectedCharacter = characterLists.get(path).get(isLeftAlign ? 0 : 1);
            try {
                PathPlannerPath paths = PathPlannerPath.fromPathFile(selectedCharacter + " Score Pathfind");
                pathfindingCommand = instance.PFThenFollowPath(paths, constraints);
            } catch (Exception e) {
                // System.out.println("Path not found, switching to pathfindToPose. Error: " + e);
                pathfindingCommand = instance.PFToPose(pose, constraints);
            }
            // pathfindingCommand.initialize();
        } else {
            pathfindingCommand = Commands.none();
            // DriverStation.reportWarning("Reef Align Null",true);
            System.out.println("Reef Align Null");
        }
        return pathfindingCommand;
    }
}

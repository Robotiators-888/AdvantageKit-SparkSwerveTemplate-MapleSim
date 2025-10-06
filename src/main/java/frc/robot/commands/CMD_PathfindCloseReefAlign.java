// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AIRobotInSimulation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class CMD_PathfindCloseReefAlign extends Command {
    private static final double FIELD_LENGTH_M = 17.55; // X-dimension (57 ft. 6⅞ in.)
    private static final double FIELD_WIDTH_M = 8.05; // Y-dimension (26 ft. 5 in.)
    private static final double MIN_X_BLUE = 0.5; // Start of Blue Alliance half
    private static final double MAX_X_BLUE =
            FIELD_LENGTH_M / 2.0 - 0.2; // End of Blue Alliance half, slightly offset (~8.575m)
    private static final Random random = new Random();
    private static final double MIN_Y = 0.5;
    private static final double MAX_Y = FIELD_WIDTH_M - 0.5;
    Command pathfindingCommand;
    boolean isLeftAlign = false;
    Vision photonVision;
    Drive drivetrain;
    Supplier<Integer> targetId;
    Supplier<Integer> pathId;
    private List<Integer> targetTagSet;
    HashMap<Integer, Translation2d> redLeft = new HashMap<>();
    HashMap<Integer, Translation2d> redRight = new HashMap<>();
    HashMap<Integer, Translation2d> blueLeft = new HashMap<>();
    HashMap<Integer, Translation2d> blueRight = new HashMap<>();

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    /** Creates a new CMD_PathfindReefAlign. */
    public CMD_PathfindCloseReefAlign(Drive drivetrain, Vision photonVision, boolean isLeftAlign) {
        this.photonVision = photonVision;
        this.drivetrain = drivetrain;
        this.isLeftAlign = isLeftAlign;

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

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d tagPose = new Pose2d();
        Integer targetId = 7;

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            targetTagSet = alliance.get() == DriverStation.Alliance.Red
                    ? Arrays.asList(10, 11, 6, 7, 8, 9)
                    : Arrays.asList(21, 20, 19, 18, 17, 22);
            // int[] targetTagSet = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new
            // int[]{10,11,6,7,8,9} : new int[]{21, 20, 19,18, 17, 22};
        }
        double minDistance = Double.MAX_VALUE;
        for (int tag : targetTagSet) {
            Pose2d pose = fieldLayout.getTagPose(tag).orElse(new Pose3d()).toPose2d();
            Translation2d translate = pose.minus(drivetrain.getPose()).getTranslation();
            double distance = translate.getNorm();

            if (distance < minDistance) {
                tagPose = pose;
                targetId = tag;
                minDistance = distance;
            }
        }
        int target = targetId;
        // int path = pathId.get();
        int path = targetTagSet.indexOf(target);
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
            return;
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
                pathfindingCommand = AutoBuilder.pathfindThenFollowPath(paths, constraints);
            } catch (Exception e) {
                // System.out.println("Path not found, switching to pathfindToPose. Error: " + e);
                pathfindingCommand = AutoBuilder.pathfindToPose(pose, constraints);
            }
            pathfindingCommand.initialize();
        } else {
            pathfindingCommand = Commands.none();
            // DriverStation.reportWarning("Reef Align Null",true);
            System.out.println("Reef Align Null");
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pathfindingCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pathfindingCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pathfindingCommand.isFinished();
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

        return AllianceFlipUtil.apply(targetPose, alliance.get() == DriverStation.Alliance.Red);
    }

    public static Command pathfindToRandomPose(
            Optional<DriverStation.Alliance> alliance, AIRobotInSimulation instance) {

        Logger.recordOutput("Pathfind Test Out", 0);
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
        Logger.recordOutput("Pathfind Test Out", 1);
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

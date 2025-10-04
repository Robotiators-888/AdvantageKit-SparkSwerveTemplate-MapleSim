import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.AllianceFlipUtil;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import java.util.List;
import java.util.Random;

public class RobotContainer {
    
    // ==============================================================================
    // 1. CONSTANTS AND UTILITIES
    // ==============================================================================
    
    // Field Dimensions (Based on REEFSCAPE 2025 FRC field dimensions: 17.55m x 8.05m)
    private static final double FIELD_LENGTH_M = 17.55; // X-dimension (57 ft. 6⅞ in.)
    private static final double FIELD_WIDTH_M = 8.05;  // Y-dimension (26 ft. 5 in.)

    // Define the alliance side bounds for random path generation (Blue Alliance Side)
    // We use a small margin (0.5m) to avoid starting exactly on the nav grid bounds.
    private static final double MIN_X_BLUE = 0.5;      // Start of Blue Alliance half
    private static final double MAX_X_BLUE = FIELD_LENGTH_M / 2.0 - 0.2; // End of Blue Alliance half, slightly offset (~8.575m)

    private static final double MIN_Y = 0.5;
    private static final double MAX_Y = FIELD_WIDTH_M - 0.5;

    // Fixed Target Poses (Examples based on a hypothetical 2025 game)
    // NOTE: Poses are Field-Relative (Blue Alliance Origin 0,0)
    
    /** Pose for the Feeder Station (Example: Blue Side) */
    private static final Pose2d FEEDER_STATION_POSE = new Pose2d(
        8.0, 7.0, Rotation2d.fromDegrees(180)
    );

    // Common Target Poses for Reefsides (e.g., scoring locations near the goal)
    // Assuming 3 scoring points on the Blue Alliance side:
    private static final List<Pose2d> BLUE_REEF_POSES = List.of(
        new Pose2d(1.5, 6.0, Rotation2d.fromDegrees(0)),  // Reef 1
        new Pose2d(1.5, 4.1, Rotation2d.fromDegrees(0)),  // Reef 2 (Center)
        new Pose2d(1.5, 2.2, Rotation2d.fromDegrees(0))   // Reef 3
    );
    
    // Path Constraints (Replace with your actual robot constraints)
    private final PathConstraints constraints = new PathConstraints(
        4.0, // Max Velocity (m/s)
        3.0, // Max Acceleration (m/s^2)
        360.0 * Math.PI / 180.0, // Max Angular Velocity (rad/s)
        720.0 * Math.PI / 180.0  // Max Angular Acceleration (rad/s^2)
    );

    // Random Number Generator
    private final Random random = new Random();

    // ==============================================================================
    // 2. PLACEHOLDER AND UTILITY COMMANDS (Your Subsystems Go Here)
    // ==============================================================================

    /** * Mocks the robot's drive subsystem (Replace with your actual Drive Subsystem reference) 
     * In a real FRC project, this would be a real object passed to the commands.
     */
    // private final DriveSubsystem m_drive = new DriveSubsystem();

    /** Mocks the pose estimator supplier */
    private final java.util.function.Supplier<Pose2d> m_poseSupplier = 
        () -> new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)); // Mocked current pose

    /** Mocks the pose reset consumer */
    private final java.util.function.Consumer<Pose2d> m_poseConsumer = 
        (pose) -> System.out.println("Resetting Pose to: " + pose); // Mocked reset

    /** Command provided by the user to simulate receiving a coral. */
    public final Command giveCoralCommand = new InstantCommand(
        () -> System.out.println("COMMAND: Giving robot L1 Simulation Coral."),
        // m_intakeSubsystem // Placeholder subsystem
        null 
    );

    /** Command provided by the user to shoot an L3 coral. */
    public final Command shootL3Command = new InstantCommand(
        () -> System.out.println("COMMAND: Shooting L3 Coral."),
        // m_shooterSubsystem // Placeholder subsystem
        null
    );

    /** Command provided by the user to shoot an L4 coral. */
    public final Command shootL4Command = new InstantCommand(
        () -> System.out.println("COMMAND: Shooting L4 Coral."),
        // m_shooterSubsystem // Placeholder subsystem
        null
    );

    // ==============================================================================
    // 3. NAMED COMMAND IMPLEMENTATIONS
    // ==============================================================================

    /**
     * NamedCommand: Randomly selects between shooting an L3 or L4 coral.
     * This command runs instantly, selecting which shoot command to execute.
     */
    public final Command randomShooterCommand = Commands.defer(
        () -> {
            boolean useL3 = random.nextBoolean();
            System.out.println("RANDOM SHOOTER: Choosing to shoot " + (useL3 ? "L3" : "L4") + " coral.");
            return useL3 ? shootL3Command : shootL4Command;
        },
        // m_shooterSubsystem // Placeholder subsystem
        null
    ).withName("RandomShoot");


    /**
     * NamedCommand: Finds the closest Reefside pose and pathfinds to it for alignment.
     * This allows for dynamic alignment based on the robot's position.
     */
    public final Command autoAlignReefCommand = Commands.defer(
        () -> {
            Pose2d currentPose = m_poseSupplier.get();
            Pose2d closestPose = findClosestReefPose(currentPose);
            
            System.out.println("AUTO ALIGN: Current Pose: " + currentPose);
            System.out.println("AUTO ALIGN: Pathfinding to closest Reefside at: " + closestPose);

            // Pathfind to the closest pose
            return AutoBuilder.pathfindToPose(
                closestPose, 
                constraints, 
                0.0, 
                0.0
            );
        },
        // m_drive // Placeholder subsystem
        null
    ).withName("AutoAlignReef");
    
    /**
     * NamedCommand: Generates a random Pose2d on your alliance side and pathfinds to it.
     */
    public final Command generateRandomPathCommand = Commands.defer(
        () -> {
            Pose2d randomTarget = generateRandomTargetPose(AutoBuilder.getAlliance());
            
            System.out.println("RANDOM PATH: Generated target pose: " + randomTarget);
            
            // Pathfind to the random pose
            return AutoBuilder.pathfindToPose(
                randomTarget, 
                constraints, 
                0.0, 
                0.0
            );
        },
        // m_drive // Placeholder subsystem
        null
    ).withName("PathfindRandom");
    
    // ==============================================================================
    // 4. PRIVATE LOGIC METHODS
    // ==============================================================================

    /**
     * Logic to determine the closest Reefside pose.
     */
    private Pose2d findClosestReefPose(Pose2d currentPose) {
        // Use the current alliance to select the correct set of target poses
        boolean isRed = AutoBuilder.getAlliance() == Alliance.Red;
        List<Pose2d> reefPoses = isRed ? BLUE_REEF_POSES.stream().map(Pose2d::mirror).toList() : BLUE_REEF_POSES;

        Translation2d currentTranslation = currentPose.getTranslation();
        Pose2d closestPose = null;
        double minDistanceSq = Double.MAX_VALUE;

        for (Pose2d targetPose : reefPoses) {
            double distanceSq = currentTranslation.getDistanceSq(targetPose.getTranslation());
            if (distanceSq < minDistanceSq) {
                minDistanceSq = distanceSq;
                closestPose = targetPose;
            }
        }
        return closestPose;
    }
    public Command findRandomReachablePoseCommand() {
        Command pathfindingCommand = null;

        while (pathfindingCommand == null) {
            try {
                // 1. Generate a random pose on your field
                Pose2d randomPose = generateRandomPose();
                
                // 2. Attempt to create the pathfinding command
                pathfindingCommand = new PathfindToPose(
                    randomPose,
                    constraints,
                    0.0, // Goal end velocity (m/s)
                    swerveSubsystem::getPose,
                    swerveSubsystem::resetOdometry,
                    swerveSubsystem::getChassisSpeeds,
                    swerveSubsystem::driveRobotRelative,
                    swerveSubsystem::isRedAlliance);
                
            } catch (IllegalArgumentException e) {
                // Pathfinding failed, likely due to an unreachable pose.
                // The loop will continue, and a new pose will be generated.
                System.err.println("Failed to create path to random pose. Regenerating...");
            }
        }
        
        return pathfindingCommand;
    }
    /**
     * Logic to generate a random Pose2d on the robot's side of the field.
     */
    private Pose2d generateRandomTargetPose() {
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
        
        return AllianceFlipUtil.apply(targetPose);
    }

    // ==============================================================================
    // 5. ROBOT CONTAINER SETUP
    // ==============================================================================

    public RobotContainer() {
        // Configure the command registrations
        configureNamedCommands();
        configureAutoBuilder();
    }
    
    /**
     * This method is where you register all the commands you want to use
     * inside the PathPlanner GUI as Named Commands.
     */
    private void configureNamedCommands() {
        // 1. The custom pathfinding and logic commands
        AutoBuilder.configureCommand(generateRandomPathCommand);
        AutoBuilder.configureCommand(autoAlignReefCommand);
        AutoBuilder.configureCommand(randomShooterCommand);

        // 2. The simple action commands
        AutoBuilder.configureCommand(giveCoralCommand.withName("GiveCoral"));
    }

    /**
     * MUST be called to properly initialize AutoBuilder for PathPlannerLib.
     */
    private void configureAutoBuilder() {
        AutoBuilder.configure(
            m_poseSupplier, // Supplier of current robot pose (e.g., m_drive::getPose)
            m_poseConsumer, // Consumer for seeding pose against auto (e.g., m_drive::resetPose)
            // The method to get the chassis speeds (e.g., m_drive::getSpeeds)
            // Replace with your actual drivetrain supplier
            () -> new com.pathplanner.lib.util.ReplanningConfig(), // Path replanning configuration
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This is crucial for matching random/fixed poses to the correct side.
            () -> DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent(),
            // The Drivetrain Subsystem itself (or null if not using)
            null
        );
    }
}

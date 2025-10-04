// subsystems/intake/IntakeIOSim.java
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class IntakeIOSim extends SubsystemBase {
    private final IntakeSimulation intakeSimulation;
    private final SwerveDriveSimulation driveTrain;

    public IntakeIOSim(SwerveDriveSimulation driveTrain) {
        // Here, create the intake simulation with respect to the intake on your real robot
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                "Coral",
                // Specify the drivetrain to which this intake is attached
                driveTrain,
                // Width of the intake
                Meters.of(0.4),
                // The extension length of the intake beyond the robot's frame (when activated)
                Meters.of(0.2),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.BACK,
                // The intake can hold up to 1 note
                1);
        this.driveTrain = driveTrain;
    }

    public void setRunning(boolean runIntake) {
        if (runIntake) {
            intakeSimulation
                    .startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with
            // game pieces
        } else {
            intakeSimulation
                    .stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
        }
    }

    public boolean isCoralInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    public void launchCoralLevel3() {
        // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            // ShooterIOSim.launchCoral(); // notify the simulated flywheels to launch a note
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            driveTrain.getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                            new Translation2d(0.35, 0),
                            // Obtain robot speed from drive simulation
                            driveTrain.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            driveTrain.getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            Meters.of(1.28),
                            // The initial speed of the coral
                            MetersPerSecond.of(4),
                            // The coral is ejected at a 35-degree slope
                            Degrees.of(-35)));
        }
    }

    public void launchCoralLevel4() {
        // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            // ShooterIOSim.launchCoral(); // notify the simulated flywheels to launch a note
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            driveTrain.getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                            new Translation2d(0.46, 0),
                            // Obtain robot speed from drive simulation
                            driveTrain.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            driveTrain.getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            Meters.of(2.1),
                            // The initial speed of the coral
                            MetersPerSecond.of(1),
                            // The coral is ejected vertically downwards
                            Degrees.of(-90)));
        }
    }

    public void intakeCoralStation() {
        intakeSimulation.addGamePieceToIntake();
    }
}

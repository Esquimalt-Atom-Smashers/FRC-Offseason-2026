// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Mass;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSim;
import frc.robot.simulation.HubLightSimDisplay;
import frc.robot.simulation.SimStartingPoseFullFieldSim;
import frc.robot.simulation.SimStartingPoseUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera2;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// Controller
	private final CommandXboxController driverController = new CommandXboxController(0);

	// Competition Toggle
	@AutoLogOutput(key = "CompetitionToggle")
	private boolean isCompetition   = false;

	// Subsystems Toggle
	private boolean isDriveEnabled  = true;
	private boolean isVisionEnabled = true;

	// Simulation Toggle
	private boolean halfFuelOnly    = false;
	private boolean fuelSimEnabled  = true;

	// Subsystems
	private final Drive drive;
	@SuppressWarnings("unused")
	private final Vision vision;

	// Drive Commands
	private final TeleopDrive teleopDrive;

	// Robot Auto-Face Hub Target Mode
	@AutoLogOutput(key = "TeleopDrive/IsFacingHub")
	private boolean isFacingHub = false;
	private ProfiledPIDController faceTargetController;

	// Field view (robot pose)
	private final Field2d field = new Field2d();

	// Robot-centric vs Field-centric drive
	@AutoLogOutput(key = "TeleopDrive/IsRobotCentric")
	private boolean isRobotCentric = false;

	// Manual Override
	@AutoLogOutput(key = "ManualOverride/Driver")
	public static boolean driverManualOverride = false;

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;
	private boolean chooserRepublishPending = true;
	private boolean ntConnectedLastTick = false;

	// Drive Simulation
	private SwerveDriveSimulation driveSimulation = null;

	// Fuel Simulation (robot-ball collision in SIM)
	private final FuelSim fuelSim = new FuelSim();

	// Starting Pose Chooser (SIM only; null outside simulation)
	private SimStartingPoseFullFieldSim simSpawnSim = null;

	/** Field Reset SmartDashboard Key */
	private static final String SIM_RESET_SIMULATION_FIELD_KEY = "SimField/ResetSimulationField";

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Initialize subsystems based on mode (REAL, SIM, or REPLAY)
		switch (Constants.currentMode) {
			// Real robot, instantiate hardware IO implementations
			case REAL:
				if (isDriveEnabled) {
					drive = new Drive(
							new GyroIOPigeon2(),
							new ModuleIOTalonFX(TunerConstants.FrontLeft), new ModuleIOTalonFX(TunerConstants.FrontRight),
							new ModuleIOTalonFX(TunerConstants.BackLeft), new ModuleIOTalonFX(TunerConstants.BackRight),
							(pose) -> {});
				} else {
					drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
				}

				// Initialize Vision after Drive (Vision needs the Drive reference)
				if (isVisionEnabled) {
					vision = new Vision(drive,
							new VisionIOPhotonVision(camera0Name, robotToCamera0),
							new VisionIOPhotonVision(camera1Name, robotToCamera1),
							new VisionIOPhotonVision(camera2Name, robotToCamera2));
				} else {
					vision = new Vision(drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
				}
				break;

			// Sim robot, instantiate physics sim IO implementations
			case SIM:
				// Configure simulation timing for accurate physics
				// 5 ticks per 20ms period = 4kHz effective control loop rate
				SimulatedArena.overrideSimulationTimings(
						edu.wpi.first.units.Units.Seconds.of(0.02), // 20ms robot period
						5); // 5 simulation ticks per period

				// Rebuilt arena, including ramp colliders
				SimulatedArena.overrideInstance(new frc.robot.simulation.Arena2026Rebuilt(true));

				driveSimulation = new SwerveDriveSimulation(
						Drive.mapleSimConfig, SimStartingPoseUtil.poseForStem(SimStartingPoseUtil.STEM_RIGHT, false));
				createRobotShape(driveSimulation, 0.0);
				SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
				drive = new Drive(
						new GyroIOSim(driveSimulation.getGyroSimulation()),
						new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0], 0),
						new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1], 1),
						new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2], 2),
						new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3], 3),
						driveSimulation::setSimulationWorldPose);

				// Initialize Vision after Drive (Vision needs the Drive reference)
				vision = new Vision(drive,
						new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
						new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose),
						new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, driveSimulation::getSimulatedDriveTrainPose));

				// Fuel sim
				if (fuelSimEnabled) {
					registerFuelSimRobotBody(drive, driveSimulation);
					configureFuelSim();
				}

				// Starting pose chooser
				simSpawnSim = new SimStartingPoseFullFieldSim();
				simSpawnSim.init();
				break;

			// Replayed robot, disable IO implementations
			default:
				drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
				vision = new Vision(drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
				break;
		}

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------------- Drive Commands --------------------------------------
		/// -------------------------------------------------------------------------------------------
		// Initialize the face-target PID. Rotates the robot so its heading points at the alliance hub.
		faceTargetController = new ProfiledPIDController(
				DriveCommands.getAngleKp(),
				0.0,
				DriveCommands.getAngleKd(),
				new TrapezoidProfile.Constraints(DriveCommands.getAngleMaxVelocity(), DriveCommands.getAngleMaxAcceleration()));
		faceTargetController.enableContinuousInput(-Math.PI, Math.PI);

		teleopDrive = new TeleopDrive(
				drive, driverController, () -> isRobotCentric, () -> isFacingHub, faceTargetController);
		teleopDrive.setManualOverrideSupplier(() -> driverManualOverride);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------------ Logger Dashboard -------------------------------------
		/// -------------------------------------------------------------------------------------------
		// Field view
		SmartDashboard.putData("Field", field);
		if (Constants.currentMode == Constants.Mode.SIM) {
			SmartDashboard.putBoolean("SimStartingPose/Apply", false);
			SmartDashboard.putBoolean("SimStartingPose/ResetToDefaults", false);
			SmartDashboard.putBoolean(SIM_RESET_SIMULATION_FIELD_KEY, false);
		}

		// Set up auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Things to set up if not in competition
		if (!isCompetition) {
			// Set up swerve calibration programs
			DriveCommands.swerveCalibration(autoChooser, drive);
		}

		// Configure button bindings
		configureDriverBindings();
	} // End RobotContainer Constructor

	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------------ Controller Input -----------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
	/** Configure driver controls. */
	private void configureDriverBindings() {
		drive.setDefaultCommand(teleopDrive);

		// In sim, snap odometry to the Maple pose. Otherwise zero the heading and keep translation.
		final Runnable resetGyro =
				Constants.currentMode == Constants.Mode.SIM
						? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
						: () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

		// Toggle face-target mode when Y is pressed
		driverController.y().onTrue(Commands.runOnce(() -> {
			isFacingHub = !isFacingHub;
			// Reset the PID controller when enabling face-target mode
			if (isFacingHub) {
				faceTargetController.reset(drive.getRotation().getRadians());
			}
		}, drive));

		// Start: toggle robot-centric drive, or reset the gyro while manual override is on
		driverController.start().onTrue(
				new ConditionalCommand(
						Commands.runOnce(() -> isRobotCentric = !isRobotCentric, drive),
						Commands.runOnce(resetGyro, drive).ignoringDisable(true),
						() -> !driverManualOverride));

		// Toggle driver manual override
		driverController.back().onTrue(
				new ConditionalCommand(
						Commands.runOnce(() -> driverManualOverride = false),
						Commands.runOnce(() -> driverManualOverride = true),
						() -> driverManualOverride));
	} // End configureDriverBindings

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	} // End getAutonomousCommand

	/**
	 * Re-publishes chooser selected values once on NT connect (and again on reconnect) so dashboard chooser widgets
	 * bind reliably without manual topic re-confirm.
	 */
	public void republishDashboardChoosersOnNtConnect() {
		NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
		boolean ntConnected = ntInstance.isConnected();
		if (ntConnected && !ntConnectedLastTick) {
			chooserRepublishPending = true;
		}
		ntConnectedLastTick = ntConnected;
		if (!ntConnected || !chooserRepublishPending) {
			return;
		}

		republishChooserSelectedEntry("Auto Choices");
		if (simSpawnSim != null) {
			simSpawnSim.republishSelectedChoices();
		}
		chooserRepublishPending = false;
	} // End republishDashboardChoosersOnNtConnect

	private static void republishChooserSelectedEntry(String chooserRootKey) {
		NetworkTable chooserTable = NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(chooserRootKey);
		String selected = chooserTable.getEntry("selected").getString("");
		if (selected == null || selected.isEmpty()) {
			selected = chooserTable.getEntry("active").getString("");
		}
		if (selected != null && !selected.isEmpty()) {
			chooserTable.getEntry("selected").setString(selected);
		}
	} // End republishChooserSelectedEntry

	/** Stops the drivetrain. */
	public void idleAllSubsystems() {
		if (drive != null) {
			drive.stop();
		}
	} // End idleAllSubsystems

	/**
	 * Updates Field2d from drive odometry. Skipped in SIM mode, where the displayed pose is the physics-sim pose.
	 */
	public void updateFieldPose() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return;
		}
		field.setRobotPose(drive.getPose());
	} // End updateFieldPose

	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------------ Simulation Only ------------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
	/**
	 * Applies the chooser starting pose and resets fuel when fuel sim is enabled. No-op outside simulation.
	 */
	public void resetSimulationField() {
		if (Constants.currentMode != Constants.Mode.SIM) {
			return;
		}
		runSimulationFieldReset();
	} // End resetSimulationField

	/** Pulses {@code SimStartingPose/Apply}, teleports to that pose, then resets fuel when fuel sim is enabled. */
	private void runSimulationFieldReset() {
		if (simSpawnSim == null) {
			return;
		}
		SmartDashboard.putBoolean("SimStartingPose/Apply", true);
		pollSimStartingPosesApplyButton();
		if (fuelSimEnabled) {
			fuelSim.resetFuel();
		}
	} // End runSimulationFieldReset

	/** If {@value #SIM_RESET_SIMULATION_FIELD_KEY} is true, clears it and runs {@link #runSimulationFieldReset()}. */
	private void pollResetSimulationFieldDashboardButton() {
		if (!SmartDashboard.getBoolean(SIM_RESET_SIMULATION_FIELD_KEY, false)) {
			return;
		}
		SmartDashboard.putBoolean(SIM_RESET_SIMULATION_FIELD_KEY, false);
		runSimulationFieldReset();
	} // End pollResetSimulationFieldDashboardButton

	/** Applies the chooser pose to the Maple sim and to drive odometry. */
	private void applyPrimarySimStartingPoseFromChooser(Pose2d appliedPose) {
		driveSimulation.setSimulationWorldPose(appliedPose);
		drive.setPose(appliedPose);
	} // End applyPrimarySimStartingPoseFromChooser

	/** Consumes {@code SimStartingPose/Apply} and teleports the robot to the chooser pose. */
	private void pollSimStartingPosesApplyButton() {
		simSpawnSim.pollApplyButton(true, this::applyPrimarySimStartingPoseFromChooser);
	} // End pollSimStartingPosesApplyButton

	/**
	 * Replaces the MapleSim bumper rectangle with a U-shaped footprint (rear gap centered on Y).
	 * {@code frontExtensionBeyondBumpersM} lengthens the hull in +X.
	 * Drive-sim collision only; FuelSim keeps the rectangular robot box.
	 */
	private static void createRobotShape(SwerveDriveSimulation driveSimulation, double frontExtensionBeyondBumpersM) {
		// MapleSim / dyn4j dimensions are meters.
		double bumperLengthX = Constants.Dimensions.FULL_LENGTH.in(Meters);
		double bumperWidthY = Constants.Dimensions.FULL_WIDTH.in(Meters);

		// Rear gap: width = 17" (centered on robot Y=0) and depth = 3" (into the robot in -X).
		double gapHalfWidthY = 8.5 * 0.0254; // 8.5 in half-width
		double gapDepthX = 3.0 * 0.0254; // 3 in depth

		double halfLenX = bumperLengthX / 2.0;
		double halfWidthY = bumperWidthY / 2.0;

		double sideHeightY = halfWidthY - gapHalfWidthY;
		if (sideHeightY <= 0) {
			// Avoid generating invalid fixtures if constants are inconsistent.
			return;
		}

		// Preserve dyn4j mass/inertia (MapleSim applies forces using its original COM assumptions).
		// Recomputing mass from the U-shape shifts the COM forward and the physics becomes unstable.
		Mass oldMass = driveSimulation.getMass().copy();

		// Replace the default bumper rectangle with 3 rectangles:
		// - a front rectangle (full width) stopping at the rear gap
		// - a left rear strip
		// - a right rear strip
		driveSimulation.removeAllFixtures();

		// Front rectangle: x in [-halfLenX + gapDepthX, +halfLenX + frontExtensionBeyondBumpersM]
		double frontLeftX = -halfLenX + gapDepthX;
		double frontRightX = halfLenX + frontExtensionBeyondBumpersM;
		double frontWidthX = frontRightX - frontLeftX;
		double frontCenterX = (frontLeftX + frontRightX) * 0.5;
		Rectangle front = new Rectangle(frontWidthX, bumperWidthY);
		front.translate(frontCenterX, 0);
		BodyFixture frontFixture = new BodyFixture(front);
		frontFixture.setFriction(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_FRICTION);
		frontFixture.setRestitution(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_RESTITUTION);
		frontFixture.setDensity(0.0); // collision-only; keep oldMass for dynamics stability
		driveSimulation.addFixture(frontFixture);

		// Left and right strips: x in [-halfLenX, -halfLenX + gapDepthX], y outside the gap
		double stripWidthX = gapDepthX;
		double stripCenterX = -halfLenX + gapDepthX / 2.0;
		double sideCenterY = (gapHalfWidthY + halfWidthY) / 2.0;

		Rectangle left = new Rectangle(stripWidthX, sideHeightY);
		left.translate(stripCenterX, -sideCenterY);
		BodyFixture leftFixture = new BodyFixture(left);
		leftFixture.setFriction(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_FRICTION);
		leftFixture.setRestitution(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_RESTITUTION);
		leftFixture.setDensity(0.0); // collision-only; keep oldMass for dynamics stability
		driveSimulation.addFixture(leftFixture);

		Rectangle right = new Rectangle(stripWidthX, sideHeightY);
		right.translate(stripCenterX, sideCenterY);
		BodyFixture rightFixture = new BodyFixture(right);
		rightFixture.setFriction(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_FRICTION);
		rightFixture.setRestitution(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_RESTITUTION);
		rightFixture.setDensity(0.0); // collision-only; keep oldMass for dynamics stability
		driveSimulation.addFixture(rightFixture);

		// Restore the old mass/inertia so the COM stays where MapleSim expects.
		driveSimulation.setMass(oldMass);
	} // End createRobotShape

	/** Configures FuelSim for robot-ball collision in simulation. */
	private void configureFuelSim() {
		fuelSim.setShowHalfFuel(halfFuelOnly);
		fuelSim.spawnStartingFuel();
		fuelSim.start();
		SmartDashboard.putData(
				Commands.runOnce(() -> fuelSim.resetFuel()).withName("Reset Fuel").ignoringDisable(true));
	} // End configureFuelSim

	/** Registers this robot's bumper box with FuelSim so fuel collides with the chassis. */
	private void registerFuelSimRobotBody(Drive driveForFuel, SwerveDriveSimulation mapleDriveSimulation) {
		fuelSim.registerRobot(
				Constants.Dimensions.FULL_WIDTH.in(Meters),
				Constants.Dimensions.FULL_LENGTH.in(Meters),
				Constants.Dimensions.BUMPER_HEIGHT.in(Meters),
				mapleDriveSimulation::getSimulatedDriveTrainPose,
				driveForFuel::getFieldRelativeChassisSpeeds,
				mapleDriveSimulation);
	} // End registerFuelSimRobotBody

	/** Steps the simulation world. Only runs in SIM mode. */
	public void updateSimulation() {
		if (Constants.currentMode != Constants.Mode.SIM) {
			return;
		}

		pollResetSimulationFieldDashboardButton();
		simSpawnSim.pollResetToDefaults(true);
		pollSimStartingPosesApplyButton();

		SimulatedArena.getInstance().simulationPeriodic();

		// Hub light simulation
		HubLightSimDisplay.update();

		// Robot pose for visualization
		Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
		Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);

		// Update field view
		field.setRobotPose(robotPose);

		// Fuel sim (robot-ball collision) and hub scores
		if (fuelSimEnabled) {
			fuelSim.updateSim();
			Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
			Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
		}
	} // End updateSimulation
}

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * SIM-only starting pose chooser for the primary robot. Published once under
 * {@code SimStartingPose/Primary-Robot}. Selection is read from NetworkTables so a local reset write is visible
 * immediately.
 */
public final class SimStartingPoseFullFieldSim {

	private static final String DASHBOARD_KEY = "SimStartingPose/Primary-Robot";
	private static final String DEFAULT_STEM = SimStartingPoseUtil.STEM_RIGHT;

	@FunctionalInterface
	public interface PoseSink {
		void accept(Pose2d pose);
	} // End PoseSink

	private SendableChooser<String> chooser;

	/** Publishes the primary-robot starting-pose chooser. */
	public void init() {
		chooser = new SendableChooser<>();
		for (String stem : SimStartingPoseUtil.PATH_STEMS) {
			chooser.addOption(stem, stem);
		}
		chooser.setDefaultOption(DEFAULT_STEM, DEFAULT_STEM);
		SmartDashboard.putData(DASHBOARD_KEY, chooser);
	} // End init

	/**
	 * Stem currently selected, read from the NT {@code selected} entry. Bypasses {@link SendableChooser#getSelected()}
	 * because its internal state is not updated by local NT writes.
	 */
	public String selectedStem() {
		String ntValue = NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(DASHBOARD_KEY)
				.getEntry("selected")
				.getString("");
		if (ntValue == null || ntValue.isEmpty()) {
			return DEFAULT_STEM;
		}
		return ntValue;
	} // End selectedStem

	/** Reset button: writes the default stem back to the primary chooser. Does not teleport the robot. */
	public void pollResetToDefaults(boolean simMode) {
		if (!simMode || chooser == null) {
			return;
		}
		if (!SmartDashboard.getBoolean("SimStartingPose/ResetToDefaults", false)) {
			return;
		}
		forceSelection(DEFAULT_STEM);
		SmartDashboard.putBoolean("SimStartingPose/ResetToDefaults", false);
	} // End pollResetToDefaults

	/** Apply button: teleports the primary robot to the pose for the selected stem (blue-origin path pose). */
	public void pollApplyButton(boolean simMode, PoseSink primaryPose) {
		if (!simMode || chooser == null || primaryPose == null) {
			return;
		}
		if (!SmartDashboard.getBoolean("SimStartingPose/Apply", false)) {
			return;
		}
		primaryPose.accept(SimStartingPoseUtil.poseForStem(selectedStem(), false));
		SmartDashboard.putBoolean("SimStartingPose/Apply", false);
	} // End pollApplyButton

	/** Re-publishes the selected stem so a dashboard chooser rebinds after NT reconnect. */
	public void republishSelectedChoices() {
		forceSelection(selectedStem());
	} // End republishSelectedChoices

	private static void forceSelection(String stem) {
		NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(DASHBOARD_KEY)
				.getEntry("selected")
				.setString(stem);
	} // End forceSelection
} // End SimStartingPoseFullFieldSim

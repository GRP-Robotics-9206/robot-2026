package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;


/**
 * IO implementation for the QuestNav vision system.
 */
public class VisionIOQuestNav implements VisionIO {
    private final QuestNav questNav;
    private final Transform3d robotToQuest;

    /**
     * Constructs a new VisionIOQuestNav.
     * * @param robotToQuest The physical transformation from the robot's center to the 
     * location and orientation of the Quest headset.
     */
    public VisionIOQuestNav(Transform3d robotToQuest) {
        this.robotToQuest = robotToQuest;
        questNav = new QuestNav();
    }

    /**
     * Synchronizes the QuestNav local coordinate system with a known field-relative pose.
     * * @param robotPos The current estimated 3D pose of the robot on the field.
     */
    @Override
    public void setPose(Pose3d robotPos) {
        Pose3d questPos = robotPos.transformBy(robotToQuest);
        questNav.setPose(questPos);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        questNav.commandPeriodic();
        inputs.connected = questNav.isConnected();

        // Retrieve all unread pose frames
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
            if (questNav.isTracking()) {
                Pose3d questPose = frame.questPose3d();
                double timestamp = frame.dataTimestamp();

                // Convert the headset's pose back to the robot's center coordinates
                Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());
                
                poseObservations.add(
                    new PoseObservation(
                        timestamp,
                        robotPose,
                        0.0, // QuestNav does not provide an ambiguity metric, so we will set it to 0.0 (most certain)
                        0, // QuestNav does not provide tag count, set to 0
                        0.0, // QuestNav does not provide tag distance, set to 0.0
                        PoseObservationType.QUESTNAV
                    )
                );

            }
        }

        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }
    }
}

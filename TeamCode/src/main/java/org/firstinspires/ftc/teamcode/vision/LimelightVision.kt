package org.firstinspires.ftc.teamcode.vision

import Distance
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.vision.VisionConstants.LimelightIdentification
import org.firstinspires.ftc.teamcode.vision.VisionConstants.LimelightConfiguration
import org.firstinspires.ftc.teamcode.vision.VisionConstants.AlignmentParameters
import kotlin.math.abs

class LimelightVision(val hw: HardwareMap) {
    private val limelight: Limelight3A

    init {
        // Ensures poll rate is between 1 - 250 Hz
        require(LimelightConfiguration.PollRateHz in 1 .. 250)
        limelight = hw.get(Limelight3A::class.java, LimelightIdentification.Id)

        // Configures the poll rate. Must be called before start()
        limelight.setPollRateHz(LimelightConfiguration.PollRateHz)
        // Sets the pipeline
        limelight.pipelineSwitch(LimelightConfiguration.PipelineIndex)
    }

    fun getDistance(): Pair<Distance, Distance> {
        // Gets the camera position relative to the closest detected aprilTag.
        // It is important to use cameraPoseTargetSpace and NOT robotPoseTargetSpace, as the last
        // one is useful when the camera is fixed, which in this case is not.
        val robotPoseRelativeToAprilTag: Pose3D = limelight.latestResult
            .fiducialResults[0].cameraPoseTargetSpace

        return Pair(Distance.fromMeters(robotPoseRelativeToAprilTag.position.z),
            Distance.fromMeters(robotPoseRelativeToAprilTag.position.x))
    }

    /* Determines if the limelight (and therefore, turret, NOT chassis) is at set point by comparing
    * the horizontal robot pose relative to the aprilTag with a tolerance defined by us.
    * Vertical comparison is not necessary for shooting.
    * Frontal comparison is not necessary for shooting, as it shouldn't keep us from shooting.
    * A tolerance value of 0 means perfectly centered to the aprilTag */
    fun isAtSetPoint(): Boolean {
        // Gets the camera position relative to the closest detected aprilTag.
        // It is important to use cameraPoseTargetSpace and NOT robotPoseTargetSpace, as the last
        // one is useful when the camera is fixed, which in this case is not.
        val robotPoseRelativeToAprilTag: Pose3D = limelight.latestResult
            .fiducialResults[0].cameraPoseTargetSpace

        // Compares the absolute horizontal displacement to the tolerance
        return abs(robotPoseRelativeToAprilTag.position.x) < AlignmentParameters.ShootingHorizontalTolerance.meters
    }

    /* Checks the latest results from limelight, validating that they are not null, they are valid
    * and the fiducial (aprilTags) results are not empty*/
    fun hasTarget(): Boolean {
        val results = limelight.latestResult
        return results != null && results.isValid && results.fiducialResults.isEmpty().not()
    }

    /* Returns the closest detection ID. If there is not target found, method will return -1 */
    fun getTargetId(): Int {
        // Closest detection will have index 0
        if (hasTarget()) return limelight.latestResult.fiducialResults[0].fiducialId
        return -1
    }

    /* Should be called when the robot starts -not when it's powered-, to save battery.
    * It also resumes data retrieving after calling pauseLimelight() */
    fun enableLimelight() = limelight.start()

    /* Pauses data retrieving. Can be resumed with enableLimelight() */
    fun pauseLimelight() = limelight.pause()

    /* Completely stops the limelight */
    fun disableLimelight() = limelight.stop()
}
package org.firstinspires.ftc.teamcode.vision

import Distance
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.vision.VisionConstants.LimelightPhysicalDescription
import org.firstinspires.ftc.teamcode.vision.VisionConstants.AprilTagsPhysicalDescription
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import kotlin.math.tan

class Limelight(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    var otos: SparkFunOTOS
) : SubsystemBase() {

    private var limelight: Limelight3A? = null
    private var obeliskId = 0
    var llResult: LLResult? = null

    private var ty = 0.0
    private var tx = 0.0
    private var ta = 0.0

    private var distanceFromLimelightToGoalInches = Distance.fromInches(0.0)

    init {
        limelight = hardwareMap.get<Limelight3A?>(
            Limelight3A::class.java,
            VisionConstants.LimelightIdentification.Id
        )

        // Retrieves pipeline
        limelight!!.pipelineSwitch(VisionConstants.LimelightConfiguration.PipelineIndex) // Gets the limelight pipeline
        // How many times per second the limelight receives data in seconds
        limelight!!.setPollRateHz(VisionConstants.LimelightConfiguration.PollRateHz)

        // Initialize otos
        otos = hardwareMap.get<SparkFunOTOS?>(SparkFunOTOS::class.java, "otos")
        // Setting an angular unit so the readings it returns are in that unit
        otos.setAngularUnit(AngleUnit.DEGREES)
        // Setting a linear unit so the readings it returns are in that unit
        otos.setLinearUnit(DistanceUnit.INCH)
        // Reset's the otos readings to set the front of the robot when
        otos.resetTracking()

        // Starts the limelight's readings
        limelight!!.start()
    }

    fun getTx(): Double = tx
    fun getTy(): Double = ty
    fun getTa(): Double = ta

    /**
     *  Receives an array of Ids that the limelight can track and ignores the other ones, returns the offset angle from te limelight
     *  lenses the filtered april tag
     *  @param filterArray the desired ids for the limelight to follow
     *  @return the offset angle from the limelight lenses to the filtered april tag
     */
    fun getAngleToGoal(filterArray: IntArray): Double {

        if (llResult!!.isValid && llResult != null) {
            val fiducialResult = llResult!!.fiducialResults

            for (detectedId in fiducialResult) {
                for (id in filterArray) {
                    if (detectedId.fiducialId == id) {
                        return tx
                    }
                }
            }
        }

        return 0.0
    }

    /**
     * Gets the distance from the limelight lenses to a filtered id and returns the distance in any unit desired
     *  @param filterArray the desired ids for the limelight to follow
     *  @return the distance from the filtered id to the limelight lenses
     */
    fun getDistanceToGoal(filterArray: IntArray): Distance {

        if (llResult!!.isValid && llResult != null) {
            val fiducialResult = llResult!!.fiducialResults

            for (detectedId in fiducialResult) {
                for (id in filterArray) {
                    if (detectedId.fiducialId == id) {
                        return distanceFromLimelightToGoalInches
                    }
                }
            }
        }

        return Distance.fromInches(0.0)
    }

    /**
     * Checks if the limelight detects any obelisk april tag and assigns that value to [obeliskId] so it can be retrieved from
     * [getMotifPattern]
     */
    private fun getObeliskId() {

        if (llResult!!.isValid && llResult != null) {
            val fiducialResult = llResult!!.fiducialResults

            outerLoop@ for (detectedId in fiducialResult) {
                for (aprilTagId in VisionConstants.AprilTagsIdentification.ObeliskIds) {
                    if (detectedId.fiducialId == aprilTagId) {
                        obeliskId = detectedId.fiducialId
                        break@outerLoop
                        break
                    }
                }
            }
        }
    }

    /**
     * Gets the obelisk april tag id and relates it to a [MotifPatterns] depending on the actual pattern of the match
     * @return the current motif pattern
     */
    fun getMotifPattern(): MotifPatterns {
        return when (obeliskId) {
            21 -> MotifPatterns.GREEN_PURPLE_PURPLE
            22 -> MotifPatterns.PURPLE_GREEN_PURPLE
            23 -> MotifPatterns.PURPLE_PURPLE_GREEN
            else -> MotifPatterns.NO_PATTERN_DETECTED
        }
    }

    override fun periodic() {

        // Updating limelights' robot orientation with the Yaw
        limelight!!.updateRobotOrientation(otos.getPosition().h)

        // LLResult is like a container full of information about what Limelight sees
        llResult = limelight!!.getLatestResult()

        getObeliskId()
        // Math to calculate distance was taken from documentation:
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance#using-area-to-estimate-distance
        // The condition verifies whether the LimeLight Result is a valid statement
        if (llResult != null && llResult!!.isValid()) {

            // Offset to target in degrees (from crosshair)
            val targetOffsetAngle_Vertical = Angle.fromDegrees(llResult!!.getTy())
            // Needs to be in radians for tan() method
            val angleToGoalRadians =
                Math.toRadians(LimelightPhysicalDescription.LLMountAngleFromHorizontal.degrees + targetOffsetAngle_Vertical.degrees)

            // Calculated distance from limelight lens to goal (in inches)
            distanceFromLimelightToGoalInches =
                (AprilTagsPhysicalDescription.GoalHeightFromGround - LimelightPhysicalDescription.LLHeightFromGroundToLens) / tan(angleToGoalRadians)
            telemetry.addData("TargetDistanceInches", distanceFromLimelightToGoalInches.inches)

            // We will first get a (MetaTag2) Pose3D. From here, we will extract its Tx, Ty & Ta components
            tx = llResult!!.getTx()
            ty = llResult!!.getTy()
            ta = llResult!!.getTa()

            val botPose = llResult!!.getBotpose_MT2()
//            telemetry.addData(
//                "Tx",
//                tx
//            ) // Represents how far left/right the target is (in degrees)
//            telemetry.addData(
//                "Ty",
//                ty
//            ) // Represents how far up/down the target is (in degrees)
//            telemetry.addData("Ta", ta) // Represents how big the AprilTag looks
//
//            // according to the camera field of view (0-100%)
//            telemetry.addData("BotPose", botPose.toString())
//            telemetry.addData("Yaw", botPose.getOrientation().getYaw())

            /*
             * It is important to notice that the Full3D option should be enabled
             * */
        } else {
            tx = 0.0
            ty = 0.0
            ta = 0.0
        }
    } /*
     * ESTIMATING DISTANCE
     *
     *   1. Place the robot at a fixed, measured distance from the AprilTag
     *   2. Get the how big the AprilTag looks from the camera field of view, i.e. the Ta param
     *   3. Get a curve / regression out of all values
     *   4. Work backwards and from the curve, get the distance to the current point
     *
     * */
}
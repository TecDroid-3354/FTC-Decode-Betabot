package org.firstinspires.ftc.teamcode.vision

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

    private var ty = 0.0
    private var tx = 0.0
    private var ta = 0.0

    init {
        limelight = hardwareMap.get<Limelight3A?>(
            Limelight3A::class.java,
            VisionConstants.LimelightIdentification.Id
        ) // Retrieves pipeline
        limelight!!.pipelineSwitch(VisionConstants.LimelightConfiguration.PipelineIndex) // Gets the limelight pipeline
        limelight!!.setPollRateHz(VisionConstants.LimelightConfiguration.PollRateHz)

        otos = hardwareMap.get<SparkFunOTOS?>(SparkFunOTOS::class.java, "otos")
        otos.setAngularUnit(AngleUnit.DEGREES)
        otos.setLinearUnit(DistanceUnit.INCH)
        otos.resetTracking()
    }

    fun start() {
        // We start the limelight specifically at this point so that it doesn't take any energy
        // before the start button is pressed in match
        limelight!!.start()
    }

    fun getTx(): Double = tx
    fun getTy(): Double = ty
    fun getTa(): Double = ta

    fun getClasifierTx(filterArray: IntArray): Double {
        val llResult = limelight!!.latestResult

        if (llResult.isValid && llResult != null) {
            val fiducialResult = llResult.fiducialResults

            for (detectedId in fiducialResult) {
                for (id in filterArray) {
                    if (detectedId.fiducialId == id) {
                        return llResult.tx
                    }
                }
            }
        }

        return 0.0
    }

    private fun getMotifPattern(): MotifPatterns {

        val llResult = limelight!!.latestResult
        var obeliskId = 0

        if (llResult.isValid && llResult != null) {
            val fiducialResult = llResult.fiducialResults

            for (detectedId in fiducialResult) {
                for (aprilTagId in VisionConstants.AprilTagsIdentification.ObeliskIds) {
                    if (detectedId.fiducialId == aprilTagId) {
                        obeliskId = detectedId.fiducialId
                    }
                }
            }
        }

        return when (obeliskId) {
            21 -> MotifPatterns.GREEN_PURPLE_PURPLE
            22 -> MotifPatterns.PURPLE_GREEN_PURPLE
            23 -> MotifPatterns.PURPLE_PURPLE_GREEN
            else -> MotifPatterns.NO_PATTERN_DETECTED
        }
    }

    override fun periodic() {
        telemetry.addData("orientationOTOS", otos.getPosition().h)
        telemetry.addData("Motif", getMotifPattern().pattern.toString())

        //telemetry.addData("Id detected", getObeliskId())

        // Updating limelights' robot orientation with the Yaw
        limelight!!.updateRobotOrientation(otos.getPosition().h)

        // LLResult is like a container full of information about what Limelight sees
        val llResult = limelight!!.getLatestResult()

        // Math to calculate distance was taken from documentation:
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance#using-area-to-estimate-distance
        // The condition verifies whether the LimeLight Result is a valid statement
        if (llResult != null && llResult.isValid()) {

            // Offset to target in degrees (from crosshair)
            val targetOffsetAngle_Vertical = Angle.fromDegrees(llResult.getTy())
            // Needs to be in radians for tan() method
            val angleToGoalRadians =
                Math.toRadians(LimelightPhysicalDescription.LLMountAngleFromHorizontal.degrees + targetOffsetAngle_Vertical.degrees)

            // Calculated distance from limelight lens to goal (in inches)
            val distanceFromLimelightToGoalInches =
                (AprilTagsPhysicalDescription.GoalHeightFromGround - LimelightPhysicalDescription.LLHeightFromGroundToLens) / tan(angleToGoalRadians)
            telemetry.addData("TargetDistanceInches", distanceFromLimelightToGoalInches.inches)

            // We will first get a (MetaTag2) Pose3D. From here, we will extract its Tx, Ty & Ta components
            tx = llResult.getTx()
            ty = llResult.getTy()
            ta = llResult.getTa()

            val botPose = llResult.getBotpose_MT2()
            telemetry.addData(
                "Tx",
                tx
            ) // Represents how far left/right the target is (in degrees)
            telemetry.addData(
                "Ty",
                ty
            ) // Represents how far up/down the target is (in degrees)
            telemetry.addData("Ta", ta) // Represents how big the AprilTag looks

            // according to the camera field of view (0-100%)
            telemetry.addData("BotPose", botPose.toString())
            telemetry.addData("Yaw", botPose.getOrientation().getYaw())

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
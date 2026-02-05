package org.firstinspires.ftc.teamcode.vision

import Angle
import Distance
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
import org.firstinspires.ftc.teamcode.vision.VisionConstants.AprilTagsPhysicalDescription
import org.firstinspires.ftc.teamcode.vision.VisionConstants.LimelightPhysicalDescription
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan


class Limelight(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    var otos: Otos
) : SubsystemBase() {

    private var limelight: Limelight3A? = null
    var llResult: LLResult? = null

    private var obeliskId = 0

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
        // Reset's the otos readings to set the front of the robot when
        otos.resetTracking()

        // Starts the limelight's readings
        limelight!!.start()
    }

    fun start() {
        limelight!!.start()
    }

    fun getTx(): Double = tx
    fun getTy(): Double = ty
    fun getTa(): Double = ta

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

    fun getCameraMT2Position(): Pose3D? {
        if (llResult != null && llResult!!.isValid()) {
            val botPoseMT2: Pose3D? = llResult!!.botpose
            if (botPoseMT2 != null) {
                val x = botPoseMT2.getPosition().x
                val y = botPoseMT2.getPosition().y
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")")
                return botPoseMT2
            }
        }

        return null
    }

    fun getRobotPoseFromMegaTag2(turretAngle: Angle): SparkFunOTOS.Pose2D? {
        val cameraPose = getCameraMT2Position()

        if (cameraPose != null) {
            // 1. OFFSET FÍSICO ROBOT → CÁMARA (cuando torreta = 0)

            val dx = LimelightPhysicalDescription.LLXOffsetToLens.meters // metros hacia adelante
            val dy = LimelightPhysicalDescription.LLYOffsetToLens.meters // metros hacia la izquierda
            val dz = 0.0

            // 2. Rotar offset por la torreta
            val xOff = dx * cos(turretAngle.radians) - dy * sin(turretAngle.radians)
            val yOff = dx * sin(turretAngle.radians) + dy * cos(turretAngle.radians)

            // 3. Extraer pose de la cámara
            val camPos: Position = cameraPose.getPosition()
            val camAngles = cameraPose.getOrientation()

            val xCam: Double = camPos.x
            val yCam: Double = camPos.y
            val zCam: Double = camPos.z

            val yawCam = camAngles.getYaw(AngleUnit.RADIANS)

            // 4. Heading REAL del robot
            val yawRobot = yawCam - turretAngle.radians

            // 5. Restar offset en el frame del campo
            val xRobot = xCam - (xOff * cos(yawRobot) - yOff * sin(yawRobot))
            val yRobot = yCam - (xOff * sin(yawRobot) + yOff * cos(yawRobot))
            val zRobot = zCam - dz

            val time = System.nanoTime()

            // 6. Construir Pose3D del robot
            /* val robotPosition = Position(
                DistanceUnit.METER,
                xRobot,
                yRobot,
                zRobot,
                time
            )

            val robotOrientation = YawPitchRollAngles(
                AngleUnit.RADIANS,
                yawRobot,  // yaw
                0.0,  // pitch (robot plano)
                0.0,  // roll
                time
            ) */

            val xCoordinate = Distance.fromMeters(xRobot)
            val yCoordinate = Distance.fromMeters(yRobot)
            val heading = Angle.fromRadians(yawRobot)

            telemetry.addData("Bot Location:", "(" + xCoordinate.inches + ", " + yCoordinate.inches + ")")

            return SparkFunOTOS.Pose2D(xCoordinate.inches, yCoordinate.inches, heading.radians)
        }

        return null
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

    fun getRobotHeadingDegrees(): Double {
        return otos.getHeading().degrees
    }

    override fun periodic() {

        // Updating limelights' robot orientation with the Yaw
        limelight!!.updateRobotOrientation(getRobotHeadingDegrees())

        // LLResult is like a container full of information about what Limelight sees
        llResult = limelight!!.getLatestResult()

        val fiducialResult = llResult!!.fiducialResults

        // Math to calculate distance was taken from documentation:
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance#using-area-to-estimate-distance
        // The condition verifies whether the LimeLight Result is a valid statement
        if (llResult != null && llResult!!.isValid()) {

            // Obelisk ID detection
            outerLoop@ for (detectedId in fiducialResult) {
                for (aprilTagId in VisionConstants.AprilTagsIdentification.ObeliskIds) {
                    if (detectedId.fiducialId == aprilTagId && getMotifPattern() == MotifPatterns.NO_PATTERN_DETECTED) {
                        obeliskId = detectedId.fiducialId
                        break@outerLoop
                        break
                    }
                }
            }

            val targetOffsetAngle_Vertical = Angle.fromDegrees(llResult!!.getTy())
            // Needs to be in radians for tan() method
            val angleToGoalRadians =
                Math.toRadians(LimelightPhysicalDescription.LLMountAngleFromHorizontal.degrees + targetOffsetAngle_Vertical.degrees)

            // Calculated distance from limelight lens to goal (in inches)
            distanceFromLimelightToGoalInches =
                (AprilTagsPhysicalDescription.GoalHeightFromGround - LimelightPhysicalDescription.LLHeightFromGroundToLens) / tan(
                    angleToGoalRadians
                )

            telemetry.addData(
                "Limelight TargetDistanceInches",
                distanceFromLimelightToGoalInches.inches
            )

            // We will first get a (MetaTag2) Pose3D. From here, we will extract its Tx, Ty & Ta components
            tx = llResult!!.tx
            ty = llResult!!.ty
            ta = llResult!!.ta

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
     */
}
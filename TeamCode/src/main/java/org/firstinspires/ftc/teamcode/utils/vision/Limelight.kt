package org.firstinspires.ftc.teamcode.utils.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.tan

class Limelight(hardwareMap: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    private var limelight: Limelight3A? = null
    private var imu: IMU? = null // Setting the IMU is necessary to get MetaTag2,

    // which allows for better data retrieving using the IMU
    private var otos: SparkFunOTOS? = null

    private var limelightMountAngleDegrees = 0.0 // LL mount angle from horizontal
    private var limelightLensHeightInches = 0.0 // LL lens Height from ground
    private var goalHeightInches = 0.0 // Target height from ground

    private var ty = 0.0
    private var tx = 0.0
    private var ta = 0.0

    init {
        limelight = hardwareMap.get<Limelight3A?>(
            Limelight3A::class.java,
            "limelight"
        ) // Retrieves pipeline
        limelight!!.pipelineSwitch(1) // Gets the limelight pipeline

        // TODO: Get these values
        limelightMountAngleDegrees = 0.0
        limelightLensHeightInches = 0.0
        goalHeightInches = 0.0

        imu = hardwareMap.get<IMU?>(IMU::class.java, "imu")
        imu!!.resetYaw()

        otos = hardwareMap.get<SparkFunOTOS?>(SparkFunOTOS::class.java, "otos")
        otos!!.setAngularUnit(AngleUnit.DEGREES)
        otos!!.setLinearUnit(DistanceUnit.INCH)
        otos!!.resetTracking()
    }

    fun start() {
        // We start the limelight specifically at this point so that it doesn't take any energy
        // before the start button is pressed in match
        limelight!!.start()
    }

    fun getTx(): Double = tx
    fun getTy(): Double = ty
    fun getTa(): Double = ta

    override fun periodic() {
        // Getting the robot's orientation through the IMU
        val orientation = imu!!.getRobotYawPitchRollAngles()
        telemetry.addData("orientationIMU", orientation.getYaw())
        telemetry.addData("orientationOTOS", otos!!.getPosition().h)

        // Updating limelights' robot orientation with the Yaw
        limelight!!.updateRobotOrientation(otos!!.getPosition().h)

        // LLResult is like a container full of information about what Limelight sees
        val llResult = limelight!!.getLatestResult()

        // Math to calculate distance was taken from documentation:
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance#using-area-to-estimate-distance
        // The condition verifies whether the LimeLight Result is a valid statement
        if (llResult != null && llResult.isValid()) {
            // Offset to target in degrees (from crosshair)
            val targetOffsetAngle_Vertical = llResult.getTy()
            // Needs to be in radians for tan() method
            val angleToGoalRadians =
                Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical)

            // Calculated distance from limelight lens to goal (in inches)
            val distanceFromLimelightToGoalInches =
                (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians)
            telemetry.addData("TargetDistanceInches", distanceFromLimelightToGoalInches)

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
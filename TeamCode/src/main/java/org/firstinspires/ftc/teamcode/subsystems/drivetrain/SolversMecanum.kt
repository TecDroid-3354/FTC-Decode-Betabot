package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.drivebase.MecanumDrive
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit


class SolversMecanum(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry
) : SubsystemBase() {

    // Declaring motors
    lateinit var frontRightMotor: Motor
    lateinit var frontLeftMotor: Motor
    lateinit var backRightMotor: Motor
    lateinit var backLeftMotor: Motor

    // Declaring mecanum from solverslib
    var mecanum: MecanumDrive

    // Declaring useful components
    lateinit var imu: IMU
    lateinit var otos: SparkFunOTOS
    lateinit var revHubOrientation: RevHubOrientationOnRobot

    // Initialization code //
    init {
        motorsConfig()
        componentConfig()

        // Setting up the mecanum using the previously declared motors
        mecanum = MecanumDrive(
            frontLeftMotor, frontRightMotor,
            backLeftMotor, backRightMotor
        )
    }


    override fun periodic() {
        // Telemetry to retrieve useful data
        telemetry.addData("RobotYaw", getRobotYaw(AngleUnit.DEGREES))
        telemetry.addData("X", otos.position.x)
        telemetry.addData("Y", otos.position.y)
        telemetry.addData("Heading", otos.position.h)
    }

    // Functional code //

    // Robot-oriented chassis speeds
    // Only takes one parameter: the chassis speeds to be used
    private fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        mecanum.driveRobotCentric(
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond)
    }

    // Field-oriented chassis speeds
    // Takes two parameters: chassis speeds and gyro angle in degrees, with the last one taken from the IMU
    fun setChassisSpeedsFromFieldOriented(chassisSpeeds: ChassisSpeeds, gyroAngleInDegrees: Double) {
        mecanum.driveFieldCentric(
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond,
            gyroAngleInDegrees)
    }

    fun getRobotYaw(angleUnit: AngleUnit): Double = imu.robotYawPitchRollAngles.getYaw(angleUnit)
    fun resetRobotYaw(): Unit = imu.resetYaw()

    // Setup code //
    private fun motorsConfig() {
        // Configuring motors according to their revolutions per minute
        frontRightMotor = Motor(hardwareMap, MecanumConstants.Ids.frontRightId, GoBILDA.RPM_312)
        frontLeftMotor = Motor(hardwareMap, MecanumConstants.Ids.frontLeftId, GoBILDA.RPM_312)
        backRightMotor = Motor(hardwareMap, MecanumConstants.Ids.backRightId, GoBILDA.RPM_312)
        backLeftMotor = Motor(hardwareMap, MecanumConstants.Ids.backLeftId, GoBILDA.RPM_312)
    }

    private fun componentConfig() {
        // REV Hub IMU declaration
        otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        imu = hardwareMap.get(IMU::class.java, "imu")
        imu.resetYaw()

        revHubOrientation = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )

        imu.initialize(IMU.Parameters(revHubOrientation))
    }
}
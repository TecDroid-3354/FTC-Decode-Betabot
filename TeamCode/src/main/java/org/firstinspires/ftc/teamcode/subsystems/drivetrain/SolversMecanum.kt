package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.drivebase.MecanumDrive
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumConstants.Ids
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumConstants.Physics


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
    //lateinit var imu: IMU
    lateinit var otos: SparkFunOTOS
    //lateinit var revHubOrientation: RevHubOrientationOnRobot

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
        telemetry.addData("RobotYaw", getRobotYaw())
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
    fun setChassisSpeedsFromFieldOriented(chassisSpeeds: ChassisSpeeds) {
        mecanum.driveFieldCentric(
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond,
            getRobotYaw())
    }

    fun getRobotYaw(): Double = otos.position.h
    fun resetOtosYaw(): Unit = otos.resetTracking()
//    fun getRobotYaw(angleUnit: AngleUnit): Double = imu.robotYawPitchRollAngles.getYaw(angleUnit)
//    fun resetRobotYaw(): Unit = imu.resetYaw()

    // Setup code //
    private fun motorsConfig() {
        // Configuring motors according to their revolutions per minute
        frontRightMotor = Motor(hardwareMap, Ids.frontRightId, Physics.countPerRevolution, Physics.maxRPM)
        frontLeftMotor = Motor(hardwareMap, Ids.frontLeftId, Physics.countPerRevolution, Physics.maxRPM)
        backRightMotor = Motor(hardwareMap, Ids.backRightId, Physics.countPerRevolution, Physics.maxRPM)
        backLeftMotor = Motor(hardwareMap, Ids.backLeftId, Physics.countPerRevolution, Physics.maxRPM)

        frontRightMotor.setRunMode(Motor.RunMode.RawPower)
        frontLeftMotor.setRunMode(Motor.RunMode.RawPower)
        backRightMotor.setRunMode(Motor.RunMode.RawPower)
        backLeftMotor.setRunMode(Motor.RunMode.RawPower)

        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

    }

    private fun componentConfig() {
        // REV Hub IMU declaration
        otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        //imu = hardwareMap.get(IMU::class.java, "imu")
        //imu.resetYaw()

//        revHubOrientation = RevHubOrientationOnRobot(
//            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//        )

        //imu.initialize(IMU.Parameters(revHubOrientation))
    }
}
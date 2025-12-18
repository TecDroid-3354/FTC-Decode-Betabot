package org.firstinspires.ftc.teamcode.utils.gyroscopes

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit


data class RevHubIMUConfig(
    val imuName: String,
    val revHubLogoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection,
    val revHubUSBFacingDirection: RevHubOrientationOnRobot.UsbFacingDirection
)


@Suppress("JoinDeclarationAndAssignment")
class RevHubIMU(hw: HardwareMap, config: RevHubIMUConfig) {

    private var imu: IMU

    init {
        imu = hw.get(IMU::class.java, config.imuName)

        val revHubOrientationOnRobot = RevHubOrientationOnRobot(
            config.revHubLogoFacingDirection,
            config.revHubUSBFacingDirection
        )

        imu.initialize(IMU.Parameters(revHubOrientationOnRobot))
        imu.resetYaw()
    }

    fun getYaw(): Double {
        return imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
    }

    fun getIMU(): IMU {
        return imu
    }

}
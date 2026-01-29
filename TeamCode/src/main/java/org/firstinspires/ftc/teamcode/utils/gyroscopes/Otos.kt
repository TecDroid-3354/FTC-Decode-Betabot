package org.firstinspires.ftc.teamcode.utils.gyroscopes

import Angle
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Vector2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D

data class OtosConfig(
    val otosId: String,
    val angleUnit: AngleUnit,
    val linearUnit: DistanceUnit
)

class Otos(hardwareMap: HardwareMap, val telemetry: Telemetry, val config: OtosConfig) {

    val otos: SparkFunOTOS = hardwareMap.get(SparkFunOTOS::class.java, config.otosId)

    init {
        sensorConfiguration()
    }

    fun setOffset(offset: Pose2D) {
        otos.offset = offset
    }

    fun resetTracking() {
        otos.resetTracking()
    }

    fun getSensorInstance(): SparkFunOTOS = otos
    fun getPositionVector(): Vector2d = Vector2d(otos.position.x, otos.position.y)
    fun getHeading(): Angle = when (config.angleUnit) {
            AngleUnit.DEGREES -> Angle.fromDegrees(otos.position.h)
            AngleUnit.RADIANS -> Angle.fromRadians(otos.position.h)
    }

    fun log() {
        telemetry.addData("X position", otos.position.x)
        telemetry.addData("Y position", otos.position.y)
        telemetry.addData("Heading", otos.position.h)
    }

    fun sensorConfiguration() {
        otos.resetTracking()
        otos.calibrateImu()
        otos.angularUnit = config.angleUnit
        otos.linearUnit = config.linearUnit
    }
}
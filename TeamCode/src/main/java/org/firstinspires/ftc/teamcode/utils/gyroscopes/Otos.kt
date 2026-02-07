package org.firstinspires.ftc.teamcode.utils.gyroscopes

import Angle
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Vector2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

data class OtosConfig(
    val otosId: String,
    val angleUnit: AngleUnit,
    val linearUnit: DistanceUnit
)

class Otos(hardwareMap: HardwareMap, val telemetry: Telemetry, val config: OtosConfig) {

    val otos: SparkFunOTOS = hardwareMap.get(SparkFunOTOS::class.java, config.otosId)

    fun setOffset(offset: Pose2D) {
        otos.setOffset(offset)
    }

    fun setPosition(pose: Pose2D) {
        otos.position = pose
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
        telemetry.addLine("// OTOS //")
        telemetry.addData("X position", otos.position.x)
        telemetry.addData("Y position", otos.position.y)
        telemetry.addData("Heading", otos.position.h)
    }

    fun sensorConfiguration() {
        otos.angularUnit = config.angleUnit
        otos.linearUnit = config.linearUnit

        WaitCommand(50)
        otos.calibrateImu()
        setOffset(Pose2D(0.0, -0.70637, -90.0))
        otos.resetTracking()
        WaitCommand(50)
    }
}
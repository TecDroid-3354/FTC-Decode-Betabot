package org.firstinspires.ftc.teamcode.subsystems.indexer.Slot

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.hardware.ServoEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx

data class SlotConfig(
    val servoName: String,
    val colorSensorName: String
)

class Slot (config: SlotConfig, hw: HardwareMap, telemetry: Telemetry) {
    private lateinit var servo: ServoEx
    private lateinit var colorSensor: ColorSensorEx

    init {
        servo = ServoEx(hw, config.servoName);
        colorSensor = ColorSensorEx(hw.get(
            ColorSensor::class.java,
            config.colorSensorName),
            telemetry)

    }

    fun getDetectedColor(): ColorSensorEx.DetectedColor = colorSensor.getColorFromSensor()

    private fun setServoPosition(position: Double) {
        servo.set(position)
    }

    fun feed() {
        setServoPosition(IndexerConstants.Positions.feedPosition)
    }

    fun home() {
        setServoPosition(IndexerConstants.Positions.homePosition)
    }
}
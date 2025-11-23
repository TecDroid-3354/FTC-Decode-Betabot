package org.firstinspires.ftc.teamcode.subsystems.indexer.Slot

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.hardware.ServoEx
import kotlinx.coroutines.delay
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx

data class SlotConfig(
    val servoName: String,
    val colorSensorName: String,
    val slotId: String
)

class Slot (val config: SlotConfig, hw: HardwareMap, telemetry: Telemetry) {
    private lateinit var servo: ServoEx
    public lateinit var colorSensor: ColorSensorEx

    init {
        servo = ServoEx(hw, config.servoName)
        colorSensor = ColorSensorEx(hw.get(
            ColorSensor::class.java,
            config.colorSensorName),
            telemetry)

        SequentialCommandGroup(
            InstantCommand({ home() }),
            WaitCommand(500),
            InstantCommand({ awake() })
        ).schedule()
    }

    fun getDetectedColor(): ColorSensorEx.DetectedColor = colorSensor.getColorFromSensor()

    fun getHSV() = colorSensor.getHSV()

    private fun setServoPosition(position: Double) {
        servo.set(position)
    }

    fun feed() {
        setServoPosition(IndexerConstants.Positions.feedPosition)
    }

    fun home() {
        setServoPosition(IndexerConstants.Positions.homePosition)
    }

    fun awake() {
        setServoPosition(IndexerConstants.Positions.awakePosition)
    }
}
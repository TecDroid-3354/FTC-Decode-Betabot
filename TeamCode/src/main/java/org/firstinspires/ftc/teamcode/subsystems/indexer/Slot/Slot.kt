package org.firstinspires.ftc.teamcode.subsystems.indexer.Slot

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder
import com.seattlesolvers.solverslib.hardware.ServoEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx.DetectedColor

data class SlotConfig(
    val servoName: String,
    val isInverted: Boolean,
    val feedPosition: Double,
    val homePosition: Double,
    val absoluteId: String,
    val rightColorSensorId: String,
    val leftColorSensorId: String,
    val archiveExtension: String
)

@Suppress("JoinDeclarationAndAssignment")
class Slot (val config: SlotConfig, hw: HardwareMap, telemetry: Telemetry) {
    private var servo: ServoEx
    private var absEncoder: AbsoluteAnalogEncoder
    private var rightColorSensor: ColorSensorEx
    private var leftColorSensor: ColorSensorEx


    init {
        servo = ServoEx(hw, config.servoName)
        servo.inverted = config.isInverted
        absEncoder = AbsoluteAnalogEncoder(hw, config.absoluteId, 90.0, AngleUnit.RADIANS)

        rightColorSensor = ColorSensorEx(hw.get(
            ColorSensor::class.java,
            config.rightColorSensorId),
            telemetry)

        leftColorSensor = ColorSensorEx(hw.get(
            ColorSensor::class.java,
            config.leftColorSensorId),
            telemetry)

        SequentialCommandGroup(
            InstantCommand({ home() }),
            WaitCommand(500),
            InstantCommand({ awake() })
        ).schedule()
    }

    fun getAbsoluteReading(): Double = absEncoder.currentPosition

    fun getPosition(): Double {
        return servo.rawPosition
    }

    fun getDetectedColor(): DetectedColor {
        // Ensure both readings are equal to correctly detect the ball inside the slot

        //return Pair(rightColorSensor.colorFromSensor, leftColorSensor.colorFromSensor)
        return if (rightColorSensor.colorFromSensor == leftColorSensor.colorFromSensor) {
            // Now that we know they are the same, we can grab either the right or left reading
            // In this case we grabbed the right one
            rightColorSensor.colorFromSensor
        } else {
            DetectedColor.UNKNOWN
        }
    }

    private fun setServoPosition(position: Double) {
        servo.set(position)
    }

    fun feed() {
        setServoPosition(config.feedPosition)
    }

    fun home() {
        setServoPosition(config.homePosition)
    }

    fun awake() {
        setServoPosition(config.homePosition - 0.001)
    }
}
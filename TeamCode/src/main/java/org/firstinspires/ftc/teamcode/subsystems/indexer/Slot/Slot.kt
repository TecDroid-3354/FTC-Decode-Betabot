package org.firstinspires.ftc.teamcode.subsystems.indexer.Slot

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.HardwareMap
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

    // Declare the slot components
    private var servo: ServoEx
    private var absEncoder: AbsoluteAnalogEncoder
    var rightColorSensor: ColorSensorEx
    var leftColorSensor: ColorSensorEx

    // Initialization code //
    init {
        // Initialize the servo
        servo = ServoEx(hw, config.servoName)
        servo.inverted = config.isInverted
        // Initialize the absolute encoder
        absEncoder = AbsoluteAnalogEncoder(hw, config.absoluteId, 90.0, AngleUnit.RADIANS)

        // Initialize both coclor sensors
        rightColorSensor = ColorSensorEx(hw.get(
            ColorSensor::class.java,
            config.rightColorSensorId),
            telemetry,
            config.archiveExtension)

        leftColorSensor = ColorSensorEx(hw.get(
            ColorSensor::class.java,
            config.leftColorSensorId),
            telemetry,
            config.archiveExtension)

        awakeServo()
    }

    /**
     * @return the reading of the absolute encoder in [AngleUnit.RADIANS]
     */
    fun getAbsoluteReading(): Double = absEncoder.currentPosition

    /**
     * Compares the reading of both color sensors and if they are the same, it returns the [DetectedColor]
     * @return the [DetectedColor] of the [Slot]
     */
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

    /**
     * Sets a raw servo position based on the range it physically has
     */
    private fun setServoPosition(position: Double) {
        servo.set(position)
    }

    /**
     * Moves the servo to the feed position
     */
    fun feed() {
        setServoPosition(config.feedPosition)
    }

    /**
     * Moves the servo to the home position
     */
    fun home() {
        setServoPosition(config.homePosition)
    }

    /**
     * Executes a [SequentialCommandGroup] that literally awakens the servo for it to be ready for feeding
     * the shooter. This needs to be called in the initialization code so the servo works correctly
     */
    fun awakeServo() {
        SequentialCommandGroup(
            InstantCommand({ home() }),
            WaitCommand(500),
            InstantCommand({ setServoPosition(config.homePosition + 0.001)})
        ).schedule()
    }
}
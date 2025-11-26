package org.firstinspires.ftc.teamcode.subsystems.indexer

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.command.WaitCommand
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.Slot.Slot
import org.firstinspires.ftc.teamcode.subsystems.indexer.Slot.SlotConfig
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Ids
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx.DetectedColor

enum class MotifPatterns(val pattern: List<DetectedColor>) {
    PURPLE_PURPLE_GREEN(listOf(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN)),
    PURPLE_GREEN_PURPLE(listOf(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE)),
    GREEN_PURPLE_PURPLE(listOf(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE))
}

class Indexer(hw: HardwareMap, telemetry: Telemetry) : SubsystemBase() {
    private lateinit var frontSlot: Slot
    private lateinit var leftSlot: Slot
    private lateinit var rightSlot: Slot
    private lateinit var slotList: List<Slot>

    init {
        frontSlot = Slot(
            SlotConfig(Ids.servoFront, Ids.colorSensorFront),
            hw,
            telemetry)

        leftSlot = Slot(
            SlotConfig(Ids.servoLeft, Ids.colorSensorLeft),
            hw,
            telemetry)

        rightSlot = Slot(
            SlotConfig(Ids.servoRight, Ids.colorSensorRight),
            hw,
            telemetry)

        slotList = listOf(frontSlot, leftSlot, rightSlot)

    }

    fun feedShooter(): SequentialCommandGroup {
        return SequentialCommandGroup(
            feedCMD(),
            feedCMD(),
            feedCMD(),
        )
    }

    fun feedShooter(motifPatterns: MotifPatterns): SequentialCommandGroup {
        return SequentialCommandGroup(
            feedCMD(motifPatterns.pattern[0]),
            feedCMD(motifPatterns.pattern[1]),
            feedCMD(motifPatterns.pattern[2]),
        )
    }

    private fun feedCMD(color: DetectedColor): Command {
        for (slot in slotList) {
            if (slot.getDetectedColor() == color) {
                return SequentialCommandGroup(
                    InstantCommand({ slot.feed() }),
                    WaitCommand(1000),
                    InstantCommand({ slot.home() }),
                    WaitCommand(1000)
                )
            }
        }

        return InstantCommand()
    }

    private fun feedCMD(): Command {
        for (slot in slotList) {
            if (slot.getDetectedColor() != DetectedColor.UNKNOWN) {
                return SequentialCommandGroup(
                    InstantCommand({ slot.feed() }),
                    WaitCommand(1000),
                    InstantCommand({ slot.home() }),
                    WaitCommand(1000)
                )
            }
        }

        return InstantCommand()
    }

}
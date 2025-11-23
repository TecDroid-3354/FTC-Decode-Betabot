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
import kotlin.collections.contains

enum class MotifPatterns(val pattern: List<DetectedColor>) {
    PURPLE_PURPLE_GREEN(listOf(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN)),
    PURPLE_GREEN_PURPLE(listOf(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE)),
    GREEN_PURPLE_PURPLE(listOf(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE))
}

class Indexer(val hw: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    private lateinit var frontSlot: Slot
    private lateinit var leftSlot: Slot
    private lateinit var rightSlot: Slot
    private lateinit var slotList: Array<Slot>

    init {
        frontSlot = Slot(
            SlotConfig(Ids.frontServo, Ids.colorSensorFront, "front"),
            hw,
            telemetry)

        rightSlot = Slot(
            SlotConfig(Ids.rightServo, Ids.colorSensorRight, "right"),
            hw,
            telemetry)

        leftSlot = Slot(
            SlotConfig(Ids.leftServo, Ids.colorSensorLeft, "left"),
            hw,
            telemetry)

        slotList = arrayOf(frontSlot, rightSlot, leftSlot)
    }

    fun rejectEvaluation(): Boolean {
        var greenIndex = 0
        var purpleIndex = 0
        for (slot in slotList) {
            if (slot.getDetectedColor() == DetectedColor.GREEN) {
                greenIndex++
            }
            if (slot.getDetectedColor() == DetectedColor.PURPLE) {
                purpleIndex++
            }
        }

        return greenIndex > 1 || purpleIndex > 2
    }

    fun feedShooter(): SequentialCommandGroup {
        var slotOrder = arrayOf("", "", "")
        val cmdGroup = SequentialCommandGroup()

        for ((index, slot) in slotList.withIndex()) {
            if (slot.getDetectedColor() != DetectedColor.UNKNOWN) {
                slotOrder.fill(slot.config.slotId, index)
                cmdGroup.addCommands(feedCMD(slotOrder[index]))
            }
        }

        return cmdGroup
    }

    fun feedAllShooter(): SequentialCommandGroup {
        return SequentialCommandGroup(
            feedCMD(slotList[0]),
            feedCMD(slotList[1]),
            feedCMD(slotList[2]))
    }

    fun feedShooter(motifPatterns: MotifPatterns): SequentialCommandGroup {
        var slotOrder = arrayOf("", "", "")
        val cmdGroup = SequentialCommandGroup()

        if (rejectEvaluation()) {
            return feedShooter()
        }

        for ((index, color) in motifPatterns.pattern.withIndex()) {
            for (slot in slotList) {
                if (slot.getDetectedColor() == color && !slotOrder.contains(slot.config.slotId)) {
                    slotOrder.fill(slot.config.slotId, index)
                    cmdGroup.addCommands(feedCMD(slotOrder[index]))
                    break
                }
            }
        }

        return cmdGroup
    }

    private fun feedCMD(slotId: String): Command {
        val slot: Slot? = when(slotId) {
            frontSlot.config.slotId -> frontSlot
            leftSlot.config.slotId -> leftSlot
            rightSlot.config.slotId -> rightSlot
            else -> null
        }

        return if (slot != null) {
            SequentialCommandGroup(
                InstantCommand({ slot.feed() }),
                WaitCommand(1000),
                InstantCommand({ slot.home() }),
                WaitCommand(1000)
            )
        } else {
            InstantCommand()
        }
    }

    private fun feedCMD(slot: Slot): Command {
            return SequentialCommandGroup(
                InstantCommand({ slot.feed() }),
                WaitCommand(1000),
                InstantCommand({ slot.home() }),
                WaitCommand(1000))
    }

    override fun periodic() {
        telemetry.addData("Slot1", slotList[0].getDetectedColor())
        telemetry.addData("Slot2", slotList[1].getDetectedColor())
        telemetry.addData("Slot3", slotList[2].getDetectedColor())
    }

}
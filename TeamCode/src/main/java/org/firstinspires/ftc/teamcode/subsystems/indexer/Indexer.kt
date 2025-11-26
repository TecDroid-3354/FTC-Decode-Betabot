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
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Positions
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Extensions
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx.DetectedColor

enum class MotifPatterns(val pattern: List<DetectedColor>) {
    PURPLE_PURPLE_GREEN(listOf(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN)),
    PURPLE_GREEN_PURPLE(listOf(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE)),
    GREEN_PURPLE_PURPLE(listOf(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE))
}

@Suppress("JoinDeclarationAndAssignment")
class Indexer(val hw: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    var frontSlot: Slot
    var backSlot: Slot
    var middleSlot: Slot
    private var slotList: Array<Slot>

    init {
        frontSlot = Slot(
            SlotConfig(Ids.frontServo, true, Positions.FrontPositions.FEED,
                Positions.FrontPositions.HOME, Ids.absFront, Ids.frontSlotRightSensor, Ids.frontSlotLeftSensor,
                Extensions.frontSlotExtension),
            hw,
            telemetry)

        middleSlot = Slot(
            SlotConfig(Ids.rightServo, false, Positions.MiddlePositions.FEED,
                Positions.MiddlePositions.HOME ,Ids.absRight, Ids.middleSlotRightSensor, Ids.middleSlotLeftSensor,
                Extensions.middleSlotExtension),
            hw,
            telemetry)

        backSlot = Slot(
            SlotConfig(Ids.leftServo, false, Positions.BackPositions.FEED,
                Positions.BackPositions.HOME ,Ids.absLeft ,Ids.backSlotRightSensor, Ids.backSlotLeftSensor,
                Extensions.backSlotExtension),
            hw,
            telemetry)

        slotList = arrayOf(frontSlot, middleSlot, backSlot)
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
                slotOrder.fill(slot.config.archiveExtension, index)
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
                if (slot.getDetectedColor() == color && !slotOrder.contains(slot.config.archiveExtension)) {
                    slotOrder.fill(slot.config.archiveExtension, index)
                    cmdGroup.addCommands(feedCMD(slotOrder[index]))
                    break
                }
            }
        }

        return cmdGroup
    }

    fun feedCMD(slotId: String): Command {
        val slot: Slot? = when(slotId) {
            frontSlot.config.archiveExtension -> frontSlot
            backSlot.config.archiveExtension -> backSlot
            middleSlot.config.archiveExtension -> middleSlot
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
        telemetry.addData("FrontSlot", slotList[0].getDetectedColor())
        telemetry.addData("MiddleSlot", slotList[1].getDetectedColor())
        telemetry.addData("BackSlot", slotList[2].getDetectedColor())
    }
}
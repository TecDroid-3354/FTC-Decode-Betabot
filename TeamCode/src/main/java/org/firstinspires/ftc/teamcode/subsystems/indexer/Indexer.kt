package org.firstinspires.ftc.teamcode.subsystems.indexer

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Positions
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.Slot.Slot
import org.firstinspires.ftc.teamcode.subsystems.indexer.Slot.SlotConfig
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Identification
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Configuration
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Extensions
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx.DetectedColor

enum class MotifPatterns(val pattern: List<DetectedColor>) {
    PURPLE_PURPLE_GREEN(listOf(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN)),
    PURPLE_GREEN_PURPLE(listOf(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE)),
    GREEN_PURPLE_PURPLE(listOf(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE)),
    NO_PATTERN_DETECTED(listOf(DetectedColor.UNKNOWN, DetectedColor.UNKNOWN, DetectedColor.UNKNOWN))
}

@Suppress("JoinDeclarationAndAssignment")
class Indexer(
    hw: HardwareMap,
    val telemetry: Telemetry
): SubsystemBase() {

    // Declaration of our slots //
    val frontSlot: Slot
    val backSlot: Slot
    val middleSlot: Slot
    var slotList: Array<Slot>

    // Initialization code //
    init {

        // Giving each slot its corresponding servo, absolute, color sensors and positions
        frontSlot = Slot(
            SlotConfig(Identification.FrontSlot.frontServoId, Configuration.isFrontServoInverted,
                Identification.FrontSlot.frontSlotRightSensor,
                Identification.FrontSlot.frontSlotLeftSensor,
                Positions.FrontBackPositions.FEED,
                Positions.FrontBackPositions.HOME,
                Extensions.frontSlotExtension),
            hw,
            telemetry)

        middleSlot = Slot(
            SlotConfig(Identification.MiddleSlot.middleServoId, Configuration.isMiddleServoInverted,
                Identification.MiddleSlot.middleSlotRightSensor,
                Identification.MiddleSlot.middleSlotLeftSensor,
                Positions.MiddlePositions.FEED,
                Positions.MiddlePositions.HOME,
                Extensions.middleSlotExtension),
            hw,
            telemetry)

        backSlot = Slot(
            SlotConfig(Identification.BackSlot.backServoId, Configuration.isBackServoInverted,
                Identification.BackSlot.backSlotRightSensor,
                Identification.BackSlot.backSlotLeftSensor,
                Positions.FrontBackPositions.FEED,
                Positions.FrontBackPositions.HOME,
                Extensions.backSlotExtension),
            hw,
            telemetry)

        slotList = arrayOf(frontSlot, middleSlot, backSlot)
    }

    // This code will execute indefinably during your operation
    override fun periodic() {
//        telemetry.addData("frontSlotColor", frontSlot.getDetectedColor())
//        telemetry.addData("middleSlotColor", middleSlot.getDetectedColor())
//        telemetry.addData("backSlotColor", backSlot.getDetectedColor())
    }

    /**
     * [rejectEvaluation] tracks if the ball configuration inside our indexer is valid for
     * launching with a defined order, for that it updates indexes for both colors and if green balls are
     * more than 1, it will reject the current ball configuration. Same if there are more than 2 purple balls.
     * @return True if there is more than one green ball or two purple balls
     */
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

        return greenIndex != 1 || purpleIndex != 2
    }

    /**
     * @return true if every single slot in our indexer currently has a ball
     */
    fun isFull(): Boolean {
        for (slot in slotList) {
            if (slot.getDetectedColor() == DetectedColor.UNKNOWN) {
                return false
            }
        }
        return true
    }

    /**
     * [feedShooter] returns a [SequentialCommandGroup] that feeds each slot if the color the color sensors detection
     * is not [DetectedColor.UNKNOWN]
     * @return a [SequentialCommandGroup] that feeds every slot that has a ball
     */
    fun feedShooter(): SequentialCommandGroup {
        val cmdGroup = SequentialCommandGroup()
        var slotTracker: MutableList<String> = MutableList(3) { "" }

        for ((index, slot) in slotList.withIndex()) {
            if (slot.getDetectedColor() != DetectedColor.UNKNOWN && slot.config.archiveExtension !in slotTracker) {
                slotTracker.add(index, slot.config.archiveExtension)
                cmdGroup.addCommands(feedCMD(slot))
            }
        }

        return cmdGroup
    }

    /**
     * [feedShooter] receives a [MotifPatterns] and determines if the bal configuration inside the [Indexer]
     * is valid for completing the [MotifPatterns], and then sets the slot order if the ccolors inside each [Slot]
     * satisfy the [MotifPatterns]
     * @param motifPatterns The current Pattern, it must be received from Limelight readings
     * @return a [SequentialCommandGroup] that executes the feed sequence on each valid slot
     */
    fun feedShooter(motifPatterns: MotifPatterns): SequentialCommandGroup {
        val cmdGroup = SequentialCommandGroup()
        var slotTracker: MutableList<String> = MutableList(3) {""}

        if (rejectEvaluation() || motifPatterns == MotifPatterns.NO_PATTERN_DETECTED) {
            return feedShooter()
        }

        for ((index, color) in motifPatterns.pattern.withIndex()) {
            for (slot in slotList) {
                if (slot.getDetectedColor() == color && slot.config.archiveExtension !in slotTracker) {
                    slotTracker.add(index, slot.config.archiveExtension)
                    cmdGroup.addCommands(feedCMD(slot))
                    break
                }
            }
        }

        return cmdGroup
    }

    /**
     * [feedAllShooter] Returns a [SequentialCommandGroup] that executes a whole feed sequence involving
     * three slots no matter if they have no ball inside
     * @return a [SequentialCommandGroup] that executes [feedCMD] for each slot
     */
    fun feedAllShooter(): SequentialCommandGroup {
        return SequentialCommandGroup(
            feedCMD(slotList[0]),
            feedCMD(slotList[1]),
            feedCMD(slotList[2])
        )
    }

    /**
     * [feedAllShooterAuto] Executes a whole feed sequence involving
     * three slots no matter if they have no ball inside
     */
    fun feedAllShooterAuto() {
        feedAuto(slotList[0])
        feedAuto(slotList[1])
        feedAuto(slotList[2])
    }

    /**
     * [feedAuto] receives a [Slot] as an argument and rises the flicker
     * and lowers it after the ball was launched.
     * @param  slot , it must be initialized [Slot]
     */
    fun feedAuto(slot: Slot) {
        slot.feed()
        WaitCommand(100)
        slot.home()
        WaitCommand(300)
    }

// MAY NEED THIS FUNCTION LATER, NOW ITS NOT USEFUL //

//    /**
//     * [feedCMD] receives a string as an argument and returns a [SequentialCommandGroup] that executes if
//     * the received [Slot] is not null and executes normally the feed sequence.
//     * @param slotId the archive extension of a slot
//     * @return A [SequentialCommandGroup] that executes the feed sequence, if slot is null, it returns nothing
//     */
//    private fun feedCMD(slotId: String): Command {
//        val slot: Slot? = when(slotId) {
//            frontSlot.config.archiveExtension -> frontSlot
//            backSlot.config.archiveExtension -> backSlot
//            middleSlot.config.archiveExtension -> middleSlot
//            else -> null
//        }
//
//        return if (slot != null) {
//            SequentialCommandGroup(
//                InstantCommand({ slot.feed() }),
//                WaitCommand(500),
//                InstantCommand({ slot.home() }),
//                WaitCommand(500)
//            )
//        } else {
//            InstantCommand()
//        }
//    }

    /**
     * [feedCMD] receives a [Slot] as an argument and returns a [SequentialCommandGroup] that rises the flicker
     * and lowers it after the ball was launched.
     * @param  slot , it must be initialized [Slot]
     * @return a [SequentialCommandGroup] that executes the feed sequence
     */
    private fun feedCMD(slot: Slot): Command {
        return SequentialCommandGroup(
            WaitCommand(250),
            InstantCommand({ slot.feed() }),
            WaitCommand(100),
            InstantCommand({ slot.home() })
        )
    }
}
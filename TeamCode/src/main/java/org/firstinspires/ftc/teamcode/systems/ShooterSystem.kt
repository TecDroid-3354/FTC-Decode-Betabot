package org.firstinspires.ftc.teamcode.systems

import Angle
import Distance
import androidx.core.util.Supplier
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.subsystems.shooter.HoodConstants
import org.firstinspires.ftc.teamcode.systems.Point


/**
 * These values need to be measured physically
 */
val lInterpolationConfig = LInterpolationConfig(
    // The angle asked for is the hood's angle at that position, NOT RELATED TO THE APRILTAG

    // TODO: TUNE IN FIELD CALIBRATION; WORKS PERFECTLY WITH OUR MANCRAFT GOAL, BUT CAN'T ASSURE
    // TODO: IT DOES WITH THE ACTUAL GOAL
    // angle at 19.18in was 0.69 rotations
    // angle at 58.31in was 0.54 rotations
    firstCoordinate = Point(Distance.fromInches(58.31), Angle.fromRotations(0.54)),
    secondCoordinate = Point(Distance.fromInches(19.18), Angle.fromRotations(0.68))
)

@Suppress("JoinDeclarationAndAssignment")
class ShooterSystem(hw: HardwareMap, val telemetry: Telemetry, distanceToAprilTag: Supplier<Distance>, val isLLResultValid: Supplier<Boolean>) {

    val shooter: Shooter
    val indexer: Indexer
    val hood: Hood

    val interpolation = LinearInterpolationConstructor(lInterpolationConfig, distanceToAprilTag)

    init {
        shooter = Shooter(hw, telemetry)
        indexer = Indexer(hw, telemetry)
        hood = Hood(hw, telemetry)
    }

    fun getObtainedSetPointForHood(): Angle {
        return if (isLLResultValid.get()) {
            Angle.fromRotations(interpolation.getDesiredPoint())
        } else {
            return Angle.fromDegrees(HoodConstants.Positions.homePosition.degrees)
        }
    }

    fun shoot(motifPatterns: MotifPatterns) : Command {
        return SequentialCommandGroup(
            shooter.shootCMD(),
            InstantCommand({ hood.setHoodPosition(getObtainedSetPointForHood()) }),
            WaitCommand(1200),
            indexer.feedShooter(motifPatterns),
            InstantCommand({ shooter.stop() })
        )
    }
}
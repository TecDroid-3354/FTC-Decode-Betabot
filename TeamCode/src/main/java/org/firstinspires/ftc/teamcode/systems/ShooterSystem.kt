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
    firstCoordinate = Point(Distance.fromInches(22.18), Angle.fromDegrees(0.0)),
    secondCoordinate = Point(Distance.fromInches(0.0), Angle.fromDegrees(0.0))
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
            Angle.fromDegrees(interpolation.getDesiredPoint())
        } else {
            return Angle.fromDegrees(HoodConstants.Positions.homePosition.degrees)
        }
    }

    fun shoot(motifPatterns: MotifPatterns) : Command {
        return SequentialCommandGroup(
            shooter.shootCMD(),
            InstantCommand({ hood.setHoodPosition(getObtainedSetPointForHood()) }),
            WaitCommand(1000),
            indexer.feedShooter(motifPatterns),
            InstantCommand({ shooter.stop() })
        )
    }
}
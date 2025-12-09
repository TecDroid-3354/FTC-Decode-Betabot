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

/**
 * These values need to be measured physically
 */
val lInterpolationFarConfig = LInterpolationConfig(
    // The angle asked for is the hood's angle at that position, NOT RELATED TO THE APRILTAG

    // TODO: TUNE IN FIELD CALIBRATION; WORKS PERFECTLY WITH OUR MANCRAFT GOAL, BUT CAN'T ASSURE
    // TODO: IT DOES WITH THE ACTUAL GOAL
    // angle at 19.18in was 0.69 rotations
    // angle at 58.31in was 0.54 rotations
    firstCoordinate = Point(Distance.fromInches(54.82), Angle.fromRotations(0.67)),
    secondCoordinate = Point(Distance.fromInches(36.0), Angle.fromRotations(0.71))
)

@Suppress("JoinDeclarationAndAssignment")
class ShooterSystem(hw: HardwareMap, val telemetry: Telemetry, distanceToAprilTagInches: Supplier<Double>, val isLLResultValid: Supplier<Boolean>) {

    val indexer: Indexer
    private val distanceToAprilTag = {Distance.fromInches(distanceToAprilTagInches.get())}
    val shooter: Shooter
    val hood: Hood

    private val interpolator = LinearInterpolationConstructor(lInterpolationFarConfig, distanceToAprilTag)

    init {
        shooter = Shooter(hw, telemetry)
        indexer = Indexer(hw, telemetry)
        hood = Hood(hw, telemetry)
    }

    fun getObtainedSetPointForHood(): Angle {
        return if (isLLResultValid.get()) {
            Angle.fromRotations(interpolator.getDesiredPoint())
        } else {
            return Angle.fromDegrees(HoodConstants.Positions.homePosition.degrees)
        }
    }

    fun shoot(motifPatterns: MotifPatterns) : Command {
        return SequentialCommandGroup(
            shooter.shootCMD(),
            WaitCommand(1400),
            indexer.feedAllShooter(),
            InstantCommand({ shooter.stop() })
        )
    }

    fun ajustHood(): Command {
        return InstantCommand({ hood.setHoodPosition(getObtainedSetPointForHood()) })
    }

    fun stopShooter(): Command {
        return InstantCommand({ shooter.stop() })
    }
}
package org.firstinspires.ftc.teamcode.systems.shooterSystem

import Angle
import androidx.core.util.Supplier
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.utils.interpolation.InterpolatingDouble
import org.firstinspires.ftc.teamcode.utils.interpolation.InterpolatingTreeMap

/**
 * These values need to be measured physically
 */






/*val lInterpolationFarConfig = LInterpolationConfig(
    // The angle asked for is the hood's angle at that position, NOT RELATED TO THE APRILTAG

    // TODO: TUNE IN FIELD CALIBRATION; WORKS PERFECTLY WITH OUR MANCRAFT GOAL, BUT CAN'T ASSURE
    // TODO: IT DOES WITH THE ACTUAL GOAL
    // angle at 19.18in was 0.69 rotations
    // angle at 58.31in was 0.54 rotations
    firstCoordinate = Point(Distance.fromInches(54.82), Angle.fromRotations(0.67)),
    secondCoordinate = Point(Distance.fromInches(36.0), Angle.fromRotations(0.71))
)*/



@Suppress("JoinDeclarationAndAssignment")
class ShooterSystem(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    distanceToAprilTagInches: Supplier<Double>,
    val isLLResultValid: Supplier<Boolean>
) {

    // Declaring subsystems
    val indexer: Indexer
    val shooter: Shooter
    val hood: Hood

    // Setting the interpolation & its supplier
    private val distanceToAprilTag = { Distance.fromInches(distanceToAprilTagInches.get()) }
    private val hoodInterpolator: InterpolationConstructor
    private val shooterInterpolator: InterpolationConstructor

    init {
        // Starting interpolators
        hoodInterpolator = InterpolationConstructor(distanceToAprilTag, "hood")
        shooterInterpolator = InterpolationConstructor(distanceToAprilTag, "shooter")

        // Assigning subsystems
        shooter = Shooter(hardwareMap, telemetry)
        indexer = Indexer(hardwareMap, telemetry)
        hood = Hood(hardwareMap, telemetry, hoodInterpolator)
    }

    // todo: fallback in case interpolation doesn't work
    // Uses interpolation to get desired hood values
    /*fun getHoodTarget(): Angle {
        return Angle.fromDegrees(interpolator.getDesiredPoint())
    }*/

    // todo: test
    /*fun getObtainedSetPointForHood(): Angle {
        if (isLLResultValid.get()) {
            return interpolator.getDesiredPoint()
        } else {
            return HoodConstants.Positions.homePosition
        }
    }*/

    // Command to shoot the Artifacts according to pattern
    fun shoot(motifPatterns: MotifPatterns) : Command {
        return SequentialCommandGroup(
            shooter.shootCMD(),
            WaitCommand(400),
            InstantCommand({ indexer.feedShooterCMD(motifPatterns).schedule() }),
            WaitCommand(2000),
            stopShooter()
        )
    }

    // Command to stop the shooter
    fun stopShooter(): Command {
        return shooter.stopCMD()
    }

    // Returns whether the indexer is full or not
    fun isFull(): Boolean {
        return indexer.isFull()
    }
}
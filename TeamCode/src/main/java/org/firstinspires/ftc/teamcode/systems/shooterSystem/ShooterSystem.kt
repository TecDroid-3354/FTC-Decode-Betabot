package org.firstinspires.ftc.teamcode.systems.shooterSystem

import Angle
import AngularVelocity
import androidx.core.util.Supplier
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.subsystems.shooter.HoodConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.turret.turretConfig
import org.firstinspires.ftc.teamcode.utils.interpolation.InterpolatingDouble
import org.firstinspires.ftc.teamcode.utils.interpolation.InterpolatingTreeMap
import java.util.function.BooleanSupplier

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
    val distanceToAprilTagInches: Supplier<Double>,
    val isLLResultValid: Supplier<Boolean>
) {

    // Declaring subsystems
    val indexer: Indexer
    val shooter: Shooter
    val hood: Hood

    // Setting the interpolation & its supplier
    private val hoodInterpolator: InterpolationConstructor
    private val shooterInterpolator: InterpolationConstructor

    private var shooterPoint = AngularVelocity.fromRpm(1000.0)
    private var hoodPoint = Angle.fromRotations(HoodConstants.Positions.minPosition.rotations)

    init {
        // Starting interpolators
        hoodInterpolator = InterpolationConstructor("hood")
        shooterInterpolator = InterpolationConstructor("shooter")

        // Assigning subsystems
        indexer = Indexer(hardwareMap, telemetry)
        shooter = Shooter(hardwareMap, telemetry)
        hood = Hood(hardwareMap, telemetry, hoodInterpolator)
    }

    fun periodic() {
        shooterPoint = AngularVelocity.fromRpm(shooterInterpolator.getDesiredPoint(Distance.fromInches(distanceToAprilTagInches.get())))
        hoodPoint = Angle.fromRotations(hoodInterpolator.getDesiredPoint(Distance.fromInches(distanceToAprilTagInches.get())))

//        telemetry.addData("Shooter Interpolation", shooterPoint.rpm)
//        telemetry.addData("Hood Interpolation", hoodPoint.rotations)
//        telemetry.addData("Distance to AprilTag (supplier) (Shooter system)", Distance.fromInches(distanceToAprilTagInches.get()).inches)
        //hoodInterpolator.log(distanceToAprilTag, telemetry)
        //telemetry.update()
    }

    // Command to shoot the Artifacts according to pattern
    fun shoot(motifPatterns: MotifPatterns) : Command {
        val sequentialCMD = SequentialCommandGroup(
            InstantCommand({
                shooter.setFlyWheelVelocityFunc(AngularVelocity.fromRpm(shooterPoint.rpm))
            }),
            InstantCommand({ hood.setHoodPosition(hoodPoint) }),
//          shooter.setFlyWheelVelocity(AngularVelocity.fromRpm(4500.0)),
            WaitCommand(500),
            InstantCommand({ indexer.feedShooterCMD(motifPatterns).schedule() }),
            WaitCommand(2500),
            shooter.setFlyWheelVelocity(AngularVelocity.fromRpm(1000.0))
        )

        for (subsystem in listOf<SubsystemBase>(shooter, indexer, hood)) {
            sequentialCMD.addRequirements(subsystem)
        }

        return sequentialCMD
    }

    fun shoot(motifPatterns: MotifPatterns, velocity: AngularVelocity, hoodAngle: Angle) : Command {
        val sequentialCMD =  SequentialCommandGroup(
            InstantCommand({ hood.setHoodPosition(hoodAngle) }),
            shooter.setFlyWheelVelocity(velocity),
            WaitCommand(800),
            InstantCommand({ indexer.feedShooterCMD(motifPatterns).schedule() }, indexer),
            WaitCommand(2600),
            shooter.setFlyWheelVelocity(AngularVelocity.fromRpm(1000.0))
        )

        for (subsystem in listOf<SubsystemBase>(shooter, indexer, hood)) {
            sequentialCMD.addRequirements(subsystem)
        }

        return sequentialCMD
    }

    // Command to stop the shooter
    fun stopShooter(): Command {
        return shooter.stopCMD()
    }

    // Returns whether the indexer is full or not
    fun isFull(): BooleanSupplier {
        return BooleanSupplier { indexer.isFull() }
    }

    fun isShooterActive():Boolean {
        return shooter.isActive()
    }
}
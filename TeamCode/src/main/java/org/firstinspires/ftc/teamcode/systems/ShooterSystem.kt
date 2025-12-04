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
import org.firstinspires.ftc.teamcode.systems.Point

val lInterpolationConfig = LInterpolationConfig(
    Point(22.18, 255.2),
    Point(60.0, 280.0)
)

@Suppress("JoinDeclarationAndAssignment")
class ShooterSystem(hw: HardwareMap, val telemetry: Telemetry, var aprilTagDistance: Supplier<Distance>) {

    val shooter: Shooter
    val indexer: Indexer
    val hood: Hood

    val interpolation = LinearInterpolationConstructor(lInterpolationConfig, aprilTagDistance)

    init {
        shooter = Shooter(hw, telemetry)
        indexer = Indexer(hw, telemetry)
        hood = Hood(hw, telemetry)
    }

//    private fun adjustHood() {
//
//    }

    fun getObtainedSetPointForHood(): Angle {
        return Angle.fromDegrees(interpolation.getDesiredPoint())
    }

    fun shoot(motifPatterns: MotifPatterns) : Command {
        return SequentialCommandGroup(
            shooter.shootCMD(),
            //InstantCommand({ adjustHood() }),
            WaitCommand(850),
            indexer.feedShooter(motifPatterns),
            InstantCommand({ shooter.stop() })
        )
    }
}
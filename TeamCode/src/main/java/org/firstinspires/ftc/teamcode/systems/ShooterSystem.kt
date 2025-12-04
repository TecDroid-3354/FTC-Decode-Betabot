package org.firstinspires.ftc.teamcode.systems

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

@Suppress("JoinDeclarationAndAssignment")
class ShooterSystem(hw: HardwareMap, val telemetry: Telemetry, var aprilTagDistance: Supplier<Distance>) {

    val shooter: Shooter
    val indexer: Indexer
    val hood: Hood

    init {
        shooter = Shooter(hw, telemetry)
        indexer = Indexer(hw, telemetry)
        hood = Hood(hw, telemetry)
    }

    private fun adjustHood() {
        val angle = (aprilTagDistance.get().cm * 1.0) + 10.0
        hood.setHoodPosition(angle)
    }

    fun shoot(motifPatterns: MotifPatterns) : Command {
        return SequentialCommandGroup(
            shooter.shootCMD(),
            InstantCommand({ adjustHood() }),
            WaitCommand(650),
            indexer.feedAllShooter(),
            InstantCommand({ shooter.stop() })
        )

    }
}
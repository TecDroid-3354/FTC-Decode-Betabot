package org.firstinspires.ftc.teamcode.systems

import Distance
import androidx.core.util.Supplier
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood

class ShooterSystem(val shooter: Shooter, val hood: Hood, val indexer: Indexer, var aprilTagDistance: Supplier<Distance>) {

    private fun adjustHood() {
        val angle = (aprilTagDistance.get().cm * 1.0) + 10.0
        hood.setHoodPosition(angle)
    }

    fun shoot(motifPatterns: MotifPatterns) : Command {
        return SequentialCommandGroup(
            shooter.shootCMD(),
            InstantCommand({ adjustHood() }),
            WaitCommand(500),
            indexer.feedShooter(motifPatterns),
            InstantCommand({ shooter.stop() })
        )

    }
}
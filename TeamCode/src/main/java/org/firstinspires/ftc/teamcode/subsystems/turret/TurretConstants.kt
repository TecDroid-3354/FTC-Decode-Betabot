package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.utils.positionMotorEx.PositionMotorExConfig

class TurretConstants {
    object Identification {
        val turretId = "turretMotor"
    }

    object Configuration {
        val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        val direction = Motor.Direction.FORWARD
        val ticksPerRevolution = Motor.GoBILDA.RPM_435.cpr
        val pidfCoefficients = PIDFCoefficients(0.0, 0.0, 0.0, 0.0)
        val gearRatio = 100.0 / 537.0 // 13.7
        val powerThreshold = 0.1
    }

    object Limits {
        val minimumLimit = Angle.fromDegrees(-180.0)
        val maximumLimit = Angle.fromDegrees(50.0)
    }
}

val turretMotorConfig = PositionMotorExConfig(
    TurretConstants.Configuration.zeroPowerBehavior,
    TurretConstants.Configuration.direction,
    TurretConstants.Configuration.ticksPerRevolution,
    TurretConstants.Configuration.pidfCoefficients,
    TurretConstants.Configuration.gearRatio,
    TurretConstants.Configuration.powerThreshold
)
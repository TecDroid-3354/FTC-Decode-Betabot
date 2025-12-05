package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.utils.positionMotorEx.PositionMotorExConfig
import com.bylazar.configurables.annotations.Configurable

class TurretConstants {

    object Identification {
        // The motor's Id, this must be called inside the robot's configuration in the Control Hub
        val turretId = "turretMotor"
    }

    object Configuration {
        // When not given any output power, the motor will behave like this
        val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        // The motor's direction
        val direction = Motor.Direction.FORWARD
        // The motor's ticks per revolution
        val ticksPerRevolution = Motor.GoBILDA.RPM_435.cpr
        // The subsystem reduction, this value means that for every 13.7 revolutions of the motor, the turret
        // will only turn one revolution
        val gearRatio = 100.0 / 537.0 // 13.7
        // The minimum output the motor has to be commanded for it to move
        val powerThreshold = 0.0001
    }

    object Limits {
        // It is quite important to remark that these limits will only be useful if the robot is turned on
        // with the turret facing towards the battery and the Expansion Hub
        /*
        * The turret starts at -90° when taking the Front Intake as the robots front. From here, it can
        * physically move towards the following limits:
        * */
        val minimumLimit = Angle.fromDegrees(-100.0)
        val maximumLimit = Angle.fromDegrees(125.0)
    }

    // The subsystem's PIDF controller, it must be @Configurable so it can be changed in real-time using Panels
    @Configurable
    class PIDF {
        companion object {
            @JvmField
            var pidfCoefficients = PIDFCoefficients(0.028, 0.0, 0.0, 0.002)
        }
    }
}

/**
 * Receives every configuration the motor needs to be specified and keeps it inside only one [PositionMotorExConfig]
 */
val turretMotorConfig = PositionMotorExConfig(
    TurretConstants.Configuration.zeroPowerBehavior,
    TurretConstants.Configuration.direction,
    TurretConstants.Configuration.ticksPerRevolution,
    TurretConstants.PIDF.pidfCoefficients,
    TurretConstants.Configuration.gearRatio,
    TurretConstants.Configuration.powerThreshold
)
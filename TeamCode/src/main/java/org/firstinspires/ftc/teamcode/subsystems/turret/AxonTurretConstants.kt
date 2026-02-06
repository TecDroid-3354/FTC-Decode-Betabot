package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServo
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServoConfig

@Configurable
class AxonTurretConstants {

    object Identification {

        object RightServo {

            val servoId = "rightServo"
            val absoluteId = "rightAbs"
        }

        object LeftServo {

            val servoId = "leftServo"
            val absoluteId = "leftAbs"
        }
    }

    object Configuration {

        val direction = RTPServo.Direction.FORWARD

        object AbsoluteEncoder {

            val maximumVoltage = Voltage.fromVolts(3.23)
            val offset = Angle.fromDegrees(175.68 - 177.24)
        }
    }

    companion object {
        @JvmField
        var turretControllerCoefficients = PIDFCoefficients(0.006, 0.0, 0.0, 0.000025)
    }

    object PhysicalDescription {
        const val gearRatio = 1.0 / 5.5

        val limits = Limits(Angle.fromDegrees(-141.0), Angle.fromDegrees(140.0))
    }
}

data class Limits(
    val minVal: Angle,
    val maxVal: Angle
)

val rightServoConfig = RTPServoConfig(
    AxonTurretConstants.Identification.RightServo.servoId,
    AxonTurretConstants.Identification.RightServo.absoluteId,
    AxonTurretConstants.Configuration.AbsoluteEncoder.maximumVoltage,
    AxonTurretConstants.Configuration.direction,
    AxonTurretConstants.Configuration.AbsoluteEncoder.offset,
    1.0,
    AxonTurretConstants.PhysicalDescription.gearRatio,
    AxonTurretConstants.turretControllerCoefficients
)

val leftServoConfig = RTPServoConfig(
    AxonTurretConstants.Identification.LeftServo.servoId,
    AxonTurretConstants.Identification.LeftServo.absoluteId,
    AxonTurretConstants.Configuration.AbsoluteEncoder.maximumVoltage,
    AxonTurretConstants.Configuration.direction,
    AxonTurretConstants.Configuration.AbsoluteEncoder.offset,
    1.0,
    AxonTurretConstants.PhysicalDescription.gearRatio,
    AxonTurretConstants.turretControllerCoefficients
)

val turretConfig = AxonTurretConfig(
    rightServoConfig,
    leftServoConfig,
    AxonTurretConstants.PhysicalDescription.limits
)

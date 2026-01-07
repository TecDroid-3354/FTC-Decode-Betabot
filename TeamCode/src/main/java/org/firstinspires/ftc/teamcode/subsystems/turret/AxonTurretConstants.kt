package org.firstinspires.ftc.teamcode.subsystems.turret

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

        object RightServo {

            val direction = RTPServo.Direction.REVERSE

            object AbsoluteEncoder {

                val maximumVoltage = Voltage.fromVolts(3.178)
                val offset = Angle.fromDegrees(86.5125 + 7.5)
            }
        }


        object LeftServo {

            val direction = RTPServo.Direction.REVERSE

            object AbsoluteEncoder {

                val maximumVoltage = Voltage.fromVolts(3.205)
                val offset = Angle.fromDegrees(86.5125 + 7.5)
            }
        }
    }

    companion object {
        @JvmField
        var rightPIDCoefficients = PIDFCoefficients(0.00725, 0.0, 0.0, 0.000025)
        @JvmField
        var leftPIDCoefficients = PIDFCoefficients(0.006, 0.0, 0.0, 0.0)
    }

    object Limits {
        val turretAngleLimits = Angle.fromDegrees(-180.0).degrees..Angle.fromDegrees(180.0).degrees
    }
}

val rightServoConfig = RTPServoConfig(
    AxonTurretConstants.Identification.RightServo.servoId,
    AxonTurretConstants.Identification.RightServo.absoluteId,
    AxonTurretConstants.Configuration.RightServo.AbsoluteEncoder.maximumVoltage,
    AxonTurretConstants.Configuration.RightServo.direction,
    AxonTurretConstants.Configuration.RightServo.AbsoluteEncoder.offset,
    1.0,
    AxonTurretConstants.rightPIDCoefficients
)

val leftServoConfig = RTPServoConfig(
    AxonTurretConstants.Identification.LeftServo.servoId,
    AxonTurretConstants.Identification.LeftServo.absoluteId,
    AxonTurretConstants.Configuration.LeftServo.AbsoluteEncoder.maximumVoltage,
    AxonTurretConstants.Configuration.LeftServo.direction,
    AxonTurretConstants.Configuration.LeftServo.AbsoluteEncoder.offset,
    1.0,
    AxonTurretConstants.leftPIDCoefficients
)

val turretConfig = AxonTurretConfig(
    rightServoConfig,
    leftServoConfig,
    AxonTurretConstants.Limits.turretAngleLimits
)

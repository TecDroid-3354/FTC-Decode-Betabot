package org.firstinspires.ftc.teamcode.axonTurretTest

import Angle
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurret
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurretConfig
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServoConfig
import org.firstinspires.ftc.teamcode.utils.gyroscopes.RevHubIMU
import org.firstinspires.ftc.teamcode.utils.gyroscopes.RevHubIMUConfig
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServo


// Personally, I chose to run my code using a command-based Op Mode since it works better for me
// In a regular LinearOpMode, processes are executed in a sequential workflow
// In an OpMode, on the other hand, code is executed through loops

/* To connect to the robot and deploy the code wirelessly, type the following in the terminal:
 *    adb connect 192.168.43.1:5555 (connects to Control Hub)
 *    adb connect 192.168.43.1:8080 (connects to FTC Dashboard)
 * To visit the FTC dashboard online (while connected to the Control Hub's internet)
 *    http://192.168.43.1:8080/?page=connection.html&pop=true
 *
 */

object AprilTagLocationInDegrees {
    val blueAprilTag = Angle.fromDegrees(135.0)

    val redAprilTag = Angle.fromDegrees(45.0)
}

// Real-time pid configuration
@Configurable
class AxonTurretConstants {
    companion object {
        @JvmField
        var pidCoefficients = PIDFCoefficients(0.015, 0.0, 0.0, 0.000025)
    }
    object Limits {
        val turretAngleLimits = Angle.fromDegrees(-180.0).degrees..Angle.fromDegrees(180.0).degrees
    }
}

val rightServoConfig = RTPServoConfig(
    "rightServo",
    "rightAbs",
    RTPServo.Direction.REVERSE,
    Angle.fromDegrees((-13.76 - 56.77)),
    1.0,
    AxonTurretConstants.pidCoefficients
)

val leftServoConfig = RTPServoConfig(
    "leftServo",
    "leftAbs",
    RTPServo.Direction.FORWARD,
    Angle.fromDegrees(180.0),
    1.0,
    AxonTurretConstants.pidCoefficients
)

private val turretConfig = AxonTurretConfig(
    rightServoConfig,
    leftServoConfig,
    AxonTurretConstants.Limits.turretAngleLimits,
    AxonTurretConstants.pidCoefficients,
)

private val revHubIMUConfig = RevHubIMUConfig(
    "imu",
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
)

@TeleOp(name = "AxonTest", group = "Op Mode")
class CMDOpMode: CommandOpMode() {

    /* ! SET UP CODE ! */
    lateinit var turret: AxonTurret

    lateinit var imu: RevHubIMU

    lateinit var controller: GamepadEx

    var turretTarget: Angle = Angle.fromDegrees(0.0)

    // Change this line if the RED April tag location is needed
    val alliance = Alliance.BLUE
    var targetAprilTagLocation: Angle = Angle.fromDegrees(0.0)

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {

        turret = AxonTurret(hardwareMap, telemetry, turretConfig)

        imu = RevHubIMU(hardwareMap, revHubIMUConfig)

        controller = GamepadEx(gamepad1)

        targetAprilTagLocation =
            if (alliance == Alliance.BLUE) AprilTagLocationInDegrees.blueAprilTag
            else AprilTagLocationInDegrees.redAprilTag

        configureButtonBindings()
    }

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {
        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(InstantCommand({
                turret.toggleState()
            }))

//        GamepadButton(controller, GamepadKeys.Button.Y)
//            .whenPressed(InstantCommand({
//                turret.rightServo.changeTargetRotation(Angle.fromDegrees(90.0))
//            }))
//
//        GamepadButton(controller, GamepadKeys.Button.X)
//            .whenPressed(InstantCommand({
//                turret.rightServo.changeTargetRotation(Angle.fromDegrees(-90.0))
//            }))
    }

    // Main code body
    override fun runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize()

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart()

        // Run the scheduler
        while (!isStopRequested && opModeIsActive()) {

            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()

            // SUPER IMPORTANT calling this line for the servo to update the PID feedback

            // Updating the target in relation to the robot's heading
            turretTarget = targetAprilTagLocation - Angle.fromDegrees(imu.getYaw().degrees)

            // Actually aligning to it
            //turret.setTargetTurretAngle(turretTarget)

            // Useful data
            telemetry.addData("imu reading", imu.getYaw().degrees)
            telemetry.addData("Turret Target", normalizeDegrees(turretTarget.degrees))
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
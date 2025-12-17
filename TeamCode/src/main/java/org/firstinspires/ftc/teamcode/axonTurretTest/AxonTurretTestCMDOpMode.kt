package org.firstinspires.ftc.teamcode.axonTurretTest

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import org.firstinspires.ftc.teamcode.systems.LinearInterpolationConstructor
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPAxon
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServo
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServoConfig
import org.firstinspires.ftc.teamcode.utils.gyroscopes.RevHubIMU
import org.firstinspires.ftc.teamcode.utils.gyroscopes.RevHubIMUConfig


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

// Real-time pid configuration
@Configurable
private class AxonConstants {

    object AprilTagLocationInDegrees {

        val blueAprilTag = Angle.fromDegrees(135.0)

        val redAprilTag = Angle.fromDegrees(45.0)

    }
    companion object PIDF {
        @JvmField
        var pidCoefficients = PIDCoefficients(0.002, 0.0, 0.0)
    }
}

private val servoConfig = RTPServoConfig(
    "servo",
    "abs",
    1.0  / 1.0,
    1.0,
    AxonConstants.pidCoefficients
)

private val revHubIMUConfig = RevHubIMUConfig(
    "imu",
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD

)

@TeleOp(name = "AxonTest", group = "Op Mode")
class CMDOpMode : CommandOpMode() {

    /* ! SET UP CODE ! */
    lateinit var servo: RTPServo

    lateinit var controller: GamepadEx

    lateinit var imu: RevHubIMU

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {
        /* Subsystem initialization */
        servo = RTPServo(hardwareMap, servoConfig)

        imu = RevHubIMU(hardwareMap, revHubIMUConfig)

        controller = GamepadEx(gamepad1)

        servo.getServo().currentAngle

        configureButtonBindings()
    }

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {

        GamepadButton(controller, GamepadKeys.Button.A)
            .whenPressed(InstantCommand({
                servo.getServo().targetRotation = 150.0
            }))

        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(InstantCommand({
                servo.getServo().targetRotation = 180.0
            }))

        GamepadButton(controller, GamepadKeys.Button.X)
            .whenPressed(InstantCommand({
                servo.getServo().targetRotation = 90.0
            }))

        GamepadButton(controller, GamepadKeys.Button.Y)
            .whenPressed(InstantCommand({
                servo.getServo().targetRotation = 0.0
            }))

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
            servo.update()

            val robotOrientationDifference = AxonConstants.AprilTagLocationInDegrees.blueAprilTag.degrees - Angle.fromDegrees(imu.getYaw()).degrees

            servo.getServo().targetRotation = robotOrientationDifference

            telemetry.addData("imu reading", imu.getYaw())
            telemetry.addData("absolutePosition", servo.getServo().totalRotation)
            telemetry.addData("current Angle", servo.getServo().currentAngle)
            telemetry.addData("current Angle", servo.getServo().targetRotation)
            telemetry.addData("difference", robotOrientationDifference)
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
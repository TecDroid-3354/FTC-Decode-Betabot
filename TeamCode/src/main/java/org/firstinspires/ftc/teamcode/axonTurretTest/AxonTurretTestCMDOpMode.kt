package org.firstinspires.ftc.teamcode.axonTurretTest

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import kotlin.math.abs


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
class AxonConstants {
    companion object PIDF {
        @JvmField
        var pidCoefficients = PIDCoefficients(0.002, 0.0, 0.0)
    }
}


@TeleOp(name = "AxonTest", group = "Op Mode")
class CMDOpMode : CommandOpMode() {

    /* ! SET UP CODE ! */

    lateinit var servo: CRServo
    lateinit var absolute: AnalogInput
    lateinit var rtpServo: RTPAxon

    lateinit var controller: GamepadEx

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {
        /* Subsystem initialization */
        servo = hardwareMap.get(CRServo::class.java, "servo")
        absolute = hardwareMap.get(AnalogInput::class.java, "abs")
        rtpServo = RTPAxon(servo, absolute)
        rtpServo.maxPower = 0.8
        rtpServo.setPidCoeffs(AxonConstants.PIDF.pidCoefficients.p, AxonConstants.PIDF.pidCoefficients.i, AxonConstants.PIDF.pidCoefficients.d)

        // IMPORTANT line in order for the servo to respond correctly to commands
        rtpServo.changeTargetRotation(0.1)

        controller = GamepadEx(gamepad1)

        configureButtonBindings()
    }

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {

        // Moves the serro 90 degrees from the current position
        GamepadButton(controller, GamepadKeys.Button.A)
            .whenPressed(InstantCommand({
                rtpServo.changeTargetRotation(90.0)
            }))

        // Moves the sevro to 180 degrees absolute
        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(InstantCommand({
                rtpServo.targetRotation = 180.0
            }))

    }

    fun periodic() {
        rtpServo.setPidCoeffs(AxonConstants.PIDF.pidCoefficients.p, AxonConstants.PIDF.pidCoefficients.i, AxonConstants.PIDF.pidCoefficients.d)
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
            rtpServo.update()

            periodic()

            telemetry.addData("Absolute position", rtpServo.currentAngle)
            telemetry.addData("Total rotation", rtpServo.totalRotation)
            telemetry.addData("Target Rotation", rtpServo.targetRotation)
            telemetry.addLine(rtpServo.log())
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
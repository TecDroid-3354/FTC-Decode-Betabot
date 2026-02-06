package org.firstinspires.ftc.teamcode.OpModes

import Angle
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.geometry.Vector2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.gyroscopes.RevHubIMUConfig
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.turret.AprilTagVectorLocations
import org.firstinspires.ftc.teamcode.subsystems.turret.turretConfig
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
import org.firstinspires.ftc.teamcode.utils.gyroscopes.OtosConfig


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

private val revHubIMUConfig = RevHubIMUConfig(
    "imu",
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
)

val otosConfig = OtosConfig(
    "otos",
    AngleUnit.DEGREES,
    DistanceUnit.INCH
)

@Disabled
@TeleOp(name = "Turret + Mecanum Test", group = "Op Mode")
class AxonTurretTestCMDOpMode: CommandOpMode() {

    /* ! SET UP CODE ! */
    lateinit var turret: AxonTurret

    lateinit var mecanum: SolversMecanum

    lateinit var otos: Otos

    lateinit var controller: GamepadEx

    var turretTarget: Angle = Angle.fromDegrees(0.0)

    // Change this line if the RED April tag location is needed
    val alliance = Alliance.RED
    var targetAprilTagLocation: Vector2d = Vector2d(0.0, 0.0)

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {

        otos = Otos(hardwareMap, telemetry, otosConfig)

        turret = AxonTurret(hardwareMap, telemetry, turretConfig)

        mecanum = SolversMecanum(hardwareMap, telemetry, otos)
        mecanum.defaultCommand = JoystickCmd(
            { controller.leftX },
            { controller.leftY },
            { controller.rightX },
            mecanum
        )

        controller = GamepadEx(gamepad1)

        targetAprilTagLocation =
            if (alliance == Alliance.BLUE) AprilTagVectorLocations.blueGoalCornerVector
            else AprilTagVectorLocations.redGoalCornerVector

        configureButtonBindings()
    }

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {
        GamepadButton(controller, GamepadKeys.Button.START)
            .whenPressed(InstantCommand({
                otos.resetTracking()
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

            // Get robot's location in a vector
            val robotLocationX = -otos.getPositionVector().x

            val robotLocationY = -otos.getPositionVector().y
            // Get the robot's heading
            val robotHeading = otos.getHeading()
            // Get the vector difference from the goal's and robot position
            val newVector = targetAprilTagLocation - Vector2d(robotLocationX, robotLocationY)
            // The angle to the positive x axis of the vector difference
            val angleToGoal = Angle.fromRadians(newVector.angle())
            // Getting the turret angle by subtracting the robot's rotation to the field target angle
            turretTarget = Angle.fromDegrees(
                normalizeDegrees(angleToGoal.degrees - robotHeading.degrees)
            )

            telemetry.addData("Field target Angle", angleToGoal.degrees)
            telemetry.addData("Turret Target", turretTarget.degrees)
            telemetry.addData("Turret Angle", turret.getAbsoluteAngle().degrees)
            telemetry.addData("otos x", robotLocationX)
            telemetry.addData("otos y", robotLocationY)
            telemetry.addData("heading", otos.getHeading().degrees)
            telemetry.addData("otos offset", otos.otos.offset)
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
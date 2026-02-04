package org.firstinspires.ftc.teamcode.OpModes

import Angle
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.geometry.Vector2d
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.turret.AprilTagVectorLocations
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurret
import org.firstinspires.ftc.teamcode.subsystems.turret.turretConfig
import org.firstinspires.ftc.teamcode.systems.ledSystem.LedSystem
import org.firstinspires.ftc.teamcode.systems.shooterSystem.ShooterSystem
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
import org.firstinspires.ftc.teamcode.vision.Limelight


// Personally, I chose to run my code using a command-based Op Mode since it works better for me
// In a regular LinearOpMode, processes are executed in a sequential workflow
// In an OpMode, on the other hand, code is executed through loops

/* To connect to the robot and deploy the code wirelessly, type the following in the terminal:
 *    adb connect 192.168.43.1:5555 (connects to Control Hub)
 *    adb connect 192.168.43.1:8080 (connects to FTC Dashboard)
 * To visit the FTC dashboard online (while connected to the Control Hub's internet)
 *    http://192.168.43.1:8080/?page=connection.html&pop=true
 */

@TeleOp(name = "CMD", group = "Op Mode")
class CMDOpMode : CommandOpMode() {

    /* ! SET UP CODE ! */

    // Declaring subsystems //

    lateinit var mecanum: SolversMecanum

    lateinit var intake: Intake

    lateinit var turret: AxonTurret
    var turretTarget: Angle = Angle.fromDegrees(0.0)

    lateinit var shooterSystem: ShooterSystem

    // Declaring useful components //
    lateinit var controller: GamepadEx

    lateinit var limelight: Limelight

    lateinit var otos: Otos

    lateinit var leds: LedSystem

    val limelightIdFilter: IntArray = intArrayOf(20, 21, 22, 23, 24)

    // Change this line if the RED April tag location is needed
    val alliance = Alliance.BLUE
    var targetAprilTagLocation: Vector2d = Vector2d(0.0, 0.0)

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {
        /* Subsystem initialization */

        // Initializing the OTOS
        otos = Otos(hardwareMap, telemetry, otosConfig)

        // Initializing the mecanum & its default command
        mecanum = SolversMecanum(hardwareMap, telemetry, otos.getSensorInstance())
        mecanum.defaultCommand = JoystickCmd(
            { controller.leftX },
            { controller.leftY },
            { controller.rightX },
            mecanum
        )

        // Intake initialization
        intake = Intake(hardwareMap, telemetry)

        // Limelight initialization
        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()

        turret = AxonTurret(hardwareMap, telemetry, turretConfig) { turretTarget }

        // Shooter system initialization
        shooterSystem = ShooterSystem(hardwareMap, telemetry,
            { limelight.getDistanceToGoal(limelightIdFilter).inches },
            { limelight.llResult != null && limelight.llResult!!.isValid }
        )

        // LEDs initialization
        leds = LedSystem(hardwareMap, telemetry)
        // Initializing controller & button bindings
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

        GamepadButton(controller, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                intake.enableBothOuttakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )

        GamepadButton(controller, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(
                intake.enableBothIntakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )

        GamepadButton(controller, GamepadKeys.Button.A)
            .whenPressed(
                InstantCommand({ shooterSystem.hood.modifyCurrentPositionBy(Angle.fromRotations(0.01)) })
            )

        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(
                InstantCommand({ shooterSystem.hood.modifyCurrentPositionBy(Angle.fromRotations(-0.01)) })
            )

        GamepadButton(controller, GamepadKeys.Button.X)
            .whenPressed(
                InstantCommand({ shooterSystem.hood.setHoodPosition(Angle.fromRotations(0.0)) })
            )

        GamepadButton(controller, GamepadKeys.Button.Y)
            .whenPressed(
                InstantCommand({ shooterSystem.indexer.feedShooterCMD(limelight.getMotifPattern()).schedule() })
            )

        Trigger { controller.gamepad.right_trigger > 0.2 }
            .whenActive(shooterSystem.shooter.shootCMD())
            .whenInactive(shooterSystem.stopShooter())
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

            //turretTarget = targetAprilTagLocation - Angle.fromDegrees(otos.position.h)

            controller.readButtons()

            telemetry.addData("distance to apriltag", limelight.getDistanceToGoal(limelightIdFilter))
            telemetry.addData("interpolation", shooterSystem.getObtainedSetPointForHood().degrees)


            telemetry.addData("Pattern", limelight.getMotifPattern())
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
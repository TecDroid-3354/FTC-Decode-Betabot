import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.geometry.Vector2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import org.firstinspires.ftc.teamcode.OpModes.otosConfig
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.turret.AprilTagVectorLocations
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurret
import org.firstinspires.ftc.teamcode.subsystems.turret.turretConfig
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
import org.firstinspires.ftc.teamcode.vision.Limelight

@TeleOp(name = "Integrated", group = "Op Mode")
class IntegrationTestOpMode: CommandOpMode() {

    /* ! SET UP CODE ! */
    lateinit var otos: Otos

    lateinit var mecanum: SolversMecanum

    lateinit var intake: Intake

    lateinit var indexer: Indexer

    lateinit var shooter: Shooter

    lateinit var hood: Hood

    lateinit var limelight: Limelight

    lateinit var turret: AxonTurret

    var turretTarget: Angle = Angle.fromDegrees(0.0)

    // Change this line if the RED April tag location is needed
    val alliance = Alliance.RED

    var targetAprilTagLocation: Vector2d = Vector2d(0.0, 0.0)

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {

        otos = Otos(hardwareMap, telemetry, otosConfig)

        mecanum = SolversMecanum(hardwareMap, telemetry, otos)
        mecanum.defaultCommand = JoystickCmd(
            { controller.leftX },
            { controller.leftY },
            { controller.rightX },
            mecanum
        )

        shooter = Shooter(hardwareMap, telemetry)
        intake = Intake(hardwareMap, telemetry)
        indexer = Indexer(hardwareMap, telemetry)

        hood = Hood(hardwareMap, telemetry)

        turret = AxonTurret(hardwareMap, telemetry, turretConfig) { turretTarget }

        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()

        controller = GamepadEx(gamepad1)

        targetAprilTagLocation =
            if (alliance == Alliance.BLUE) AprilTagVectorLocations.blueAprilTag
            else AprilTagVectorLocations.redAprilTag

        configureButtonBindings()
    }

    //lateinit var hood: Hood
    lateinit var controller: GamepadEx

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {
        GamepadButton(controller, GamepadKeys.Button.A)
            .whenPressed(
                InstantCommand({ hood.modifyCurrentPositionBy(Angle.fromRotations(0.01)) })
            )

        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(
                InstantCommand({ hood.modifyCurrentPositionBy(Angle.fromRotations(-0.01)) })
            )

        GamepadButton(controller, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(
                intake.enableBothIntakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )

        GamepadButton(controller, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                intake.enableBothOuttakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )

        GamepadButton(controller, GamepadKeys.Button.Y)
            .whenPressed(
                indexer.feedShooterCMD(MotifPatterns.GREEN_PURPLE_PURPLE)
            )
    }

    private fun calculateTurretTarget() {
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
            calculateTurretTarget()
            controller.readButtons()

            telemetry.addData("Turret target", turretTarget.degrees)
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
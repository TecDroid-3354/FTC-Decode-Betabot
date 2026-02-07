package org.firstinspires.ftc.teamcode.auto

import Angle
import AngularVelocity
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.auto.Visualizer.Draw
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurret
import org.firstinspires.ftc.teamcode.subsystems.turret.turretConfig
import org.firstinspires.ftc.teamcode.systems.shooterSystem.ShooterSystem
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
import org.firstinspires.ftc.teamcode.utils.gyroscopes.OtosConfig
import org.firstinspires.ftc.teamcode.vision.Limelight


@Autonomous(name = "Blue 9", group = "Blue")
class Blue9 : CommandOpMode() {
    //Variables
    private var pathState = 0
    private lateinit var pathTimer: Timer
    private lateinit var follower: Follower
    private lateinit var paths: Paths


    // Subsystem Instances
    private lateinit var shooterSystem: ShooterSystem

    private lateinit var intake: Intake
    private lateinit var turret: AxonTurret

    private lateinit var otos: Otos
    private lateinit var limelight: Limelight

    private val startPose = Pose(56.000, 8.000, Math.toRadians(180.0)) // Start Pose of our robot.

    class Paths(follower: Follower) {
        var Path1: PathChain?
        var line2: PathChain?
        var line3: PathChain?

        init {
            Path1 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(56.000, 8.000),

                    Pose(56.000, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))

                .build()

            line2 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(56.000, 36.000),

                    Pose(7.533, 36.217)
                )
            ).setTangentHeadingInterpolation()

                .build()

            line3 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(7.533, 36.217),

                    Pose(64.032, 88.660)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))

                .build()
        }
    }


    override fun initialize() {
        // Initializing subsystems
        otos = Otos(hardwareMap, telemetry, OtosConfig("otos", AngleUnit.DEGREES, DistanceUnit.INCH))
        otos.sensorConfiguration()
        otos.setPosition(SparkFunOTOS.Pose2D(-8.0, -64.0, 0.0))

        turret = AxonTurret(hardwareMap, telemetry, turretConfig)
        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()
        intake = Intake(hardwareMap, telemetry)
        shooterSystem = ShooterSystem(
            hardwareMap, telemetry,
            { limelight.getDistanceToGoal(intArrayOf(20, 24)).inches },
            { limelight.llResult != null }
        )

        follower = Constants.createFollower(hardwareMap)
        paths = Paths(follower)

        pathTimer = Timer()

        follower.setStartingPose(startPose)
    }

    override fun runOpMode() {
        initialize()

        waitForStart()

        setPathState(0)

        while (!isStopRequested && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()

            // Actual path following
            follower.update()
            autonomousPathUpdates()
            drawCurrent()

            // Updating the telemetry
            telemetry.addData("current pathState", pathState)
            telemetry.update()
        }
    }

    private fun autonomousPathUpdates() {
        when (pathState) {
            0 -> {
                /*turret.setTurretVoltage(1.0)
                sleep(250)
                turret.setTurretVoltage(0.0)
                sleep(200)

                shooterSystem.shoot(
                    limelight.getMotifPattern(),
                    AngularVelocity.fromRpm(4300.0),
                    Angle.fromRotations(0.8)
                ).andThen( InstantCommand({ setPathState(1) }) ).schedule()*/
                setPathState(1)
            }

            1 -> if (!follower.isBusy) {
                follower.followPath(paths.Path1, true)
                setPathState(2)
            }

            2 -> if (!follower.isBusy) {
                intake!!.enableBothIntakes().schedule()
                follower!!.followPath(paths!!.line2, true)

                setPathState(3)
            }

            3 -> if (!follower!!.isBusy()) {
                intake!!.stopBothIntakes().schedule()
                follower!!.followPath(paths!!.line3, true)

                setPathState(4)
            }

            4 -> if (!follower.isBusy) {
                follower.breakFollowing()

                shooterSystem.shoot(
                    limelight.getMotifPattern(),
                    AngularVelocity.fromRpm(4300.0),
                    Angle.fromRotations(0.8)
                ).andThen( InstantCommand({ setPathState(-1) }) ).schedule()
            }
        }
    }

    private fun setPathState(number: Int) {
        pathState = number
        pathTimer.resetTimer()
        telemetry.addData("Current path state to be followed", number)
    }

    private fun shootOut() {
        //Activates rollers and feed the shooter with the correct Motif Pattern.
        shooterSystem.shoot(limelight.getMotifPattern()).schedule()
    }

    private fun drawCurrent() {
        try {
            Draw.drawRobot(follower.pose)
            Draw.sendPacket()
        } catch (e: Exception) {
            throw RuntimeException("Drawing failed " + e)
        }
    }
}
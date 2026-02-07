package org.firstinspires.ftc.teamcode.auto.hardCoded

import com.pedropathing.util.Timer
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurret
import org.firstinspires.ftc.teamcode.subsystems.turret.turretConfig
import org.firstinspires.ftc.teamcode.systems.shooterSystem.ShooterSystem
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
import org.firstinspires.ftc.teamcode.utils.gyroscopes.OtosConfig
import org.firstinspires.ftc.teamcode.vision.Limelight

@Autonomous(name = "Red Far", group = "Blue")
class RedFar : CommandOpMode() {
    //Pedro ´Pathin
    //Variables
    private var pathState = 0


    // Subsystem Instances
    private lateinit var shooterSystem: ShooterSystem

    private lateinit var intake: Intake

    private lateinit var turret: AxonTurret

    private lateinit var mecanum: SolversMecanum

    private lateinit var otos: Otos
    private lateinit var limelight: Limelight

    override fun initialize() {
        // Initializing subsystems
        otos =
            Otos(hardwareMap, telemetry, OtosConfig("otos", AngleUnit.DEGREES, DistanceUnit.INCH))
        otos.sensorConfiguration()
        otos.setPosition(SparkFunOTOS.Pose2D(8.0, -64.0, 0.0))

        mecanum = SolversMecanum(hardwareMap, telemetry, otos)
        turret = AxonTurret(hardwareMap, telemetry, turretConfig)
        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()
        intake = Intake(hardwareMap, telemetry)
        shooterSystem = ShooterSystem(
            hardwareMap, telemetry,
            { limelight.getDistanceToGoal(intArrayOf(20, 24)).inches },
            { limelight.llResult != null }
        )
    }

    override fun runOpMode() {
        initialize()

        waitForStart()

        setPathState(0)

        while (!isStopRequested() && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()

            autonomousPathUpdates()

            // Updating the telemetry
            telemetry.addData("current pathState", pathState)
//            shooterSystem.shooter.log()
//            shooterSystem.hood.log()
//            shooterSystem.indexer.log()
//            otos.log()
            telemetry.update()
        }
    }

    private fun autonomousPathUpdates() {
        when (pathState) {
            0 -> {
                /*SequentialCommandGroup(
                    turret.setTurretAngle { Angle.fromDegrees(10.0) }.withTimeout(200),
                    shooterSystem.shoot(limelight.getMotifPattern(), AngularVelocity.fromRpm(3000.0), Angle.fromRotations(0.65)).withTimeout(500),
                    WaitCommand(500),
                    InstantCommand({ setPathState(1) } )
                ).schedule()*/

                turret.setTurretVoltage(-1.0)
                sleep(200)
                turret.setTurretVoltage(0.0)
                sleep(200)
                shooterSystem.shooter.setFlyWheelVelocity(AngularVelocity.fromRpm(4250.0)).schedule()
                shooterSystem.hood.setHoodPosition(Angle.fromRotations(0.8))
                sleep(1300)

                shooterSystem.indexer.slotList[0].feed()
                sleep(800)
                shooterSystem.indexer.slotList[0].home()
                sleep(450)
                shooterSystem.indexer.slotList[1].feed()
                sleep(850)
                shooterSystem.indexer.slotList[1].home()
                sleep(450)
                shooterSystem.indexer.slotList[2].feed()
                sleep(800)
                shooterSystem.indexer.slotList[2].home()

                sleep(2600)
                shooterSystem.shooter.setFlyWheelVelocity(AngularVelocity.fromRpm(1000.0)).schedule()
                mecanum.setChassisSpeeds(ChassisSpeeds(-1.0, 0.0, 0.0))
                sleep(500)
                mecanum.setChassisSpeeds(ChassisSpeeds(0.0, 0.0, 0.0))
                setPathState(-1)
            }
        }
    }

    private fun setPathState(number: Int) {
        pathState = number
        telemetry.addData("Current path state to be followed", number)
    }

    private fun shootOut() {
        //Activates rollers and feed the shooter with the correct Motif Pattern.
        shooterSystem.shoot(limelight.getMotifPattern()).schedule()
    }
}

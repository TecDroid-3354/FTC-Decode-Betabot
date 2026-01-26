package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum;
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Autonomous(name = "3BallsBlue", group = "All")
public class Blue3Balls extends CommandOpMode {

    /* ! SETUP CODE ! */
    // This following variable, pathState, will serve as the counter variable to determine
    // which is the next path to follow inside a switch statement
    private int pathState;

    // The following timer allows to set a time limit to each path
    private Timer pathTimer;

    // Declaring all subsystems
    private Intake intake;
    private Indexer indexer;
    private Hood hood;
    private Turret turret;
    private Shooter shooter;
    private SolversMecanum mecanum;
    private SparkFunOTOS otos;
    private Boolean readyToShoot = false;

    @Override
    public void initialize() {
        // Initializes the timer that accounts for the timeout before a path is considered done
        pathTimer = new Timer();

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        mecanum = new SolversMecanum(hardwareMap, telemetry, otos);

        // Initializing subsystems
        intake = new Intake(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        hood = new Hood(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);

        new Trigger(() -> readyToShoot)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> hood.setHoodPosition(0.6)),
                        shooter.shootCMD(),
                        new WaitCommand(1400),
                        indexer.feedAllShooter(),
                        new WaitCommand(2000),
                        new InstantCommand(() -> shooter.stopCMD().schedule()),
                        new InstantCommand(() -> setPathState(2)),
                        new InstantCommand(() -> readyToShoot = false)
                ));
    }


    /* ! FUNCTIONAL CODE ! */

    // Main code body
    @Override
    public void runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize();

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart();

        // The following line sets the first path to be followed to be Path number 0
        setPathState(0);

        // Run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run();

            autonomousPathUpdates();

            // Updating the telemetry
            telemetry.update();
        }

        // Cancels all previous commands
        reset();
    }

    // The following class is in charge of changing the followed path. Logic should be added inside
    // here. It is the one called continuously during the autonomous
    public void autonomousPathUpdates() {
        switch (pathState) {
            // The follower is in charge of following a PathChain declared within the Paths object

            // TODO: THE ONLY WAY TO MAKE MOTORS WORK IN AUTO IS THROUGH NON-COMMAND CODE

            // ! Shoot precharged artifacts ! //
            case 0: // Path from starting -> shooting position
                //follower.followPath(ball3Path, true);
                mecanum.setChassisSpeeds(new ChassisSpeeds(0.0, -1.0,0.0));
                sleep(800);
                mecanum.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
                /*sleep(300);
                mecanum.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.4));
                sleep(300);
                mecanum.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));*/
                setPathState(1);
                break;
            case 1: // Path from intake position --> shooting position + indexing + shooting
                readyToShoot = true;
                //sleep(4000);
                //setPathState(2);
                break;
            case 2:
                readyToShoot = false;
                mecanum.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
                sleep(600);
                mecanum.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
                setPathState(-1);
                break;
        }
    }


    // This method will change the number of the target path
    private void setPathState(int number) {
        pathState = number;
        pathTimer.resetTimer();
        telemetry.addData("Current path state to be followed", number);
    }
}
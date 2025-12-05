package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.auto.Visualizer.Draw;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeDirection;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.systems.ShooterSystem;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Autonomous(name = "Blue 9+3", group = "Blue")
public class Blue9Plus3 extends CommandOpMode {

    /* ! SETUP CODE ! */

    // Declares the PedroPathing Follower. This object is in charge of following all paths
    public static Follower follower;

    // Retrieves paths from the Path class
    private Paths paths;

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

    @Override
    public void initialize() {
        // The follower is initialized & set to the starting pose
        follower = Constants.createFollower(hardwareMap);
        // Initializes the class where all paths are created
        paths = new Paths(follower);
        follower.setStartingPose(paths.blue9Plus3Poses.blue9Plus3StartPose);
        // Initializes the timer that accounts for the timeout before a path is considered done
        pathTimer = new Timer();

        // Initializing subsystems
        intake = new Intake(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        hood = new Hood(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
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

            // Actual path following
            follower.update();
            autonomousPathUpdates();
            drawCurrent();

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
                follower.followPath(paths.blue9Plus3ShootPrecharged, true);
                setPathState(1);
                break;
            case 1: // Path from intake position --> shooting position + indexing + shooting
                if (!follower.isBusy()) {
                    new WaitCommand(2000); // waits for the route to be perfectly aligned
                    // Starts rolling the rollers
                    shooter.shootCMD().schedule();
                    // Feeds the shooter as the shooter's rollers roll
                    indexer.feedAllShooter().schedule();
                    new WaitCommand(3000); // waits for the shooter to finish shooting
                    shooter.stop(); // stops the shooter
                    setPathState(2);
                }
                break;


            // ! Pick up & shoot the first row of Artifacts ! //
            case 2: // Path from shooting -> intake position & intaking Artifacts
                if (!follower.isBusy()) {
                    intake.enableBothIntakes(1.0).schedule();
                    follower.followPath(paths.blue9Plus3PickFirstRow, true);
                    new WaitCommand(4000); // waits for it to intake all the Artifacts
                    intake.stopBothIntakes().schedule();
                    setPathState(3);
                }
                break;
            case 3: // Path from intake position --> shooting position + indexing + shooting
                if (!follower.isBusy()) {
                    // TODO: try adding all of this inside a sequential command group
                    follower.followPath(paths.blue9Plus3ShootFirstRow, true);
                    new WaitCommand(2000); // waits for the route to be perfectly aligned
                    // Starts rolling the rollers
                    shooter.shootCMD().schedule();
                    // Feeds the shooter as the shooter's rollers roll
                    indexer.feedAllShooter().schedule();
                    new WaitCommand(3000); // waits for the shooter to finish shooting
                    shooter.stop(); // stops the shooter
                    setPathState(4);
                }
                break;


            // ! Pick up & shoot the second row of Artifacts ! //
            case 4: // Path from shooting position -> intake position
                if (!follower.isBusy()) {
                    intake.enableBothIntakes(1.0).schedule();
                    follower.followPath(paths.blue9Plus3PickSecondRow, true);
                    new WaitCommand(4000); // waits for it to intake all the Artifacts
                    intake.stopBothIntakes().schedule();
                    setPathState(5);
                }
                break;
            case 5: // Path from intake position --> shooting position + indexing + shooting
                if (!follower.isBusy()) {
                    // TODO: try adding all of this inside a sequential command group
                    follower.followPath(paths.blue9Plus3ShootSecondRow, true);
                    new WaitCommand(2000); // waits for the route to be perfectly aligned
                    // Starts rolling the rollers
                    shooter.shootCMD().schedule();
                    // Feeds the shooter as the shooter's rollers roll
                    indexer.feedAllShooter().schedule();
                    new WaitCommand(3000); // waits for the shooter to finish shooting
                    shooter.stop(); // stops the shooter
                    setPathState(6);
                }
                break;


            // ! Pick up & shoot the third row of Artifacts ! //
            case 6: // Path from shooting position -> intake position
                if (!follower.isBusy()) {
                    intake.enableBothIntakes(1.0).schedule();
                    follower.followPath(paths.blue9Plus3PickThirdRow, true);
                    new WaitCommand(4000); // waits for it to intake all the Artifacts
                    intake.stopBothIntakes().schedule();
                    setPathState(7);
                }
                break;
            case 7: // Path from intake position --> shooting position + indexing + shooting
                if (!follower.isBusy()) {
                    // TODO: try adding all of this inside a sequential command group
                    follower.followPath(paths.blue9Plus3ShootThirdRow, true);
                    new WaitCommand(2000); // waits for the route to be perfectly aligned
                    // Starts rolling the rollers
                    shooter.shootCMD().schedule();
                    // Feeds the shooter as the shooter's rollers roll
                    indexer.feedAllShooter().schedule();
                    new WaitCommand(3000); // waits for the shooter to finish shooting
                    shooter.stop(); // stops the shooter
                    setPathState(8);
                }
                break;
        }
    }


    // This method will change the number of the target path
    private void setPathState(int number) {
        pathState = number;
        pathTimer.resetTimer();
        telemetry.addData("Current path state to be followed", number);
    }

    // todo: test this function whenever it is intaking
    private void feedShooter() {
        new WaitCommand(2000); // waits for the route to be perfectly aligned
        indexer.feedAllShooter().schedule();
    }

    // Display in Panels
    private static void drawCurrent() {
        try {
            Draw.drawRobot(follower.getPose());
            Draw.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
}

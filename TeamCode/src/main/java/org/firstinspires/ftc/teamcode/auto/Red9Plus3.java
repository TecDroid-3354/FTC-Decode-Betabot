package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;

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
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeDirection;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.systems.ShooterSystem;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Autonomous(name = "Red 9+3", group = "Red")
public class Red9Plus3 extends CommandOpMode {

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
    private ShooterSystem shooterSystem;
    private Indexer indexer;
    private Hood hood;
    private Turret turret;
    private Shooter shooter;


    // Limelight
    private Limelight limelight;
    private SparkFunOTOS otos;

    private int waitTimeMiliseconds;

    public static void drawCurrent() {
        try {
            Draw.drawRobot(follower.getPose());
            Draw.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }


    @Override
    public void initialize() {
        // The follower is initialized & set to the starting pose
        follower = Constants.createFollower(hardwareMap);
        // Initializes the class where all paths are created
        paths = new Paths(follower);
        follower.setStartingPose(paths.red9Plus3Poses.red9Plus3StartPose);
        // Initializes the timer that accounts for the timeout before a path is considered done
        pathTimer = new Timer();

        // Initializing useful components
        //otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        //limelight = new Limelight(hardwareMap, telemetry, otos);

        // Initializing subsystems
        intake = new Intake(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        // TODO: LIMELIGHTS ARE THE ONES TWEAKING THE WHOLE THING
        /*shooterSystem = new ShooterSystem(
                hardwareMap,
                telemetry,
                () -> limelight.getDistanceToGoalAsDouble(new int[]{24}),
                () -> limelight.getLlResult() != null && limelight.getLlResult().isValid()
            );*/

            waitTimeMiliseconds = 6000;

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
            telemetry.addData("runOpMode is being executed", true);
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

            // ! Shoot precharged artifacts ! //
            case 0:
                // follower.followPath(paths.red9Plus3ShootPrecharged);
                /*new SequentialCommandGroup(
                    new InstantCommand(() -> follower.followPath(paths.red9Plus3ShootPrecharged) ),
                    new InstantCommand (() -> shooter.shoot(limelight.getMotifPattern()) )
                );*/
                telemetry.addData("Started with pathstate 0", true);
                follower.followPath(paths.red9Plus3ShootPrecharged, true);
                //shooterSystem.shoot(MotifPatterns.GREEN_PURPLE_PURPLE);

                setPathState(1);
                break;
            case 1:
//                if (!follower.isBusy()) {
//                    // TODO: THE ONLY WAY TO MAKE MOTORS WORK IN AUTO IS THROUGH NON-COMMAND CODE
//
//
//                    new SequentialCommandGroup(
//                        new InstantCommand(() -> indexer.feedAllShooterAuto()),
//                        new WaitUntilCommand(() -> !indexer.isFull()),
//                        new InstantCommand(() -> setPathState(2))
//                    );
//                    //intake.enableIntake(IntakeDirection.LEFT, 1.0);
//                    telemetry.addData("Started with pathstate 1", true);
//                    //new InstantCommand (() -> intake.enableBothIntakes(1.0) );
//                    setPathState(2);
//                }
//                break;
                if (!follower.isBusy()) {
                    //indexer.feedAllShooterAuto();
                    indexer.feedAllShooter().schedule();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    shooter.shoot();
                    new WaitCommand(5000);
                    shooter.stop();
                    new WaitCommand(1000);
                    setPathState(7);
                }
                break;


            // ! Align itself & pick up the first row of Artifacts ! //

//            case 3:
//                if (!follower.isBusy()) {
//                    intake.enableIntake(IntakeDirection.LEFT, 1.0);
//                    follower.followPath(paths.red9Plus3PickFirstRow, true);
//                    setPathState(4);
//                }
//                break;
//
//            case 69:
//                if (!follower.isBusy()) {
//                    shooter.shoot();
//                    setPathState(3);
//                }
//                break;




//            case 1:
//                if (!follower.isBusy()) {
//                    new SequentialCommandGroup(
//                            new InstantCommand(() -> follower.followPath(paths.red9Plus3PickFirstRow, true) ),
//                            new InstantCommand (() -> intake.enableBothIntakes(1.0) ),
//                            new InstantCommand(() -> setPathState(2))
//                    );
//
//                }
//                telemetry.addData("Done with pathstate 1", true);
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    // Shoot that first row of artifacts
//                   new SequentialCommandGroup(
//                            new InstantCommand(() -> intake.stopBothIntakes() ),
//                            new InstantCommand(() -> follower.followPath(paths.red9Plus3ShootFirstRow, true) ),
//                            new InstantCommand (() -> shooter.shoot(limelight.getMotifPattern()) )
//                    );
//                    follower.followPath(paths.red9Plus3ShootFirstRow, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    // Align itself and pick the second row of Artifacts
//                    new ParallelCommandGroup(
//                            new InstantCommand(() -> follower.followPath(paths.red9Plus3PickSecondRow, true) ),
//                            new InstantCommand (() -> intake.enableBothIntakes(1.0) )
//                    );
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    // Shoot that second row of Artifacts
//                    new SequentialCommandGroup(
//                            new InstantCommand(() -> intake.stopBothIntakes() ),
//                            new InstantCommand(() -> follower.followPath(paths.red9Plus3ShootSecondRow, true) ),
//                            new InstantCommand (() -> shooter.shoot(limelight.getMotifPattern()) )
//                    );
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (!follower.isBusy()) {
//                    // Align itself and pick up the third row of Artifacts
//                    new ParallelCommandGroup(
//                        new InstantCommand(() -> follower.followPath(paths.red9Plus3PickThirdRow, true) ),
//                        new InstantCommand (() -> intake.enableBothIntakes(1.0) )
//                    );
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if (!follower.isBusy()) {
//                    // Go to its end position
//                    intake.stopBothIntakes();
//                    follower.followPath(paths.red9Plus3End, true);
//                    setPathState(7);
//                }
//                break;
        }
    }



    // This method will change the number of the target path
    private void setPathState(int number) {
        pathState = number;
        pathTimer.resetTimer();
        telemetry.addData("Current path state to be followed", number);
    }
}

class Draw {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 2.0
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.0
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}

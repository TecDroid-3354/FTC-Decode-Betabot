package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.auto.Visualizer.Draw;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.systems.shooterSystem.ShooterSystem;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Autonomous (name="Blue 9", group = "Blue")
public class Blue9 extends CommandOpMode {

    //Pedro ´Pathing
    public static Follower follower;
    public static Paths paths;

    //Variables
    private int pathState;
    private Timer pathTimer;


    // Subsystem Instances
    private ShooterSystem shooterSystem;

    private Intake intake;
    public Limelight limelight;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();

        follower.setStartingPose(paths.blue9Plus3Poses.blue9Plus3StartPose);

    }

    @Override
    public void runOpMode(){
        initialize();

        waitForStart();

        setPathState(0);

        while (!isStopRequested() && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run();

            // Actual path following
            follower.update();
            autonomousPathUpdates();
            drawCurrent();

            // Updating the telemetry
            telemetry.addData("current pathState", pathState);
            telemetry.update();
        }
    }

    private void autonomousPathUpdates(){
        switch (pathState){
            case 0:
                follower.followPath(paths.blue9Plus3ShootPrecharged,true);
                new InstantCommand(() -> setPathState(1));
                break;

            case 1:
                if(!follower.isBusy()){
                    follower.breakFollowing();

                    shootOut();

                    new InstantCommand(() -> setPathState(2));

                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intake.enableBothIntakes().schedule();
                    follower.followPath(paths.blue9Plus3PickFirstRow,true);

                    new InstantCommand(() -> setPathState(3));
                }
                break;

            case 3:
                if (!follower.isBusy()){
                    intake.stopBothIntakes().schedule();

                    follower.followPath(paths.blue9Plus3ShootFirstRow, true);

                    new InstantCommand(() -> setPathState(4));
                }

                break;

            case 4:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    shootOut();
                    new InstantCommand(() -> setPathState(5));
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    intake.enableBothIntakes().schedule();

                    follower.followPath(paths.blue9Plus3PickSecondRow, true);

                    new InstantCommand(() -> setPathState(6));
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    intake.stopBothIntakes().schedule();

                    follower.followPath(paths.blue9Plus3ShootSecondRow, true);

                    new InstantCommand(() -> setPathState(7));
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    shootOut();
                    new InstantCommand(() -> setPathState(8));
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    intake.enableBothIntakes().schedule();
                    follower.followPath(paths.blue9Plus3PickThirdRow, true);
                    new InstantCommand(() -> setPathState(9));
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    intake.stopBothIntakes().schedule();

                    follower.followPath(paths.blue9Plus3ShootThirdRow,true);

                    new InstantCommand(() -> setPathState(10));
                }
                break;
            case 10:
                follower.breakFollowing();

                shootOut();
                new InstantCommand(() -> setPathState(11));
                break;
            case 11:
                // follower.followPath(paths.blue9plus3End); // In the visualizer it shows that this is the final path.
                break;
        }
    }

    private void setPathState(int number) {
        pathState = number;
        pathTimer.resetTimer();
        telemetry.addData("Current path state to be followed", number);
    }

    private static void drawCurrent() {
        try {
            Draw.drawRobot(follower.getPose());
            Draw.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    private void shootOut(){
        //Activates rollers and feed the shooter with the correct Motif Pattern.
        shooterSystem.shoot(limelight.getMotifPattern()).schedule();
    }
}
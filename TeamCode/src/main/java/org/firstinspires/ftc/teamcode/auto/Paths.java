package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


// We will declare all paths in this class
public class Paths {

    // The flow this follows goes:
    // Pose object --> Path object --> PathChain object

    /* Pose objects are declared here
       A pose object represents a point related to the center of the robot, in the field.
       From here, we join several Poses together to form a Path object
       Then, after joining several Paths together, we form the PathChain object
     */
    private final Pose startPose = new Pose(15, 112, Math.toRadians(90));
    private final Pose shootingPose = new Pose(50, 110, Math.toRadians(145));

    /* A PathChain object basically represents a transition between points */
    public PathChain testPath1;

    // ! Actual paths !

    // In this case, the constructor will only take the Follower, which will allow us to build the paths
    public Paths(Follower follower) {
        // A PathChain is set. We can add multiple paths with the .addPath() method, but we will declare
        // each path individually through one variable to avoid confusion later on
        testPath1 = follower.pathBuilder()
            // .addPath() adds a path or curve to the chain
            .addPath(

                /* This "new BezierCurve" is a Path object, simply declared within the .addPath method
                   There are two types of Path objects:

                   1. BezierCurves (take 3 or more points). The first and last ones are the defined
                      points. The ones in the middle are the control points, which define the curve
                      between the start & end points
                   2. BezierLines (take 2 points). The first and last ones are the defined points,
                      basically a straight line
                 */
                new BezierCurve(
                    // This "new Pose" is a Pose object, simply declared within the BezierCurve
                    new Pose(14.014, 114.144),
                    new Pose(78.804, 84.694),
                    new Pose(49.200, 36.000)
                )
            )
            // .setLinearHeadingInterpolation(start, end) basically sets the heading
            // of the robot during the path. @start is the direction it'll start the path facing to
            // & @end is the direction it'll end the path with

            /* Interpolation
               Interpolation in PathPlanner represents the heading/rotation of the robot during/after a path

               We have 3 main types of interpolation:
               1. Linear Heading Interpolation: Basically makes the robot turn from a given startHeading
                  to a given endHeading, for a duration given through endTime.
               2. Constant Heading Interpolation: The robot's heading stays the same throughout the whole path
               3. Tangent Heading Interpolation:

               All measurements should be given in radians!
            */
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .build(); // Converts from pathBuilder to pathChain


        // ! Actual paths
    }
}

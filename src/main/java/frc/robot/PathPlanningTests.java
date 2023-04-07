package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;

public class PathPlanningTests {
    public void foo() {
        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("test",
                new PathConstraints(4, 3));

        // This trajectory can then be passed to a path follower such as a
        // PPSwerveControllerCommand
        // Or the path can be sampled at a given point in time for custom path following

        // Sample the state of the path at 1.2 seconds
        PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);
        DifferentialDriveKinematics k = new DifferentialDriveKinematics(0.44);
        DifferentialDriveKinematicsConstraint kc = new DifferentialDriveKinematicsConstraint(k, 4);
        
        Pose2d p = new Pose2d();

        // Print the velocity at the sampled time
        System.out.println(exampleState.velocityMetersPerSecond);
    }
}

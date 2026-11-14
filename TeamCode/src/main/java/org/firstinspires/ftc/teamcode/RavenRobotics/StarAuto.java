package org.firstinspires.ftc.teamcode.RavenRobotics;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "StarPath", group = "Autonomous")
public class StarAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(23.8294, 23.2363, 90);
    private final Pose path1 = poseFactory.of(61.9046, 45.2787, 180);
    private final Pose point2 = poseFactory.of(70.1908, 58.8569, -121.3937);
    private final Pose point3 = poseFactory.of(79.3331, 45.5589, 124.5083);
    private final Pose point4 = poseFactory.of(94.5917, 45.5849, -179.9021);
    private final Pose point5 = poseFactory.of(86.1013, 33.2042, 55.559);
    private final Pose point6 = poseFactory.of(85.4061, 16.699, 87.588);
    private final Pose point7 = poseFactory.of(71.19, 25.8994, -32.9102);
    private final Pose point8 = poseFactory.of(55.544, 17.2057, 29.0588);
    private final Pose point9 = poseFactory.of(55.0007, 32.3681, -87.9482);
    private final Pose point10 = poseFactory.of(45.3547, 44.994, -52.6206);
    private final Pose point11 = poseFactory.of(61.8838, 45.3614, -178.7268);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8()),
                follow(follower, path9()),
                follow(follower, path10()),
                follow(follower, path11())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(start, path1).linear(0,0);
    }

    public Path path2() {
        return line(path1, point2).linear(0,0);
    }

    public Path path3() {
        return line(point2, point3).linear(0,0);
    }

    public Path path4() {
        return line(point3, point4).linear(0,0);
    }

    public Path path5() {
        return line(point4, point5).linear(0,0);
    }

    public Path path6() {
        return line(point5, point6).linear(0,0);
    }

    public Path path7() {
        return line(point6, point7).linear(0,0);
    }

    public Path path8() {
        return line(point7, point8).linear(0,0);
    }

    public Path path9() {
        return line(point8, point9).linear(0,0);
    }

    public Path path10() {
        return line(point9, point10).linear(0,0);
    }

    public Path path11() {
        return line(point10, point11).linear(0,0);
    }
}

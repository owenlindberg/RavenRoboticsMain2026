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

@Autonomous(name = "OStarAuto", group = "Autonomous")
public class OStarAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(23.5246, 47.0127, 90);
    private final Pose path1 = poseFactory.of(37.8644, 60.0402, 0);
    private final Pose point2 = poseFactory.of(13.8308, 92.4896, 0);
    private final Pose point3 = poseFactory.of(48.5022, 98.3093, 0);
    private final Pose point4 = poseFactory.of(70.7288, 133.0469, 0);
    private final Pose point5 = poseFactory.of(92.2578, 97.9374, 0);
    private final Pose point6 = poseFactory.of(129.4076, 91.9717, 0);
    private final Pose point7 = poseFactory.of(103.4694, 59.4829, 51.3971);
    private final Pose point8 = poseFactory.of(105.4694, 22.3204, 93.0806);
    private final Pose point9 = poseFactory.of(71.266, 35.7355, -21.4158);
    private final Pose point10 = poseFactory.of(32.6863, 24.2907, 16.5232);
    private final Pose point11 = poseFactory.of(37.7362, 59.3808, -98.1894);

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
        return line(start, path1).constant(0);
    }

    public Path path2() {
        return line(path1, point2).constant(0);
    }

    public Path path3() {
        return line(point2, point3).constant(0);
    }

    public Path path4() {
        return line(point3, point4).constant(0);
    }

    public Path path5() {
        return line(point4, point5).constant(0);
    }

    public Path path6() {
        return line(point5, point6).constant(0);
    }

    public Path path7() {
        return line(point6, point7).constant(0);
    }

    public Path path8() {
        return line(point7, point8).constant(0);
    }

    public Path path9() {
        return line(point8, point9).constant(0);
    }

    public Path path10() {
        return line(point9, point10).constant(0);
    }

    public Path path11() {
        return line(point10, point11).constant(0);
    }
}



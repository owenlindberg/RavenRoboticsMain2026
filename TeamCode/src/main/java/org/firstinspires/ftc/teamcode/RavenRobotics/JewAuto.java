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

@Autonomous(name = "JewAuto", group = "Autonomous")
public class JewAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(78.1423, 12.4285, 90);
    private final Pose path1 = poseFactory.of(66.1557, 45.6401, 180);
    private final Pose point2 = poseFactory.of(50.0216, 9.2258, 66.1033);
    private final Pose point3 = poseFactory.of(78.3063, 11.0902, -176.2288);
    private final Pose point4 = poseFactory.of(65.5507, 3.1244, 31.9843);
    private final Pose point5 = poseFactory.of(49.2653, 34.2928, -62.4131);
    private final Pose point6 = poseFactory.of(82.4903, 33.573, 178.7589);
    private final Pose point7 = poseFactory.of(66.0887, 3.6416, 61.2784);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
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
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return line(point4, point5).reverseTangent();
    }

    public Path path6() {
        return line(point5, point6).reverseTangent();
    }

    public Path path7() {
        return line(point6, point7).reverseTangent();
    }
}



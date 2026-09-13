package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.through;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.integration.Headings;

import java.util.ArrayList;
import java.util.List;

/**
 * Pedro Pathing 3 port of the ExodusDC RedClose24BallAuto - PATHING ONLY. Ported from ExodusDCv3.
 *
 * Standalone showcase: no subsystems, no turret, no intake. The drivetrain drives the same 15 legs the
 * RoadRunner auto drives, back to back with no pauses.
 *
 * Geometry: the RoadRunner builder was replayed off-robot (RoadRunner core 1.0.1) to get the exact start
 * and end pose of every leg. Legs made of several collinear lineTo calls became one Pedro line with a
 * linear heading sweep; the two gate splines became quadratic Beziers through their arc-length midpoint.
 *
 * Coordinates: the RoadRunner/FTC field frame (origin at field centre) is used as-is; Pedro does not care where
 * the origin is, and this keeps the numbers identical to the RoadRunner auto and to the full-robot port.
 *
 * Heading sweeps use {@link Headings#linear} instead of Pedro's {@code .linear(...)}, which runs backwards on
 * straight lines in 3.0.0 (see Headings).
 */
@Autonomous(name = "Pedro RedClose24Ball (pathing demo)", group = "RedAuto")
public class PedroRedClose24BallAuto extends OpMode {

    private static final double OFFSET = 0.0;

    private Follower follower;
    private final List<Path> legs = new ArrayList<>();
    private final List<String> legNames = new ArrayList<>();
    private int legIndex = -1;
    private final ElapsedTime autoTimer = new ElapsedTime();

    /** RoadRunner red-alliance pose (inches, degrees) -> Pedro pose (inches, radians). */
    private static Pose p(double x, double y, double headingDeg) {
        return new Pose(x + OFFSET, y + OFFSET, Math.toRadians(headingDeg));
    }

    private void leg(String name, Path path) {
        legNames.add(name);
        legs.add(path);
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        // --- Poses (RoadRunner red-alliance numbers; see RedClose24BallAuto) ----------------------------
        Pose start = p(-41.127, 54.289, 0);

        Pose shoot1 = p(-31.176, 42.000, 13);
        Pose grab1 = p(-15.000, 45.734, 13);
        Pose shoot2 = p(-28.000, 32.734, 18);
        Pose grab2 = p(9.000, 44.756, 18);
        Pose shoot3 = p(-16.756, 19.000, 58);
        Pose grab3 = p(6.364, 56.000, 90);
        Pose gate1Mid = p(12.280, 55.978, 0);          // spline midpoint (heading unused)
        Pose gate1 = p(17.000, 59.000, 130);
        Pose gateShift1 = p(15.000, 60.000, 65);
        Pose shoot4 = p(-13.709, 19.000, 55);
        Pose grab4 = p(34.789, 47.000, 30);
        Pose shoot5 = p(-13.709, 19.000, 30);
        Pose grab5 = p(5.965, 56.000, 90);
        Pose gate2Mid = p(12.670, 56.021, 0);
        Pose gate2 = p(18.000, 59.500, 135);
        Pose gateShift2 = p(15.000, 60.000, 65);
        Pose shoot6 = p(57.490, 16.000, 138);

        // --- Legs, in the order the RoadRunner SequentialAction ran them --------------------------------
        leg("shoot1", line(start, shoot1).heading(Headings.linear(start, shoot1)));
        leg("grab1", line(shoot1, grab1).constant(grab1));
        leg("shoot2", line(grab1, shoot2).heading(Headings.linear(grab1, shoot2)));
        leg("grab2", line(shoot2, grab2).constant(grab2));
        leg("shoot3", line(grab2, shoot3).heading(Headings.linear(grab2, shoot3)));
        leg("grab3", line(shoot3, grab3).heading(Headings.linear(shoot3, grab3)));
        leg("gateStrafe1", through(grab3, gate1Mid, gate1).linear(grab3, gate1));
        leg("gateShift1", line(gate1, gateShift1).heading(Headings.linear(gate1, gateShift1)));
        leg("shoot4", line(gateShift1, shoot4).heading(Headings.linear(gateShift1, shoot4)));
        leg("grab4", line(shoot4, grab4).heading(Headings.linear(shoot4, grab4)));
        leg("shoot5", line(grab4, shoot5).constant(shoot5));
        leg("grab5", line(shoot5, grab5).heading(Headings.linear(shoot5, grab5)));
        leg("gateStrafe2", through(grab5, gate2Mid, gate2).linear(grab5, gate2));
        leg("gateShift2", line(gate2, gateShift2).heading(Headings.linear(gate2, gateShift2)));
        leg("shoot6", line(gateShift2, shoot6).heading(Headings.linear(gateShift2, shoot6)));
        // The original then loops FarCycleGenerator cycles; not part of this pathing demo.

        follower.setPose(start);
        telemetry.addData("Pedro", "ready, %d legs", legs.size());
    }

    @Override
    public void init_loop() {
        follower.update();
        telemetry.addData("pose", follower.pose());
    }

    @Override
    public void start() {
        autoTimer.reset();
        legIndex = 0;
        follower.follow(legs.get(0));
    }

    @Override
    public void loop() {
        follower.update();

        // Advance to the next leg as soon as the follower reports the current one done (end constraints in
        // Constants.foresightConfig decide how close is "done"). No pauses between legs.
        if (legIndex >= 0 && legIndex < legs.size() && !follower.isBusy()) {
            legIndex++;
            if (legIndex < legs.size()) {
                follower.follow(legs.get(legIndex));
            }
            // else: follower keeps holding the last pose (holdEnd defaults to true)
        }

        telemetry.addData("t", "%.1f s", autoTimer.seconds());
        telemetry.addData("leg", legIndex < legs.size() ? (legIndex + 1) + "/" + legs.size() + " " + legNames.get(legIndex) : "done");
        telemetry.addData("state", follower.mode());
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("pose", follower.pose());
        telemetry.addData("remaining", "%.1f in", follower.remainingDistance());
    }

    @Override
    public void stop() {
        follower.stop();
    }
}

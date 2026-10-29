package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Autonomo1", group = "Autonomous")
public class Autonomo1 extends OpMode {

    private Follower follower;
    private DcMotor intake;
    private int pathState = 0;

    // ===== ESPERAS (milisegundos), antes de salir de cada punto =====
    private static final int WAIT_INICIO_A_1 = 0;
    private static final int WAIT_1_A_2      = 1500;
    private static final int WAIT_2_A_3      = 2000;
    private static final int WAIT_3_A_4      = 2500;
    private static final int WAIT_4_A_5      = 1500;
    private static final int WAIT_5_A_6      = 1500;

    private final ElapsedTime waitTimer = new ElapsedTime();
    private boolean waiting = false;

    // Fábrica de poses en GRADOS
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // POSICIONES EN LA CANCHA
    // OJO: el heading de startPose = hacia dónde pones el robot físicamente
    private final Pose startPose = poseFactory.of(56.3845, 10.8838, 90);
    private final Pose point1 = poseFactory.of(12.5868, 47.7708, 180);
    private final Pose point2 = poseFactory.of(28.9356, 46.6166, 175.9616);
    private final Pose point3 = poseFactory.of(44.1696, 127.6855, 80);
    private final Pose point4 = poseFactory.of(24.2915, 9.3832, 180);
    private final Pose point5 = poseFactory.of(11.1654, 9.7274, 180);
    private final Pose point6 = poseFactory.of(24.5211, 132.6298, -96.2019);
    private final Pose point7 = poseFactory.of(9.0122, 132.3628, 270);
    private final Pose point8 = poseFactory.of(8.892, 121.3016, 270);

    private Path startToPoint1() {
        return line(startPose, point1).constant(point1);
    }

    private Path point1ToPoint2() {
        return line(point1, point2).reverseTangent();
    }

    private Path point2ToPoint3() {
        return line(point2, point3).constant(point3);
    }

    private Path point3ToPoint4() {
        return line(point3, point4).constant(point4);
    }

    private Path point4ToPoint5() {
        return line(point4, point5).constant(point5);
    }

    private Path point5ToPoint6() {
        return line(point5, point6).reverseTangent();
    }

    private Path point6ToPoint7() {
        return line(point6, point7).constant(point7);
    }

    private Path point7ToPoint8() {
        return line(point7, point8).constant(point8);
    }

    // Devuelve true cuando ya pasó el tiempo de espera (se llama en cada vuelta)
    private boolean esperar(int ms) {
        if (!waiting) {
            waitTimer.reset();
            waiting = true;
        }
        if (waitTimer.milliseconds() >= ms) {
            waiting = false;
            return true;
        }
        return false;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        // El nombre debe coincidir con el de tu configuración en el Driver Hub
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();   // SIEMPRE se llama en cada vuelta
        updateState();

        telemetry.addData("Estado", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Modo", follower.mode());
        telemetry.addData("Path completion", follower.completion());
        telemetry.addData("Distance remaining", follower.remainingDistance());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0);   // el intake se apaga si presionas Stop
    }

    private void updateState() {
        switch (pathState) {
            case 0:
                // Espera en el inicio, luego sale hacia el punto 1
                if (esperar(WAIT_INICIO_A_1)) {
                    follower.follow(startToPoint1());
                    pathState = 1;
                }
                break;

            case 1:
                if (follower.atParametricEnd() && esperar(WAIT_1_A_2)) {
                    intake.setPower(0);
                    follower.follow(point1ToPoint2());
                    pathState = 2;
                }
                break;

            case 2:
                // Llegó al punto 2, espera 2 segundos, luego enciende el intake y sale
                if (follower.atParametricEnd() && esperar(WAIT_2_A_3)) {
                    intake.setPower(1.0);
                    follower.follow(point2ToPoint3());
                    pathState = 3;
                }
                break;

            case 3:
                if (follower.atParametricEnd() && esperar(WAIT_3_A_4)) {
                    follower.follow(point3ToPoint4());
                    pathState = 4;
                }
                break;

            case 4:
                // Al llegar al punto 4 se enciende el intake
                if (follower.atParametricEnd()) {
                    intake.setPower(1.0);
                }
                if (follower.atParametricEnd() && esperar(WAIT_4_A_5)) {
                    follower.follow(point4ToPoint5());
                    pathState = 5;
                }
                break;

            case 5:
                // Al llegar al punto 5 se apaga el intake
                if (follower.atParametricEnd()) {
                    intake.setPower(0);
                }
                if (follower.atParametricEnd() && esperar(WAIT_5_A_6)) {
                    follower.follow(point5ToPoint6());
                    pathState = 6;
                }
                break;

            case 6:
                if (follower.atParametricEnd()) {
                    follower.follow(point6ToPoint7());
                    pathState = 7;
                }
                break;

            case 7:
                if (follower.atParametricEnd()) {
                    follower.follow(point7ToPoint8());
                    pathState = 8;
                }
                break;

            case 8:
                if (follower.atParametricEnd()) {
                    pathState = -1;   // terminado
                }
                break;
        }
    }
}
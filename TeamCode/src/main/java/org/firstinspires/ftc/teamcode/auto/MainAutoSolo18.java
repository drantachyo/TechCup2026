package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.GlobalState;
import org.firstinspires.ftc.teamcode.robot.utils.MirrorTool;

@Configurable
@Autonomous(name = "🚀 Main Auto Solo 18", group = "Autonomous")
public class MainAutoSolo18 extends OpMode {

    private Robot robot;
    private Follower follower;

    private int pathState;
    private ElapsedTime pathTimer;

    // ==========================================
    // 🗺️ СТРУКТУРА КООРДИНАТ (Только КРАСНЫЕ)
    // ==========================================
    private final Pose targetPose = new Pose(144, 144, 0);
    private final Pose startPose = new Pose(118.5, 125.83, 0.7469);
    private final Pose scorePose = new Pose(88.6, 79.63, -0.51);

    private final Pose NearBalls = new Pose(127, 83.25, 0);
    private final Pose MiddleBalls = new Pose(133, 59.37, 0);

    // 🔥 Координаты Гейтов: Разделили на ОТКРЫТИЕ и ЗАБОР
    private final Pose gateOpen1 = new Pose(133.5, 58.5, 0.5435);
    private final Pose gatePickup1 = new Pose(133.5, 50.0, 0.5435); // ПОДБЕРИ Y: "Чуть правее" для сбора

    private final Pose gateOpen2 = new Pose(133.5, 58.5, 0.5435); // Если нужно, настрой отдельно
    private final Pose gatePickup2 = new Pose(133.5, 50.0, 0.5435);

    private final Pose FourthBall = new Pose(133, 39, 0);
    private final Pose endPose = new Pose(100, 45, 0);

    public static Pose controlPoint1 = new Pose(90, 59);
    public static Pose controlPoint2 = new Pose(100, 59);
    public static Pose controlPointGate2 = new Pose(100, 59);
    public static Pose controlPoint3 = new Pose(85, 37);

    // ==========================================
    // ⏱️ НАСТРОЙКИ
    // ==========================================
    public static double shootTime = 0.5; // Время стояния у корзины для выплевывания
    public static double preFireDistance = 15.0; // За сколько дюймов до корзины открывать стоппер
    public static double pathTimeout = 5.0;

    public static double gateWaitTime = 1.0; // Сколько стоим и открываем гейт
    public static double gatePickupTimeout = 3.0; // Макс. время интейка у гейта
    public static double jamTpsThreshold = 1500;

    // Пути
    private PathChain scorePreload, park;
    private PathChain grabBall1, scoreBall1;
    private PathChain grabBall2, scoreBall2;
    private PathChain driveGateOpen1, driveGatePickup1, scoreBall3;
    private PathChain driveGateOpen2, driveGatePickup2, scoreBall4;
    private PathChain grabBall5, scoreBall5;

    private Pose getPose(Pose redPose) {
        return GlobalState.isBlueAlliance ? MirrorTool.toBlue(redPose) : redPose;
    }

    public void buildPaths() {
        Pose mStart = getPose(startPose), mScore = getPose(scorePose);
        Pose mNear = getPose(NearBalls), mMid1 = getPose(MiddleBalls);
        Pose mGateOpen1 = getPose(gateOpen1), mGatePickup1 = getPose(gatePickup1);
        Pose mGateOpen2 = getPose(gateOpen2), mGatePickup2 = getPose(gatePickup2);
        Pose mFourth = getPose(FourthBall), mEnd = getPose(endPose);

        scorePreload = follower.pathBuilder().addPath(new BezierLine(mStart, mScore)).setLinearHeadingInterpolation(mStart.getHeading(), mScore.getHeading()).build();

        // Линия 1
        grabBall1 = follower.pathBuilder().addPath(new BezierLine(mScore, mNear)).setTangentHeadingInterpolation().build();
        scoreBall1 = follower.pathBuilder().addPath(new BezierLine(mNear, mScore)).setTangentHeadingInterpolation().setReversed().build();

        // Линия 2
        grabBall2 = follower.pathBuilder().addPath(new BezierCurve(mScore, getPose(controlPoint1), mMid1)).setTangentHeadingInterpolation().build();
        scoreBall2 = follower.pathBuilder().addPath(new BezierLine(mMid1, mScore)).setTangentHeadingInterpolation().setReversed().build();

        // 🔥 ГЕЙТ 1 (2 движения: Открыть -> Сместиться)
        driveGateOpen1 = follower.pathBuilder().addPath(new BezierCurve(mScore, getPose(controlPoint2), mGateOpen1)).setLinearHeadingInterpolation(mScore.getHeading(), mGateOpen1.getHeading()).build();
        driveGatePickup1 = follower.pathBuilder().addPath(new BezierLine(mGateOpen1, mGatePickup1)).setLinearHeadingInterpolation(mGateOpen1.getHeading(), mGatePickup1.getHeading()).build();
        scoreBall3 = follower.pathBuilder().addPath(new BezierLine(mGatePickup1, mScore)).setTangentHeadingInterpolation().setReversed().build();

        // 🔥 ГЕЙТ 2 (2 движения)
        driveGateOpen2 = follower.pathBuilder().addPath(new BezierCurve(mScore, getPose(controlPointGate2), mGateOpen2)).setLinearHeadingInterpolation(mScore.getHeading(), mGateOpen2.getHeading()).build();
        driveGatePickup2 = follower.pathBuilder().addPath(new BezierLine(mGateOpen2, mGatePickup2)).setLinearHeadingInterpolation(mGateOpen2.getHeading(), mGatePickup2.getHeading()).build();
        scoreBall4 = follower.pathBuilder().addPath(new BezierLine(mGatePickup2, mScore)).setTangentHeadingInterpolation().setReversed().build();

        // Линия 3
        grabBall5 = follower.pathBuilder().addPath(new BezierCurve(mScore, getPose(controlPoint3), mFourth)).setTangentHeadingInterpolation().build();
        scoreBall5 = follower.pathBuilder().addPath(new BezierLine(mFourth, mScore)).setTangentHeadingInterpolation().setReversed().build();

        park = follower.pathBuilder().addPath(new BezierLine(mScore, mEnd)).setTangentHeadingInterpolation().build();
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        robot = new Robot();
        robot.init(hardwareMap, getPose(startPose));
        follower = robot.drive.getFollower();
        buildPaths();
    }

    @Override
    public void start() {
        robot.turret.on();
        robot.shooter.turnOn(); // 🔥 ВСЕГДА ВКЛЮЧЕН с самого старта
        setPathState(10);
    }

    @Override
    public void loop() {
        robot.periodic();
        Pose currentPose = follower.getPose();
        Pose actualTarget = getPose(targetPose);
        Pose actualScorePose = getPose(scorePose);

        // Динамика башни и статика маховика
        if (pathState < 999) {
            double dynamicDist = Math.hypot(actualTarget.getX() - currentPose.getX(), actualTarget.getY() - currentPose.getY());
            robot.turret.face(actualTarget, currentPose);
            robot.hood.setDistance(dynamicDist);

            double staticDist = Math.hypot(actualTarget.getX() - actualScorePose.getX(), actualTarget.getY() - actualScorePose.getY());
            robot.shooter.setDistance(staticDist);
        } else {
            robot.turret.setYaw(0);
        }

        autonomousPathUpdate(currentPose, actualScorePose);
    }

    // ==========================================
    // 🧠 ЧИСТЫЙ STATE MACHINE
    // ==========================================
    public void autonomousPathUpdate(Pose currentPose, Pose actualScorePose) {
        // Дистанция до точки скоринга (для пре-файра)
        double distToScore = Math.hypot(currentPose.getX() - actualScorePose.getX(), currentPose.getY() - actualScorePose.getY());
        boolean isPathDone = !follower.isBusy() || pathTimer.seconds() > pathTimeout;

        switch (pathState) {
            case 10: // Старт с прелоудом
                robot.stopper.close();
                robot.intake.setPower(0);
                follower.followPath(scorePreload, true);
                setPathState(11);
                break;

            case 11: // Забиваем прелоуд (Pre-fire логика)
                if (distToScore < preFireDistance) {
                    robot.stopper.open();
                    robot.intake.setPower(1.0); // Закидываем мяч в шутер ДО приезда
                }

                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > shootTime) { // Ждем shootTime после остановки
                        robot.stopper.close(); // Закрываем серво
                        follower.followPath(grabBall1, true);
                        setPathState(20);
                    }
                } else {
                    pathTimer.reset(); // Пока едем — таймер держится на нуле
                }
                break;

            // 🔥 СБОР 1: ЛИНИЯ
            case 20:
                robot.intake.setPower(1.0); // Сразу включаем интейк!
                if (isPathDone) {
                    follower.followPath(scoreBall1, true);
                    setPathState(21);
                }
                break;
            case 21: // Забиваем Мяч 1
                if (distToScore < preFireDistance) { robot.stopper.open(); robot.intake.setPower(1.0); }
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > shootTime) {
                        robot.stopper.close();
                        follower.followPath(grabBall2, true);
                        setPathState(30);
                    }
                } else { pathTimer.reset(); }
                break;

            // 🔥 СБОР 2: ЛИНИЯ
            case 30:
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    follower.followPath(scoreBall2, true);
                    setPathState(31);
                }
                break;
            case 31: // Забиваем Мяч 2
                if (distToScore < preFireDistance) { robot.stopper.open(); robot.intake.setPower(1.0); }
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > shootTime) {
                        robot.stopper.close();
                        follower.followPath(driveGateOpen1, true);
                        setPathState(40);
                    }
                } else { pathTimer.reset(); }
                break;

            // ==========================================
            // 🔥 СБОР 3: ГЕЙТ 1 (Два движения)
            // ==========================================
            case 40: // Едем открывать гейт
                robot.intake.setPower(0); // Оффнули интейк
                if (isPathDone) setPathState(41);
                break;
            case 41: // Стоим 1 секунду
                if (pathTimer.seconds() > gateWaitTime) {
                    follower.followPath(driveGatePickup1, true); // Смещаемся
                    setPathState(42);
                }
                break;
            case 42: // Включаем интейк и чекаем TPS
                robot.intake.setPower(1.0);
                boolean isJammed = (pathTimer.seconds() > 0.5) && (Math.abs(robot.intake.getVelocity()) < jamTpsThreshold);

                if (isJammed || pathTimer.seconds() > gatePickupTimeout) {
                    follower.breakFollowing();
                    follower.followPath(scoreBall3, true);
                    setPathState(43);
                }
                break;
            case 43: // Забиваем Мяч 3
                if (distToScore < preFireDistance) { robot.stopper.open(); robot.intake.setPower(1.0); }
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > shootTime) {
                        robot.stopper.close();
                        follower.followPath(driveGateOpen2, true);
                        setPathState(50);
                    }
                } else { pathTimer.reset(); }
                break;

            // ==========================================
            // 🔥 СБОР 4: ГЕЙТ 2 (Два движения)
            // ==========================================
            case 50:
                robot.intake.setPower(0);
                if (isPathDone) setPathState(51);
                break;
            case 51:
                if (pathTimer.seconds() > gateWaitTime) {
                    follower.followPath(driveGatePickup2, true);
                    setPathState(52);
                }
                break;
            case 52:
                robot.intake.setPower(1.0);
                boolean isJammed2 = (pathTimer.seconds() > 0.5) && (Math.abs(robot.intake.getVelocity()) < jamTpsThreshold);

                if (isJammed2 || pathTimer.seconds() > gatePickupTimeout) {
                    follower.breakFollowing();
                    follower.followPath(scoreBall4, true);
                    setPathState(53);
                }
                break;
            case 53: // Забиваем Мяч 4
                if (distToScore < preFireDistance) { robot.stopper.open(); robot.intake.setPower(1.0); }
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > shootTime) {
                        robot.stopper.close();
                        follower.followPath(grabBall5, true);
                        setPathState(60);
                    }
                } else { pathTimer.reset(); }
                break;

            // 🔥 СБОР 5: ЛИНИЯ
            case 60:
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    follower.followPath(scoreBall5, true);
                    setPathState(61);
                }
                break;
            case 61: // Забиваем Мяч 5
                if (distToScore < preFireDistance) { robot.stopper.open(); robot.intake.setPower(1.0); }
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > shootTime) {
                        robot.stopper.close();
                        robot.intake.setPower(0); // Выключаем интейк
                        follower.followPath(park, true);
                        setPathState(999);
                    }
                } else { pathTimer.reset(); }
                break;

            case 999:
                robot.shooter.turnOff();
                if (isPathDone) requestOpModeStop();
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    public void stop() {
        GlobalState.currentPose = follower.getPose();
        GlobalState.lastAutoPose = follower.getPose();
        GlobalState.isAutoBeen = true;
    }
}
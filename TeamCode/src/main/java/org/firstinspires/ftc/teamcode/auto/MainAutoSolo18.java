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
    private ElapsedTime firingTimer;
    private ElapsedTime stopperTimer;
    private ElapsedTime intakeTimer;

    // ==========================================
    // 🗺️ СТРУКТУРА КООРДИНАТ (Пишем только КРАСНЫЕ!)
    // ==========================================
    private final Pose targetPose = new Pose(144, 144, 0);

    private final Pose startPose = new Pose(118.5, 125.83, 0.7469);
    private final Pose scorePose = new Pose(88.6, 79.63, -0.51);

    private final Pose NearBalls = new Pose(127, 83.25, 0);
    private final Pose MiddleBalls = new Pose(133, 59.37, 0);

    // 🔥 Координаты Гейтов (Едем прямо сюда и стоим)
    private final Pose gateBalls = new Pose(133.5, 58.5, 0.5435);
    private final Pose gateBalls2 = new Pose(133.5, 58.5, 0.5435);

    private final Pose FourthBall = new Pose(133, 39, 0);
    private final Pose endPose = new Pose(100, 45, 0);

    // Контрольные точки
    public static Pose controlPoint1 = new Pose(90, 59); // Для Middle
    public static Pose controlPoint2 = new Pose(100, 59); // Для Gate 1
    public static Pose controlPointGate2 = new Pose(100, 59); // Для Gate 2
    public static Pose controlPoint3 = new Pose(85, 37); // Для Fourth

    // ==========================================
    // ⏱️ ТАЙМИНГИ И МОЩНОСТИ
    // ==========================================
    public static double shootTime = 1.0;
    public static double stopperWaitTime = 0.3;

    // Таймаут для защиты от застреваний (в секундах)
    public static double pathTimeout = 5.0;

    // Настройки для умного забора с гейта
    public static double gatePickupTimeout = 3.0;
    public static double jamTpsThreshold = 1500; // TPS интейка!

    private boolean readyToFire = false;
    private boolean arrivedAtPickup = false;
    private boolean isPreloadFired = false;

    // Пути Pedro Pathing
    private PathChain scorePreload;
    private PathChain grabBall1, scoreBall1; // Near
    private PathChain grabBall2, scoreBall2; // Middle
    private PathChain grabBall3, scoreBall3; // Gate 1
    private PathChain grabBall4, scoreBall4; // Gate 2
    private PathChain grabBall5, scoreBall5; // Fourth
    private PathChain park;

    // ==========================================
    // 🪞 АВТО-ЗЕРКАЛО КООРДИНАТ
    // ==========================================
    private Pose getPose(Pose redPose) {
        if (GlobalState.isBlueAlliance) {
            return MirrorTool.toBlue(redPose);
        }
        return redPose;
    }

    // ==========================================
    // 🛣️ СТРУКТУРА ПУТЕЙ
    // ==========================================
    public void buildPaths() {
        Pose mStart = getPose(startPose);
        Pose mScore = getPose(scorePose);
        Pose mNear = getPose(NearBalls);
        Pose mMid1 = getPose(MiddleBalls);
        Pose mGate1 = getPose(gateBalls);
        Pose mGate2 = getPose(gateBalls2);
        Pose mFourth = getPose(FourthBall);
        Pose mEnd = getPose(endPose);

        Pose cp1 = getPose(controlPoint1);
        Pose cp2 = getPose(controlPoint2);
        Pose cpGate2 = getPose(controlPointGate2);
        Pose cp3 = getPose(controlPoint3);

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(mStart, mScore))
                .setLinearHeadingInterpolation(mStart.getHeading(), mScore.getHeading())
                .build();

        // 🔥 МЯЧ 1: NearBalls (Ближний)
        grabBall1 = follower.pathBuilder()
                .addPath(new BezierLine(mScore, mNear))
                .setTangentHeadingInterpolation()
                .build();
        scoreBall1 = follower.pathBuilder()
                .addPath(new BezierLine(mNear, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // 🔥 МЯЧ 2: MiddleBalls (Средний)
        grabBall2 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp1, mMid1))
                .setTangentHeadingInterpolation()
                .build();
        scoreBall2 = follower.pathBuilder()
                .addPath(new BezierLine(mMid1, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // 🔥 МЯЧ 3: ЗАЕЗД В ГЕЙТ 1
        grabBall3 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp2, mGate1))
                .setLinearHeadingInterpolation(mScore.getHeading(), mGate1.getHeading())
                .build();
        scoreBall3 = follower.pathBuilder()
                .addPath(new BezierLine(mGate1, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // 🔥 МЯЧ 4: ЗАЕЗД В ГЕЙТ 2
        grabBall4 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cpGate2, mGate2))
                .setLinearHeadingInterpolation(mScore.getHeading(), mGate2.getHeading())
                .build();
        scoreBall4 = follower.pathBuilder()
                .addPath(new BezierLine(mGate2, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // 🔥 МЯЧ 5: FourthBall (Дальний)
        grabBall5 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp3, mFourth))
                .setTangentHeadingInterpolation()
                .build();
        scoreBall5 = follower.pathBuilder()
                .addPath(new BezierLine(mFourth, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(mScore, mEnd))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        firingTimer = new ElapsedTime();
        stopperTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();

        robot = new Robot();
        robot.init(hardwareMap, getPose(startPose));
        follower = robot.drive.getFollower();

        buildPaths();

        telemetry.addLine("Solo Автоном Готов!");
        telemetry.addData("Альянс", GlobalState.isBlueAlliance ? "🟦 СИНИЙ" : "🟥 КРАСНЫЙ");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.turret.on();
        robot.shooter.turnOn();
        setPathState(0);
    }

    @Override
    public void loop() {
        robot.periodic();

        Pose currentPose = follower.getPose();
        Pose actualTarget = getPose(targetPose);

        // ==========================================
        // 🔥 ДИНАМИКА ДЛЯ СЕРВОПРИВОДОВ
        // ==========================================
        if (pathState < 999) {
            // Пока автоном работает — целимся в корзину
            double dynamicDistance = Math.hypot(actualTarget.getX() - currentPose.getX(), actualTarget.getY() - currentPose.getY());
            robot.turret.face(actualTarget, currentPose);
            robot.hood.setDistance(dynamicDistance);
        } else {
            // Как только перешли в парковку (стейт 999) — возвращаем башню в ноль
            robot.turret.setYaw(0);
        }

        // СТАТИКА ДЛЯ МАХОВИКА (Мотор)
        Pose actualScorePose = getPose(scorePose);
        double staticScoreDistance = Math.hypot(actualTarget.getX() - actualScorePose.getX(), actualTarget.getY() - actualScorePose.getY());
        robot.shooter.setDistance(staticScoreDistance);

        autonomousPathUpdate();

        telemetry.addData("Состояние автонома (State)", pathState);
        telemetry.addData("Башня наведена?", robot.turret.getCurrentTicks());
        telemetry.addData("Шутер готов?", robot.shooter.isAtTarget());
        telemetry.update();
    }

    @Override
    public void stop() {
        Pose finalPose = follower.getPose();

        // Сохраняем в обе переменные (на случай, если ты используешь их для разных целей)
        GlobalState.currentPose = finalPose;
        GlobalState.lastAutoPose = finalPose;

        // Ставим флаг, что автоном успешно отработал
        GlobalState.isAutoBeen = true;
    }

    // ==========================================
    // 🧠 КОНЕЧНЫЙ АВТОМАТ (STATE MACHINE)
    // ==========================================
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.stopper.close();
                robot.intake.setPower(0); // 🛑 Интейк ВЫКЛЮЧЕН. Прелоуд внутри, батарея цела.
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: handleFiring(2, grabBall1); break;
            case 2: handleStopperClose(3); break;
            case 3: handleIntakeDelay(4); break;

            // 🔥 Сбор 1: NearBalls
            case 4: handleLinePickup(5, scoreBall1); break;

            case 5: handleFiring(6, grabBall2); break;
            case 6: handleStopperClose(7); break;
            case 7: handleIntakeDelay(8); break;

            // 🔥 Сбор 2: MiddleBalls
            case 8: handleLinePickup(9, scoreBall2); break;

            case 9: handleFiring(10, grabBall3); break;
            case 10: handleStopperClose(11); break;
            case 11: handleIntakeDelay(12); break;

            // 🔥 Сбор 3: Gate 1 (Умный сбор по TPS)
            case 12: handleGatePickup(13, scoreBall3, gatePickupTimeout, jamTpsThreshold); break;

            case 13: handleFiring(14, grabBall4); break;
            case 14: handleStopperClose(15); break;
            case 15: handleIntakeDelay(16); break;

            // 🔥 Сбор 4: Gate 2 (Умный сбор по TPS)
            case 16: handleGatePickup(17, scoreBall4, gatePickupTimeout, jamTpsThreshold); break;

            case 17: handleFiring(18, grabBall5); break;
            case 18: handleStopperClose(19); break;
            case 19: handleIntakeDelay(20); break;

            // 🔥 Сбор 5: FourthBall
            case 20: handleLinePickup(21, scoreBall5); break;

            case 21: handleFiring(22, park); break;
            case 22: handleStopperClose(999); break;

            case 999:
                robot.intake.setPower(0);
                robot.shooter.turnOff();
                if (!follower.isBusy() || pathTimer.seconds() > pathTimeout) {
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    // ==========================================
    // 🛠️ ХЕЛПЕР-МЕТОДЫ (ЛОГИКА ОПЕРАЦИЙ)
    // ==========================================

    private void handleFiring(int nextState, PathChain nextPath) {
        // Как только приехали на точку, запускаем таймер выстрела.
        // Защита pathTimer.seconds() > 0.3 от ложных срабатываний на старте пути.
        if (!follower.isBusy() && !readyToFire && pathTimer.seconds() > 0.3) {
            readyToFire = true;
            firingTimer.reset();
        }

        if (readyToFire) {
            robot.stopper.open();
            robot.intake.setPower(1.0); // Закидываем мяч в шутер

            // Ждем shootTime и едем дальше
            if (firingTimer.seconds() > shootTime) {
                robot.stopper.close();
                robot.intake.setPower(0); // 🛑 ВЫКЛЮЧАЕМ после выстрела!
                follower.followPath(nextPath, true);

                readyToFire = false;
                isPreloadFired = true;

                stopperTimer.reset();
                setPathState(nextState);
            }
        }
    }

    private void handleStopperClose(int nextState) {
        if (stopperTimer.seconds() > stopperWaitTime) {
            stopperTimer.reset();
            setPathState(nextState);
        }
    }

    private void handleIntakeDelay(int nextState) {
        if (stopperTimer.seconds() > stopperWaitTime) {
            robot.intake.setPower(1.0); // Включаем на 100% перед заездом в линию
            intakeTimer.reset();
            setPathState(nextState);
        }
    }

    // 🔥 БЫСТРЫЙ СБОР (Для линии - без остановки)
    private void handleLinePickup(int nextState, PathChain nextPath) {
        boolean isDoneDriving = !follower.isBusy();
        boolean isTimedOut = pathTimer.seconds() > pathTimeout;

        if (isDoneDriving || isTimedOut) {
            if (isTimedOut) follower.breakFollowing();

            stopperTimer.reset();
            follower.followPath(nextPath, true);
            setPathState(nextState);
        }
    }

    // 🔥 УМНЫЙ СБОР С ГЕЙТА (С проверкой даже в движении)
    private void handleGatePickup(int nextState, PathChain nextPath, double waitTimeout, double rpmThreshold) {
        boolean isDoneDriving = !follower.isBusy();
        boolean isPathTimedOut = pathTimer.seconds() > pathTimeout;

        // 1. Проверяем зажевывание мяча ВСЕГДА. Ждем 0.5 сек для разгона мотора.
        boolean isIntakeJammed = false;
        if (pathTimer.seconds() > 0.5) {
            isIntakeJammed = Math.abs(robot.intake.getVelocity()) < rpmThreshold;
        }

        // 2. Отмечаем прибытие на точку
        if ((isDoneDriving || isPathTimedOut) && !arrivedAtPickup) {
            if (isPathTimedOut) follower.breakFollowing();
            intakeTimer.reset();
            arrivedAtPickup = true;
        }

        // 3. Таймер ожидания на точке
        boolean isGateTimedOut = arrivedAtPickup && (intakeTimer.seconds() > waitTimeout);

        // 4. ГЛАВНЫЙ ТРИГГЕР: Наелись мячей ИЛИ вышло время — едем скорить!
        if (isIntakeJammed || isGateTimedOut) {
            follower.breakFollowing(); // Обрываем путь к гейту, если поймали мяч в полете
            robot.intake.setPower(1.0); // ✅ ЖЕСТКО 1.0, чтобы поднять мяч к стопперу во время езды

            stopperTimer.reset();
            follower.followPath(nextPath, true);

            arrivedAtPickup = false;
            setPathState(nextState);
        }
    }
}
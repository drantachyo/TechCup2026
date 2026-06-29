package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.GlobalState;
import org.firstinspires.ftc.teamcode.robot.utils.MirrorTool;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;

@TeleOp(name = "Main Duo TeleOp", group = "Competition")
public class MainTeleOp extends DecodeOpMode {

    // 1. Координаты для старта (если автонома НЕ БЫЛО)
    private final Pose START_BLUE_POSE = new Pose(72, 72, 0);
    private final Pose START_RED_POSE = new Pose(89.2, 7.09, 1.58);

    // 2. Координаты для ручного сброса позиции (калибровка во время матча)
    private final Pose RESET_BLUE_POSE = new Pose(10, 10, 0);
    private final Pose RESET_RED_POSE = new Pose(126.62, 84.76, 0);

    // Цель башни
    private final Pose baseTargetPose = new Pose(144, 144, 0);

    private boolean isTurretTracking = false;
    private boolean isAutoDrivingToZone = false;
    private boolean isAutoShootingToZone = false;

    private final ElapsedTime shootingTimer = new ElapsedTime();

    public static double idleIntakePower = 0.1;

    @Override
    public void onInit() {
        telemetry.addLine("Ready to rumble!");

        boolean isBlue = GlobalState.isBlueAlliance;
        telemetry.addData("Выбранный Альянс", isBlue ? "🟦 СИНИЙ" : "🟥 КРАСНЫЙ");
        telemetry.addData("Был ли Автоном?", GlobalState.isAutoBeen ? "ДА ✅" : "НЕТ ❌");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.turret.on();
        robot.shooter.turnOn();

        boolean isBlue = GlobalState.isBlueAlliance;
        Pose startingPose;

        if (GlobalState.isAutoBeen && GlobalState.currentPose != null) {
            startingPose = GlobalState.currentPose;
        } else {
            startingPose = isBlue ? START_BLUE_POSE : START_RED_POSE;
        }

        robot.drive.setPose(startingPose);
        GlobalState.isAutoBeen = false;
    }

    @Override
    public void onUpdate() {
        Pose currentPose = robot.drive.getPose();
        boolean isBlue = GlobalState.isBlueAlliance;

        // 🔥 Инициализируем базовые значения интейка (Зажатие Правого Бампера или кнопки А)
        double intakePower = idleIntakePower;
        if (base.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            intakePower = 1.0;
        } else if (base.getButton(GamepadKeys.Button.A)) {
            intakePower = -1.0;
        }

        boolean forceOpenStopper = false;

        // Расчет текущей зоны (используется и водителем, и оператором, и в телеметрии)
        Pose checkPose = isBlue ? MirrorTool.toBlue(currentPose) : currentPose;
        boolean inCloseZone = PoseController.isInZone(checkPose);
        boolean inSmallZone = PoseController.isInSmallZone(checkPose);

        // ==========================================
        // 🕹️ ГЕЙМПАД 1: ШАССИ И БАЗОВЫЕ ТОГЛЫ (Водитель)
        // ==========================================

        // 1. СБРОС POSЕ (КАЛИБРОВКА)
        if (base.wasJustPressed(GamepadKeys.Button.START)) {
            robot.drive.setPose(isBlue ? RESET_BLUE_POSE : RESET_RED_POSE);
            gamepad1.rumble(200);
        }

        // 2. Переключение Field-Centric
        if (base.wasJustPressed(GamepadKeys.Button.BACK)) {
            robot.drive.toggleFieldCentric();
        }

        // 3. АВТО-ПОВОРОТЫ (Snap-to-Angle)
        Double snapAngle = null;
        if (isBlue) {
            if (base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) snapAngle = Math.PI;
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) snapAngle = Math.PI / 2;
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) snapAngle = 0.0;
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) snapAngle = -Math.PI / 2;
            else if (base.wasJustPressed(GamepadKeys.Button.B)) snapAngle = 3 * Math.PI / 4;
        } else {
            if (base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) snapAngle = 0.0;
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) snapAngle = -Math.PI / 2;
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) snapAngle = Math.PI;
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) snapAngle = Math.PI / 2;
            else if (base.wasJustPressed(GamepadKeys.Button.B)) snapAngle = Math.PI / 4;
        }
        if (snapAngle != null) robot.drive.setSnapTarget(snapAngle);

        // 4. ТОГЛ СЛЕДОВАНИЯ БАШНИ НА ПЕРВОМ ДРАЙВЕРЕ (Левый Бампер)
        if (base.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            isTurretTracking = !isTurretTracking;
        }

        // 5. АВТО-АССИСТ (Подъезд к зоне / Авто-выстрел)
        // 🔥 Убрали правый бампер отсюда, теперь прерывает только движение стиками
        boolean hasManualInput = Math.abs(base.getLeftY()) > 0.1 ||
                Math.abs(base.getLeftX()) > 0.1 ||
                Math.abs(base.getRightX()) > 0.1;

        if (base.wasJustPressed(GamepadKeys.Button.X) && !isAutoDrivingToZone && !isAutoShootingToZone) {
            if (inCloseZone || inSmallZone) {
                // Если мы хоть в какой-то из зон -> СТРЕЛЯЕМ
                gamepad1.rumble(300);
                isAutoShootingToZone = true;
                shootingTimer.reset();
            } else {
                // Если вне зон -> ЕДЕМ В БОЛЬШУЮ
                Pose targetSnapped = PoseController.getNearestPose(checkPose);
                Pose finalTarget = isBlue ? MirrorTool.toBlue(targetSnapped) : targetSnapped;

                PathChain pathToZone = robot.drive.getFollower().pathBuilder()
                        .addPath(new BezierLine(currentPose, finalTarget))
                        .setTangentHeadingInterpolation()
                        .build();

                robot.drive.getFollower().followPath(pathToZone, true);
                isAutoDrivingToZone = true;
                isAutoShootingToZone = false;
            }
        }

        // Логика фаз автоматики: Езда -> Выстрел -> Отключение
        if (isAutoDrivingToZone) {
            if (hasManualInput) {
                robot.drive.getFollower().breakFollowing(); // Предохранитель при ручном перехвате движения
                robot.drive.startTeleop();
                isAutoDrivingToZone = false;
            } else if (!robot.drive.getFollower().isBusy()) { // Приехали в целевую точку
                isAutoDrivingToZone = false;
                isAutoShootingToZone = true;
                shootingTimer.reset();
            }
        } else if (isAutoShootingToZone) {
            if (hasManualInput) {
                robot.drive.startTeleop();
                isAutoShootingToZone = false;
            } else if (shootingTimer.milliseconds() < 1000) {
                robot.drive.getFollower().update();
                forceOpenStopper = true;
                intakePower = 1.0;
            } else {
                robot.drive.startTeleop();
                isAutoShootingToZone = false;
            }
        }

        if (!isAutoDrivingToZone && !isAutoShootingToZone) {
            robot.drive.drive(base, isBlue);
        }

        // ==========================================
        // 🎯 ГЕЙМПАД 2: ШУТЕР, ХУД, РУЧНОЙ СТРЕЙФ БАШНИ (Оператор)
        // ==========================================

        Pose actualTargetPose = isBlue ? MirrorTool.toBlue(baseTargetPose) : baseTargetPose;
        Pose futurePose = PoseController.getFuturePose(robot.drive.getFollower());

        double distanceToTarget = Math.hypot(actualTargetPose.getX() - futurePose.getX(), actualTargetPose.getY() - futurePose.getY());

        robot.hood.setDistance(distanceToTarget);
        robot.shooter.setDistance(distanceToTarget);

        if (isTurretTracking) {
            robot.turret.face(actualTargetPose, futurePose);
        } else {
            robot.turret.setYaw(0);
        }

        boolean isOperatorShooting = forceOpenStopper;

        if (helper.getButton(GamepadKeys.Button.X)) {
            // Разрешаем ручной выстрел оператора, если робот находится в одной из двух валидных зон
            if (inCloseZone || inSmallZone) {
                isOperatorShooting = true;
            } else {
                gamepad2.rumble(50);
            }
        }

        if (isOperatorShooting) {
            robot.stopper.open();
            intakePower = 1.0; // Принудительный оверрайд интейка на 1.0 во время стрельбы
        } else {
            robot.stopper.close();
        }

        // Финальная передача мощности на мотор интейка
        robot.intake.setPower(intakePower);

        // ==========================================
        // 📊 ТЕЛЕМЕТРИЯ
        // ==========================================
        telemetry.addData("Shooter RPM", robot.shooter.getVelocity());
        telemetry.addData("Turret Tracking", isTurretTracking ? "ON (Auto)" : "OFF (Forward)");
        telemetry.addData("Dynamic Distance", String.format("%.2f", distanceToTarget));

        telemetry.addData("In Close Zone?", inCloseZone ? "YES ✅" : "NO ❌");
        telemetry.addData("In Small Zone?", inSmallZone ? "YES ✅" : "NO ❌");

        String assistStatus = "STANDBY";
        if (isAutoDrivingToZone) assistStatus = "DRIVING";
        else if (isAutoShootingToZone) assistStatus = "SHOOTING";
        telemetry.addData("Auto-Assist", assistStatus);

        telemetry.addData("Alliance", isBlue ? "🟦 BLUE" : "🟥 RED");
        telemetry.update();
    }
}

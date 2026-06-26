package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.PoseController;

@TeleOp(name = "Main Duo TeleOp", group = "Competition")
public class MainTeleOp extends DecodeOpMode {

    // Выбор альянса
    private boolean isBlueAlliance = true;

    // Хардкоженные координаты сброса на кнопку START
    // Запиши сюда реальные стартовые координаты!
    private final Pose HARDCODED_BLUE_POSE = new Pose(72, 72, 0);
    private final Pose HARDCODED_RED_POSE = new Pose(72, 72, Math.PI);

    // Цель башни
    private Pose targetPose = new Pose(144, 144, 0);

    // Флаги состояний
    private boolean isAutoAiming = false;
    private boolean isAutoDrivingToZone = false;
    private boolean isIntakeOn = false; // Тогл интейка

    public static double idleIntakePower = 0.1;

    @Override
    public void onInit() {
        telemetry.addLine("Ready to rumble!");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.turret.on();
        // 🔥 Шутер теперь включается один раз и работает всегда
        robot.shooter.turnOn();
    }

    @Override
    public void onUpdate() {
        Pose currentPose = robot.drive.getPose();

        // ==========================================
        // 🕹️ ГЕЙМПАД 1: ШАССИ (Водитель)
        // ==========================================

        // 1. СБРОС POSЕ НА ХАРДКОД (Кнопка START)
        if (base.wasJustPressed(GamepadKeys.Button.START)) {
            robot.drive.setPose(isBlueAlliance ? HARDCODED_BLUE_POSE : HARDCODED_RED_POSE);
            gamepad1.rumble(200);
        }

        // 2. Переключение Field-Centric
        if (base.wasJustPressed(GamepadKeys.Button.BACK)) {
            robot.drive.toggleFieldCentric();
        }

        // 3. АВТО-ПОВОРОТЫ (Динамика от альянса)
        Double snapAngle = null;
        if (isBlueAlliance) {
            if (base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) snapAngle = Math.PI; // 180
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) snapAngle = Math.PI / 2; // 90
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) snapAngle = 0.0; // 0
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) snapAngle = -Math.PI / 2; // 270
            else if (base.wasJustPressed(GamepadKeys.Button.B)) snapAngle = 3 * Math.PI / 4; // 135
        } else {
            if (base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) snapAngle = Math.PI / 2; // 90
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) snapAngle = 0.0; // 0
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) snapAngle = -Math.PI / 2; // 270
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) snapAngle = Math.PI; // 180
            else if (base.wasJustPressed(GamepadKeys.Button.B)) snapAngle = Math.PI / 4; // 45
        }
        if (snapAngle != null) robot.drive.setSnapTarget(snapAngle);

        // 4. ИНТЕЙК ВОДИТЕЛЯ (Тогл и Реверс)
        if (base.wasJustPressed(GamepadKeys.Button.Y)) {
            isIntakeOn = !isIntakeOn; // Переключатель
        }

        double intakePower = isIntakeOn ? 1.0 : idleIntakePower;

        if (base.getButton(GamepadKeys.Button.A)) {
            intakePower = -1.0; // Зажатие А (Выплюнуть) перебивает тогл
        }

        // 5. АВТО-АССИСТ (Подъезд к зоне)
        boolean hasManualInput = Math.abs(base.getLeftY()) > 0.1 ||
                Math.abs(base.getLeftX()) > 0.1 ||
                Math.abs(base.getRightX()) > 0.1 ||
                base.getButton(GamepadKeys.Button.RIGHT_BUMPER); // Тормоз отменяет автоном

        if (base.wasJustPressed(GamepadKeys.Button.X) && !isAutoDrivingToZone) {
            if (PoseController.isInZone(currentPose)) {
                gamepad1.rumble(300);
            } else {
                Pose targetSnapped = PoseController.getNearestPose(currentPose);
                PathChain pathToZone = robot.drive.getFollower().pathBuilder()
                        .addPath(new BezierLine(currentPose, targetSnapped))
                        .setTangentHeadingInterpolation()
                        .build();

                robot.drive.getFollower().followPath(pathToZone, true);
                isAutoDrivingToZone = true;
            }
        }

        if (isAutoDrivingToZone) {
            if (hasManualInput) {
                robot.drive.getFollower().breakFollowing();
                robot.drive.startTeleop();
                isAutoDrivingToZone = false;
            } else if (!robot.drive.getFollower().isBusy()) {
                robot.drive.startTeleop();
                isAutoDrivingToZone = false;
            }
        } else {
            // Ручная езда (с учетом слоу-мода и тормоза внутри drive)
            robot.drive.drive(base, isBlueAlliance);
        }


        // ==========================================
        // 🎯 ГЕЙМПАД 2: ШУТЕР, БАШНЯ, ХУД (Оператор)
        // ==========================================

        if (helper.wasJustPressed(GamepadKeys.Button.B)) {
            isAutoAiming = !isAutoAiming;
        }

        // Логика авто-наведения
        if (isAutoAiming) {
            Pose futurePose = PoseController.getFuturePose(robot.drive.getFollower());
            double distanceToTarget = Math.hypot(targetPose.getX() - futurePose.getX(), targetPose.getY() - futurePose.getY());

            robot.turret.face(targetPose, futurePose);
            robot.hood.setDistance(distanceToTarget);
            robot.shooter.setDistance(distanceToTarget); // Шутер всегда работает, просто меняет обороты
        } else {
            robot.turret.setYaw(0);
        }

        // 🔥 ЗАЩИЩЕННЫЙ ВЫСТРЕЛ (Кнопка X оператора)
        boolean isOperatorShooting = false;

        if (helper.getButton(GamepadKeys.Button.X)) {
            if (PoseController.isInZone(currentPose)) {
                isOperatorShooting = true;
            } else {
                // Легкая вибрация, чтобы оператор понял: "Мы не в зоне, стрелять нельзя!"
                gamepad2.rumble(50);
            }
        }

        if (isOperatorShooting) {
            robot.stopper.open();
            intakePower = 1.0; // Помогаем интейком затолкнуть мяч
        } else {
            robot.stopper.close();
        }

        // Финальная передача мощности на интейк
        robot.intake.setPower(intakePower);

        // ==========================================
        // 📊 ТЕЛЕМЕТРИЯ
        // ==========================================
        telemetry.addData("Shooter RPM", robot.shooter.getVelocity());
        telemetry.addData("Auto-Aim", isAutoAiming ? "ON" : "OFF");
        telemetry.addData("In Launch Zone?", PoseController.isInZone(currentPose) ? "YES ✅" : "NO ❌");
        telemetry.addData("Auto-Assist Driving", isAutoDrivingToZone ? "ACTIVE" : "STANDBY");
        telemetry.update();
    }
}
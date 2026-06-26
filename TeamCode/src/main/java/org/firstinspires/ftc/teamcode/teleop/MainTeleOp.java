package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.PoseController;

@TeleOp(name = "Main Duo TeleOp", group = "Competition")
public class MainTeleOp extends DecodeOpMode {

    private boolean isBlueAlliance = false;

    // Координаты цели (корзины/хаба)
    private Pose targetPose = new Pose(144, 144, 0);

    private boolean isAutoAiming = false;

    @Override
    public void onInit() {
        telemetry.addLine("Ready to rumble!");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.turret.on();
        // Принудительно выключаем шутер на старте
        robot.shooter.turnOff();
    }

    @Override
    public void onUpdate() {
        // ==========================================
        // 🕹️ ГЕЙМПАД 1: ШАССИ И ИНТЕЙК (Водитель)
        // ==========================================

        // СБРОС НАПРАВЛЕНИЯ FIELD-CENTRIC (Кнопка START/OPTIONS)
        if (base.wasJustPressed(GamepadKeys.Button.START)) {
            Pose currentPose = robot.drive.getPose();
            robot.drive.setPose(new Pose(currentPose.getX(), currentPose.getY(), 0));
        }

        // 1. Управление шасси
        robot.drive.drive(base, isBlueAlliance);

        // 2. Переключение Field-Centric режима
        if (base.wasJustPressed(GamepadKeys.Button.BACK)) {
            robot.drive.toggleFieldCentric();
        }

        // ==========================================
        // 🎯 ГЕЙМПАД 2: ШУТЕР, БАШНЯ, ХУД (Оператор)
        // ==========================================

        // 1. Включение/выключение маховика шутера
        if (helper.wasJustPressed(GamepadKeys.Button.A)) {
            robot.shooter.toggle();
        }

        // 2. Переключение режима авто-прицеливания
        if (helper.wasJustPressed(GamepadKeys.Button.B)) {
            isAutoAiming = !isAutoAiming;
        }

        // 3. Логика авто-наведения
        if (isAutoAiming) {
            Pose futurePose = PoseController.getFuturePose(robot.drive.getFollower());
            Pose currentPose = robot.drive.getPose();

            // Безопасный расчет дистанции через встроенную математику Java (Math.hypot)
            // Это тоже поможет избежать случайных ошибок с методами Pedro Pathing
            double distanceToTarget = Math.hypot(targetPose.getX() - futurePose.getX(), targetPose.getY() - futurePose.getY());

            robot.turret.face(targetPose, futurePose);
            robot.hood.setDistance(distanceToTarget);

            if (robot.shooter.isActivated) {
                robot.shooter.setDistance(distanceToTarget);
            }
        } else {
            robot.turret.setYaw(0);
        }

        // ==========================================
        // 🔄 ОБЪЕДИНЕННАЯ ЛОГИКА: СТОППЕР + ИНТЕЙК
        // ==========================================

        // 🔥 Интейк на кнопках Y (забрать) и A (выплюнуть)
        double intakePower = 0;
        if (base.getButton(GamepadKeys.Button.Y)) {
            intakePower = 1.0;
        } else if (base.getButton(GamepadKeys.Button.A)) {
            intakePower = -1.0;
        }

        // 🔥 УСЛОВИЕ ВЫСТРЕЛА: Теперь работает мгновенно без проверок скорости
        boolean isShooting = helper.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        if (isShooting) {
            robot.stopper.open();
            // Перебиваем команду водителя: принудительно крутим интейк на 100%
            intakePower = 1.0;
        } else {
            robot.stopper.close();
        }

        // Применяем финальную мощность
        robot.intake.setPower(intakePower);

        // ==========================================
        // 📊 ТЕЛЕМЕТРИЯ
        // ==========================================
        Pose currentPose = robot.drive.getPose();
        Pose futurePose = PoseController.getFuturePose(robot.drive.getFollower());
        telemetry.addData("🤖 Робот X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("🤖 Робот Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("🤖 Угол (Градусы)", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));

        // Предсказанная позиция (куда рассчитывается упреждение)
        telemetry.addData("🔮 Будущий X", String.format("%.2f", futurePose.getX()));
        telemetry.addData("🔮 Будущий Y", String.format("%.2f", futurePose.getY()));
        telemetry.addData("Shooter RPM", robot.shooter.getVelocity());
        telemetry.addData("Auto-Aim Active", isAutoAiming);
        telemetry.update();
    }
}
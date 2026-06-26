package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.PoseController;

@TeleOp(name = "Main Duo TeleOp", group = "Competition")
public class MainTeleOp extends DecodeOpMode {

    // Выбери нужный альянс (можно потом вынести в настройку Автонома)
    private boolean isBlueAlliance = true;

    // Координаты цели (корзины/хаба). Задай реальные координаты на поле!
    // Пример: координаты красной цели (или синей).
    private Pose targetPose = new Pose(144, 36, 0);

    // Режим автонаведения оператора
    private boolean isAutoAiming = false;

    @Override
    public void onInit() {
        // Здесь можно вывести что-то в телеметрию при инициализации
        telemetry.addLine("Ready to rumble!");
        telemetry.update();
    }

    @Override
    public void onStart() {
        // Вызывается при нажатии PLAY
        // Башня и шутер по умолчанию включены в режим ожидания
        robot.turret.on();
    }

    @Override
    public void onUpdate() {
        // ==========================================
        // 🕹️ ГЕЙМПАД 1: ШАССИ И ИНТЕЙК (Водитель)
        // ==========================================

        // 1. Управление шасси (передаем весь геймпад прямо в твой метод)
        robot.drive.drive(base, isBlueAlliance);

        // 2. Управление интейком (Правый триггер - забрать, Левый - выплюнуть)
        double intakePower = base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - base.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        robot.intake.setPower(intakePower);

        // 3. Переключение Field-Centric режима (кнопка BACK/SELECT)
        if (base.wasJustPressed(GamepadKeys.Button.BACK)) {
            robot.drive.toggleFieldCentric();
        }


        // ==========================================
        // 🎯 ГЕЙМПАД 2: ШУТЕР, БАШНЯ, ХУД, СТОППЕР (Оператор)
        // ==========================================

        // 1. Включение/выключение маховика шутера (Кнопка A)
        if (helper.wasJustPressed(GamepadKeys.Button.A)) {
            robot.shooter.toggle();
        }

        // 2. Переключение режима авто-прицеливания (Кнопка B)
        if (helper.wasJustPressed(GamepadKeys.Button.B)) {
            isAutoAiming = !isAutoAiming;
        }

        // 3. Логика авто-наведения (Башня, Худ, Скорость)
        if (isAutoAiming) {
            // Используем твой PoseController для предсказания позиции (стрельба на ходу)
            Pose futurePose = PoseController.getFuturePose(robot.drive.getFollower());
            Pose currentPose = robot.drive.getPose();

            // Дистанция до цели
            double distanceToTarget = currentPose.distanceFrom(targetPose);

            // Наводим башню на цель (с учетом движения)
            robot.turret.face(targetPose, futurePose);

            // Автоматически выставляем угол капюшона и скорость маховика по LUT-таблицам
            robot.hood.setDistance(distanceToTarget);

            // Задаем скорость только если шутер включен
            if (robot.shooter.isActivated) {
                robot.shooter.setDistance(distanceToTarget);
            }
        } else {
            // Ручной сброс башни в центр, если автонаведение выключено
            robot.turret.setYaw(0);
        }

        // 4. Стрельба / Стоппер (Правый бампер - R1)
        // Если оператор зажал R1 И шутер разогнался до нужной скорости -> открываем заслонку
        if (helper.getButton(GamepadKeys.Button.RIGHT_BUMPER) && robot.shooter.isAtTarget()) {
            robot.stopper.open();
        } else {
            robot.stopper.close();
        }


        // ==========================================
        // 📊 ТЕЛЕМЕТРИЯ ДЛЯ ДЕБАГА
        // ==========================================
        telemetry.addData("Shooter RPM", robot.shooter.getVelocity());
        telemetry.addData("Shooter Target", robot.shooter.getTarget());
        telemetry.addData("Shooter Ready?", robot.shooter.isAtTarget());
        telemetry.addData("Auto-Aim Active", isAutoAiming);
        telemetry.addData("Turret Ticks", robot.turret.getCurrentTicks());
        telemetry.update();
    }
}
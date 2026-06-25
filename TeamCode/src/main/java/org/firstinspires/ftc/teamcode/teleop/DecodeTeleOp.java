package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;

@TeleOp(name = "Main TeleOp", group = "Decode")
public class DecodeTeleOp extends DecodeOpMode {

    // Координаты цели для башни на поле (например, центр корзины)
    // Подставь свои реальные координаты
    private final Pose targetPose = new Pose(144, 144, 0);

    @Override
    public void onInit() {
        // Вызывается один раз при нажатии INIT
        telemetry.addLine("System: Ready to rumble!");
        telemetry.update();
    }

    @Override
    public void onStart() {
        // Вызывается один раз при нажатии PLAY
        robot.turret.on(); // Даем питание на ПИД башни
    }

    @Override
    public void onUpdate() {
        // ==========================================
        // 1. ШАССИ (УПРАВЛЕНИЕ ДВИЖЕНИЕМ) - ВОДИТЕЛЬ (base)
        // ==========================================

        // Вторым параметром идет цвет альянса для Field-Centric (true = Blue)
        // В будущем можно брать цвет из твоего GlobalState
        robot.drive.drive(base, true);

        // Комбинация-спасатель: Сброс Field-Centric (LEFT_BUMPER + BACK)
        // Если робота закрутили и он забыл, где "перед"
        if (base.isDown(GamepadKeys.Button.LEFT_BUMPER) && base.wasJustPressed(GamepadKeys.Button.BACK)) {
            // Допустим, мы прижались к стене, смотря в 0 градусов
            robot.drive.setPose(new Pose(9, 9, 0));
            gamepad1.rumble(500); // Даем виброотклик водителю, что сброс прошел
        }

        // ==========================================
        // 2. ИНТЕЙК - ВОДИТЕЛЬ (base) или ПОМОЩНИК (helper)
        // ==========================================

        // Правый триггер — засасываем (до 1.0), Левый — выплевываем (до -1.0)
        double intakePower = base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                - base.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        robot.intake.setPower(intakePower);

        // ==========================================
        // 3. ШУТЕР - ПОМОЩНИК (helper)
        // ==========================================

        // Включаем/выключаем маховик на кнопку A (Крестик на PS)
        if (helper.wasJustPressed(GamepadKeys.Button.A)) {
            robot.shooter.toggle();
        }

        // Выбираем дистанцию по LUT с помощью крестовины (D-Pad)
        if (helper.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            robot.shooter.setDistance(150.6); // Максимальная дистанция из твоего LUT
        } else if (helper.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            robot.shooter.setDistance(43.6);  // Ближняя дистанция
        }

        // ==========================================
        // ==========================================
        // 4. БАШНЯ - АВТОПРИЦЕЛ (Автоматика)
        // ==========================================

        // Если зажат правый бампер — башня жестко лочит цель, а шутер и худ настраиваются
        if (base.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            // 1. Получаем предсказанную позицию через геттер
            Pose futureRobotPose = PoseController.getFuturePose(robot.drive.getFollower());

            // 2. Получаем угловую скорость вращения шасси
            double angularVelocity = robot.drive.getFollower().getAngularVelocity();

            // 3. Отдаем в башню
            robot.turret.face(targetPose, futureRobotPose, angularVelocity);

            // 4. СЧИТАЕМ УПРЕЖДЕННУЮ ДИСТАНЦИЮ
            // Используем метод distanceFrom, который есть в PedroPathing Pose (ты используешь его в PoseController)
            double futureDistance = futureRobotPose.distanceFrom(targetPose);

            // 5. Автоматически выставляем Худ и Шутер по будущей дистанции
            robot.shooter.setDistance(futureDistance);
            robot.hood.setDistance(futureDistance);

            // (Опционально) Блокировка выстрела, если мы не в зоне:
            if (!PoseController.isInZone(robot.drive.getPose())) {
                telemetry.addLine("WARNING: OUT OF LAUNCH ZONE!");
            }
        } else {
            // Если кнопку отпустили — башня смотрит прямо по курсу робота (0 градусов)
            robot.turret.setYaw(0);
        }
        // 5. ТЕЛЕМЕТРИЯ
        // ==========================================

        telemetry.addData("Loop Time (sec)", deltaTime);
        telemetry.addData("Shooter Target RPM", robot.shooter.getTarget());
        telemetry.addData("Shooter Current RPM", robot.shooter.getVelocity());
        telemetry.addData("Turret Target Ticks", Turret.targetTicks);
        telemetry.addData("Turret Current Ticks", robot.turret.getCurrentTicks());
        telemetry.update();
    }
}
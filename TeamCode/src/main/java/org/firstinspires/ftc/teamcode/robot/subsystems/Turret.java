package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Turret {
    private DcMotorEx turretMotor;
    private PIDController pidController;

    // --- НАСТРОЙКИ ПИД ДЛЯ МОТОРА (Крутить в Panels) ---
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0001;

    // --- КОЭФФИЦИЕНТЫ ПЕРЕВОДА (Тюнить под ваш редуктор) ---
    // Сколько тиков энкодера приходится на 1 радиан поворота башни
    public static double ticksPerRadian = 500.0;

    // Менеджер целей
    public static double targetTicks = 0;

    // --- БЕЗОПАСНОСТЬ (Чтобы не порвать провода) ---
    // Задай лимиты в тиках энкодера относительно нуля
    public static int ticksMin = -1500;
    public static int ticksMax = 1500;
    public static double maxAngle = Math.toRadians(158); // Лимит в радианах

    // rAM — коэффициент упреждения при вращении самого робота
    public static double rAM = -0.14;

    public static boolean isEnabled = true;

    public void init(HardwareMap hw) {
        turretMotor = hw.get(DcMotorEx.class, "turret");

        // Сбрасываем энкодер при инициализации (робот должен стоять башней строго ПРЯМО вперед)
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Отключаем встроенный ПИД, так как мы пишем кастомный и более шустрый
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Жестко тормозим мотор, когда на него не подается питание
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Инициализируем ПИД из официальной FTCLib
        pidController = new PIDController(kP, kI, kD);
    }

    public void periodic() {
        if (isEnabled) {
            // Обновляем коэффициенты ПИД на лету из Panels
            pidController.setPID(kP, kI, kD);

            // Получаем текущую позицию мотора
            double currentTicks = turretMotor.getCurrentPosition();

            // Считаем мощность мотора через ПИД-регулятор
            double power = pidController.calculate(currentTicks, targetTicks);

            // Ограничиваем максимальную мощность башни (например, до 70%), чтобы она не дергалась слишком резко
            power = Range.clip(power, -0.7, 0.7);

            // Если мы уже очень близко к цели (ошибка меньше 3 тиков) — глушим мотор, чтобы не жужжал
            if (Math.abs(targetTicks - currentTicks) < 3) {
                power = 0;
            }

            turretMotor.setPower(power);
        } else {
            turretMotor.setPower(0);
        }
    }

    // ==========================================
    // МАТЕМАТИКА НАВЕДЕНИЯ НА ЦЕЛЬ
    // ==========================================

    public void face(Pose targetPose, Pose robotPose) {
        face(targetPose, robotPose, 0);
    }

    /**
     * Главный метод наведения башни на цель на поле
     * @param angleVel Текущая угловая скорость робота (из Pedro Pathing)
     */
    public void face(Pose targetPose, Pose robotPose, double angleVel) {
        double y = targetPose.getY() - robotPose.getY();
        double x = targetPose.getX() - robotPose.getX();

        // Мертвая зона для угловой скорости шасси
        if (Math.abs(angleVel) < 2.5) {
            angleVel = 0;
        }

        // 1. Считаем чистый угол до цели + упреждение на вращение робота
        double angleToTargetFromCenter = Math.atan2(y, x) + (angleVel * rAM);

        // 2. Переводим глобальный угол поля в локальный угол робота
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());

        // 3. Отправляем угол на моторы
        setYaw(robotAngleDiff);
    }

    /**
     * Принимает угол в радианах и переводит его в тики для ПИДа
     */
    public void setYaw(double radians) {
        radians = normalizeAngle(radians);

        // Переводим радианы в тики мотора
        double calculatedTicks = radians * ticksPerRadian;

        // Жесткий программный ограничитель (клиппинг) по лимитам тиков
        targetTicks = Range.clip((int) calculatedTicks, ticksMin, ticksMax);
    }

    public double getYaw() {
        // Перевод текущих тиков обратно в радианы
        return turretMotor.getCurrentPosition() / ticksPerRadian;
    }

    public int getCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    public void on() {
        isEnabled = true;
    }

    public void off() {
        isEnabled = false;
    }

    // ==========================================
    // СЕРВИСНАЯ МАТЕМАТИКА УГЛОВ
    // ==========================================

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return rangeAngle(angle);
    }

    public static double rangeAngle(double angleRadians) {
        if (angleRadians <= -maxAngle) angleRadians = -maxAngle;
        if (angleRadians > maxAngle) angleRadians = maxAngle;
        return angleRadians;
    }
}
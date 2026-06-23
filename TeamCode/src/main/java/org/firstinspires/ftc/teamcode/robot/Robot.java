package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

/**
 * Главный класс-контейнер робота команды Decode (Паттерн Singleton).
 * Управляет жизненным циклом всех подсистем.
 */
public class Robot {
    // Единственный экземпляр класса в памяти
    private static final Robot INSTANCE = new Robot();

    // Подсистемы робота (объявлены как public final для быстрого доступа)
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    // Закрытый конструктор (никто не может создать Robot через new Robot())
    private Robot() {
        drive = new Drive();
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
    }

    /**
     * Получить глобальный экземпляр робота.
     */
    public static Robot getInstance() {
        return INSTANCE;
    }

    /**
     * Инициализация всех подсистем.
     * Вызывается автоматически внутри DecodeOpMode при нажатии INIT.
     */
    public void init(HardwareMap hw) {
        // Передаем hardwareMap в каждую подсистему
        drive.init(hw);
        intake.init(hw);
        shooter.init(hw);
        turret.init(hw);
    }

    /**
     * Сквозное обновление всех механизмов.
     * Вызывается автоматически в конце каждого цикла loop() в DecodeOpMode.
     */
    public void periodic() {
        // Здесь будут крутиться локализаторы, ПИД-регуляторы и фильтры
        drive.periodic();
        intake.periodic();
        shooter.periodic();
        turret.periodic();
    }
}
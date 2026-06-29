package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Наш класс интерполяции
import org.firstinspires.ftc.teamcode.robot.utils.LUT;

@Configurable
public class Shooter {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    // --- ПЕРЕМЕННЫЕ ДЛЯ КОМПЕНСАЦИИ НАПРЯЖЕНИЯ ---
    private VoltageSensor voltageSensor;
    private ElapsedTime voltageTimer;
    private double lastVoltage = 12.0;

    public static double targetVelocity = 0;

    // Настройки в Panels
    public static double kS = 0.015;
    public static double kP = 0.005;
    public static double kV1 = 0.00038;
    public static double kV2 = 0.00038;
    public static double speed1 = 1100;
    public static double speed2 = 1800;
    public static boolean isActivated = true;
    public static boolean useVoltageComp = true;

    // 🔥 ЕДИНАЯ ТАБЛИЦА ДИСТАНЦИЙ (Pre-spinning)
    public LUT shooterLut;

    public void init(HardwareMap hw) {
        leftMotor = hw.get(DcMotorEx.class, "sl");
        rightMotor = hw.get(DcMotorEx.class, "sr");

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- ИНИЦИАЛИЗАЦИЯ НАПРЯЖЕНИЯ ---
        voltageSensor = hw.voltageSensor.iterator().next();
        voltageTimer = new ElapsedTime();
        lastVoltage = voltageSensor.getVoltage();

        // ==========================================
        // 🎯 КАЛИБРОВКА ЕДИНОГО LUT
        // Вписываем ВСЕ дистанции: от самой ближней точки до самой дальней.
        // Значения ниже - примерные, откалибруйте их на практике!
        // ==========================================
        shooterLut = new LUT();

        // Зона в упор (Close Zone)
        shooterLut.add(40, 1050);
        shooterLut.add(50, 1100);
        shooterLut.add(60, 1200);
        shooterLut.add(70, 1250);
        shooterLut.add(80, 1250);
        shooterLut.add(90, 1375);
        shooterLut.add(100, 1400);
        shooterLut.add(110, 1450);
        shooterLut.add(120, 1500);
        shooterLut.add(132, 1650);
        shooterLut.add(137, 1675);
        shooterLut.add(142, 1725);
        shooterLut.add(147, 1750);
        shooterLut.add(153, 1800);
        shooterLut.add(158, 1850);
        shooterLut.add(165, 1860);
    }

    public void periodic() {
        if (isActivated && targetVelocity > 0) {
            double currentVel = getVelocity();

            // Динамический расчет Feedforward
            double interpolated_kV = kV1 + (targetVelocity - speed1) / (speed2 - speed1) * (kV2 - kV1);

            // Итоговая мощность = Feedforward + ПИД + Статическое трение
            double power = (interpolated_kV * targetVelocity) + (kP * (targetVelocity - currentVel)) + kS;

            setPower(power);
        } else {
            setPower(0);
        }
    }

    private void setPower(double power) {
        if (useVoltageComp) {
            // Читаем датчик каждые 250 мс для экономии ресурсов шины I2C
            if (voltageTimer.milliseconds() > 250) {
                lastVoltage = voltageSensor.getVoltage();
                voltageTimer.reset();
            }
            // Пропорционально повышаем мощность при просадке батареи
            power = power * (12.0 / lastVoltage);
        }

        // Защита от превышения лимитов
        power = Math.max(0.0, Math.min(1.0, power));

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    // ==========================================
    // ЛОГИКА УПРАВЛЕНИЯ
    // ==========================================

    public void turnOn() {
        isActivated = true;
    }

    public void turnOff() {
        isActivated = false;
        targetVelocity = 0;
        setPower(0);
    }

    public void toggle() {
        if (isActivated) turnOff();
        else turnOn();
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    /**
     * 🔥 Теперь Шутер берет обороты исключительно из дистанции.
     * Робот начнет разгон (Pre-spin) еще до заезда в зону!
     */
    public void setDistance(double targetDistance) {
        setTargetVelocity(shooterLut.get(targetDistance));
    }

    // ==========================================
    // ГЕТТЕРЫ И СЕРВИСНЫЕ МЕТОДЫ
    // ==========================================

    public double getTarget() {
        return targetVelocity;
    }

    public double getVelocity() {
        return leftMotor.getVelocity();
    }

    public boolean isAtTarget() {
        // Погрешность в 60 RPM считается допустимой для выстрела
        return Math.abs(getTarget() - getVelocity()) < 60;
    }

    public void enableVoltageCompensation(boolean enable) {
        useVoltageComp = enable;
    }
}
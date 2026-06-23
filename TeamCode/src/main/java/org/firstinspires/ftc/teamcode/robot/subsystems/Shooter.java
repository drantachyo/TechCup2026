package org.firstinspires.ftc.teamcode.robot.subsystems;

// Импортируем аннотацию ByLazar Panels вместо Acme Dashboard
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

// Импортируем наш собственный класс интерполяции
import org.firstinspires.ftc.teamcode.robot.utils.LUT;

@Configurable
public class Shooter {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private VoltageSensor voltageSensor; // Для компенсации просадок батареи

    public static double targetVelocity = 0;

    // Переменные для настройки в Panels
    public static double kS = 0.07;
    public static double kP = 0.004;
    public static double kV1 = 0.000293;
    public static double kV2 = 0.000317;
    public static double speed1 = 1100;
    public static double speed2 = 1800;

    public static boolean isActivated = true;
    public static boolean useVoltageComp = false;

    // Наша таблица дистанций
    public LUT shooterLut;

    public void init(HardwareMap hw) {
        // Инициализация моторов
        leftMotor = hw.get(DcMotorEx.class, "sl");
        rightMotor = hw.get(DcMotorEx.class, "sr");

        // Реверсируем один из моторов, если они стоят зеркально
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Отключаем встроенный ПИД, так как считаем свой в periodic()
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Датчик напряжения для компенсации
        voltageSensor = hw.voltageSensor.iterator().next();

        // Заполняем наш LUT (Дистанция -> Скорость маховика)
        shooterLut = new LUT();
        shooterLut.add(43.6, 980);
        shooterLut.add(75.8, 1320);
        shooterLut.add(102.6, 1360);
        shooterLut.add(135.6, 1740);
        shooterLut.add(150.6, 1840);
    }

    public void periodic() {
        if (isActivated && targetVelocity > 0) {
            double currentVel = getVelocity();

            // Интерполяция коэффициента kV в реальном времени
            double interpolated_kV = kV1 + (targetVelocity - speed1) / (speed2 - speed1) * (kV2 - kV1);

            // Расчет итоговой мощности: Feedforward + PID-коррекция + преодоление статического трения
            double power = (interpolated_kV * targetVelocity) + (kP * (targetVelocity - currentVel)) + kS;

            setPower(power);
        } else {
            setPower(0);
        }
    }

    private void setPower(double power) {
        if (useVoltageComp) {
            // Если батарея падает, пропорционально бустим мощность
            double currentVoltage = voltageSensor.getVoltage();
            power = power * (12.0 / currentVoltage);
        }

        // Предохранитель (клиппинг мощности от 0 до 1)
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
     * Автоматически выставляет скорость шутера по расстоянию до цели через LUT
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
        return Math.abs(getTarget() - getVelocity()) < 60;
    }

    public void enableVoltageCompensation(boolean enable) {
        useVoltageComp = enable;
    }
}
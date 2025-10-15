package frc.robot.subsystems.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;

public final class Diagnostics {
    private Diagnostics() {}

    /** Try to read MotorOutput.Commutation if Phoenix6 vendordep supports it. */
    private static String commutationOf(TalonFXConfiguration cfg) {
        try {
            var fld = cfg.MotorOutput.getClass().getField("Commutation");
            Object val = fld.get(cfg.MotorOutput);
            return String.valueOf(val); // "FOC" or "Trapezoidal"
        } catch (NoSuchFieldException e) {
            return "N/A (old Phoenix)";
        } catch (Exception e) {
            return "ERR:" + e.getClass().getSimpleName();
        }
    }

    /** Boot-time diagnostic for one corner: CANcoder + steerFX + driveFX. */
    public static void bootDiagOne(String tag, int encId, int steerId, int driveId) {
        if (!RobotBase.isReal()) {
            System.out.printf(
                    "[Boot][%s] (sim) encId=%d steerId=%d driveId=%d%n",
                    tag, encId, steerId, driveId);
            return;
        }

        var ecfg = new CANcoderConfiguration();
        var scfg = new TalonFXConfiguration();
        var dcfg = new TalonFXConfiguration();

        try (CANcoder enc = new CANcoder(encId);
                TalonFX fxSteer = new TalonFX(steerId);
                TalonFX fxDrive = new TalonFX(driveId)) {

            var sc1 = enc.getConfigurator().refresh(ecfg);
            var sc2 = fxSteer.getConfigurator().refresh(scfg);
            var sc3 = fxDrive.getConfigurator().refresh(dcfg);

            double absTurns = enc.getAbsolutePosition().getValueAsDouble();

            System.out.printf(
                    "[Boot][%s] encId=%d abs=%.6f off=%.6f dir=%s  steerFB=%s  steerComm=%s  driveComm=%s%n",
                    tag,
                    encId,
                    absTurns,
                    ecfg.MagnetSensor.MagnetOffset,
                    ecfg.MagnetSensor.SensorDirection,
                    scfg.Feedback.FeedbackSensorSource,
                    commutationOf(scfg),
                    commutationOf(dcfg));

            if (!sc1.isOK()) System.out.printf("[Boot][%s] WARN: CANcoder refresh=%s%n", tag, sc1);
            if (!sc2.isOK()) System.out.printf("[Boot][%s] WARN: SteerFX  refresh=%s%n", tag, sc2);
            if (!sc3.isOK()) System.out.printf("[Boot][%s] WARN: DriveFX  refresh=%s%n", tag, sc3);
        } catch (Exception e) {
            System.out.printf("[Boot][%s] WARN: exception reading devices: %s%n", tag, e);
        }
    }
}

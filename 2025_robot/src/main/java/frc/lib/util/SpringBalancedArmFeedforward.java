package frc.lib.util;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SpringBalancedArmFeedforward extends ArmFeedforward {
    private final double kb, neutralLength;
    private final Translation2d stringAttachmentPos, pulleyPos;
    SpringBalancedArmFeedforward(
        double ks,
        double kg,
        double kv,
        double ka,
        double kb,
        Translation2d stringAttachementPos,
        Translation2d pulleyPos,
        double neutralLength) {
            super(ks, kg, kv, ka);
            this.kb = kb;
            this.neutralLength = neutralLength;
            this.stringAttachmentPos = stringAttachementPos;
            this.pulleyPos = pulleyPos;
    }

    @Override
    public double calculate(double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
        // Gravity and motor properties voltage component:
        var gravVolts = super.calculate(positionRadians, velocityRadPerSec, accelRadPerSecSquared);
        var rotatedAttachmentPoint = stringAttachmentPos.rotateBy(new Rotation2d(positionRadians));
        var pulleyAttatchDelta = pulleyPos.minus(rotatedAttachmentPoint);
        var length = pulleyAttatchDelta.getNorm();
        var springVolts = kb * (length - neutralLength) * rotatedAttachmentPoint.getY() * pulleyAttatchDelta.getAngle().getCos();
        return gravVolts - springVolts;
    }
}

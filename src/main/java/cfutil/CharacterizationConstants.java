package cfutil;

public class CharacterizationConstants {
    private final double kS;
    private final double kV;
    private final double kA;

    private final double kP;
    private final double kI;
    private final double kD;

    private CharacterizationConstants(double kS, double kV, double kA,
     double kP, double kI, double kD) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getS() {
        return kS;
    }

    public double getV() {
        return kV;
    }

    public double getA() {
        return kA;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    // Builder pattern. Why? Because it's cool. Unnecessary, but cool. -Mingle, 12/21/2022
    public static class Builder {
        private double kS = 0.0;
        private double kV = 0.0;
        private double kA = 0.0;

        private double kP = 0.0;
        private double kI = 0.0;
        private double kD = 0.0;

        public Builder setFeedforwardConstants(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            return this;
        }

        public Builder setFeedbackConstants(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public CharacterizationConstants build() {
            return new CharacterizationConstants(kS, kV, kA, kP, kI, kD);
        }
    }
}

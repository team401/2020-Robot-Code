package org.team401.util;

/**
 * ProfiledPIDController that doesn't phone home to the boys up north.
 *
 * Implements a PID control loop whose setpoint is constrained by a trapezoid
 * profile. Users should call reset() when they first start running the controller
 * to avoid unwanted behavior.
 */
public class ProfiledPIDControllerLite {
    private PIDControllerLite m_controller;
    private TrapezoidProfileLite.State m_goal = new TrapezoidProfileLite.State();
    private TrapezoidProfileLite.State m_setpoint = new TrapezoidProfileLite.State();
    private TrapezoidProfileLite.Constraints m_constraints;

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
     * Kd.
     *
     * @param Kp          The proportional coefficient.
     * @param Ki          The integral coefficient.
     * @param Kd          The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public ProfiledPIDControllerLite(double Kp, double Ki, double Kd,
                                 TrapezoidProfileLite.Constraints constraints) {
        this(Kp, Ki, Kd, constraints, 0.02);
    }

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
     * Kd.
     *
     * @param Kp          The proportional coefficient.
     * @param Ki          The integral coefficient.
     * @param Kd          The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @param period      The period between controller updates in seconds. The
     *                    default is 0.02 seconds.
     */
    public ProfiledPIDControllerLite(double Kp, double Ki, double Kd,
                                 TrapezoidProfileLite.Constraints constraints,
                                 double period) {
        m_controller = new PIDControllerLite(Kp, Ki, Kd, period);
        m_constraints = constraints;
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Differential coefficient
     */
    public void setPID(double Kp, double Ki, double Kd) {
        m_controller.setPID(Kp, Ki, Kd);
    }

    /**
     * Sets the proportional coefficient of the PID controller gain.
     *
     * @param Kp proportional coefficient
     */
    public void setP(double Kp) {
        m_controller.setP(Kp);
    }

    /**
     * Sets the integral coefficient of the PID controller gain.
     *
     * @param Ki integral coefficient
     */
    public void setI(double Ki) {
        m_controller.setI(Ki);
    }

    /**
     * Sets the differential coefficient of the PID controller gain.
     *
     * @param Kd differential coefficient
     */
    public void setD(double Kd) {
        m_controller.setD(Kd);
    }

    /**
     * Gets the proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return m_controller.getP();
    }

    /**
     * Gets the integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return m_controller.getI();
    }

    /**
     * Gets the differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return m_controller.getD();
    }

    /**
     * Gets the period of this controller.
     *
     * @return The period of the controller.
     */
    public double getPeriod() {
        return m_controller.getPeriod();
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     */
    public void setGoal(TrapezoidProfileLite.State goal) {
        m_goal = goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    public void setGoal(double goal) {
        m_goal = new TrapezoidProfileLite.State(goal, 0);
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     */
    public TrapezoidProfileLite.State getGoal() {
        return m_goal;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     */
    public boolean atGoal() {
        return atSetpoint() && m_goal.equals(m_setpoint);
    }

    /**
     * Set velocity and acceleration constraints for goal.
     *
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public void setConstraints(TrapezoidProfileLite.Constraints constraints) {
        m_constraints = constraints;
    }

    public TrapezoidProfileLite.Constraints getConstraints() {
        return m_constraints;
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    public TrapezoidProfileLite.State getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers
     * them to be the same point and automatically calculates the shortest route
     * to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_controller.enableContinuousInput(minimumInput, maximumInput);
    }

    /**
     * Disables continuous input.
     */
    public void disableContinuousInput() {
        m_controller.disableContinuousInput();
    }

    /**
     * Sets the minimum and maximum values for the integrator.
     *
     * <p>When the cap is reached, the integrator value is added to the controller
     * output rather than the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_controller.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return m_controller.getPositionError();
    }

    /**
     * Returns the change in error per second.
     */
    public double getVelocityError() {
        return m_controller.getVelocityError();
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     */
    public double calculate(double measurement) {
        TrapezoidProfileLite profile = new TrapezoidProfileLite(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(getPeriod());
        return m_controller.calculate(measurement, m_setpoint.position);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     */
    public double calculate(double measurement, TrapezoidProfileLite.State goal) {
        setGoal(goal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     */
    public double calculate(double measurement, double goal) {
        setGoal(goal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public double calculate(double measurement, TrapezoidProfileLite.State goal,
                            TrapezoidProfileLite.Constraints constraints) {
        setConstraints(constraints);
        return calculate(measurement, goal);
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     */
    public void reset(TrapezoidProfileLite.State measurement) {
        m_controller.reset();
        m_setpoint = measurement;
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     */
    public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfileLite.State(measuredPosition, measuredVelocity));
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The velocity is
     *     assumed to be zero.
     */
    public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
    }
}

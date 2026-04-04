/**
 * @file IPIDController.hpp
 * @brief Hardware-agnostic interface for PID controllers.
 *
 * Defines a generic PID controller API that works with any PID implementation
 * (classic PID, fuzzy PID, adaptive PID, etc.). Concrete implementations
 * inherit from this interface and provide algorithm-specific functionality.
 */

#pragma once

namespace WroRobotSoftware {

/**
 * @class IPIDController
 * @brief Abstract interface for a PID controller.
 */
class IPIDController {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IPIDController() = default;

    /**
     * @brief Reset the controller internal state (integral sum, previous error, etc.).
     */
    virtual void reset() = 0;

    /**
     * @brief Compute one PID iteration.
     * @param setpoint Desired target value.
     * @param actual Current measured value.
     * @return Control output signal.
     */
    virtual float compute(float setpoint, float actual) = 0;

    /**
     * @brief Set PID gain parameters.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    virtual void setParameters(float kp, float ki, float kd) = 0;

    /**
     * @brief Get the current PID gain parameters.
     * @param kp Output: proportional gain.
     * @param ki Output: integral gain.
     * @param kd Output: derivative gain.
     */
    virtual void getParameters(float& kp, float& ki, float& kd) = 0;

    /**
     * @brief Check if the controller output has settled (error within tolerance).
     * @return true if settled, false otherwise.
     */
    virtual bool isSettled() const = 0;

    /**
     * @brief Get the error value from the most recent compute() call.
     * @return Last error (setpoint - actual).
     */
    virtual float getLastError() const = 0;

    /**
     * @brief Get the derivative term from the most recent compute() call.
     * @return Last derivative value.
     */
    virtual float getLastDerivative() const = 0;

    /**
     * @brief Get the output value from the most recent compute() call.
     * @return Last computed control output.
     */
    virtual float getOutput() const = 0;
};

}  // namespace WroRobotSoftware

// FaultManager.hpp - Manages robot fault injection for robustness testing
#ifndef FAULT_MANAGER_HPP
#define FAULT_MANAGER_HPP

#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>

/**
 * Types of faults that can occur in the robot system
 */
enum FaultType {
    BALL_SENSOR,     // Vision and ball detection faults
    COMMUNICATION,   // Inter-robot messaging faults
    LOCOMOTION,      // Movement and gait faults
    CONTROL_SYSTEM   // General control system faults
};

/**
 * Severity levels for faults
 */
enum FaultSeverity {
    LOW,     // Minor impact on performance
    MEDIUM,  // Moderate degradation
    HIGH,    // Significant impairment
    EXTREME  // Complete function failure
};

/**
 * Data structure representing a single fault instance
 */
struct Fault {
    int robotId;             // ID of robot affected by fault
    FaultType type;          // Type of fault
    FaultSeverity severity;  // Severity level
    double startTime;        // When fault begins (simulation time)
    double duration;         // How long fault persists
    double intensity;        // Magnitude of fault effect (0-1)
    bool isIntermittent;     // Whether fault occurs randomly within the duration
    double intermittentProb; // Probability of fault being active when intermittent
    bool isActive;           // Current active state
};

/**
 * Manages the generation, scheduling and application of faults to robot systems
 * for testing robustness and fault tolerance
 */
class FaultManager {
private:
    std::vector<Fault> scheduledFaults; // Collection of all faults
    double currentTime;                 // Current simulation time
    bool mEnabled;                      // Master toggle for fault injection
    
public:
    // Access methods
    bool isEnabled() { return mEnabled; }
    void setEnabled(bool enabled) { mEnabled = enabled; }
    
    // Constructor
    FaultManager();
    
    /**
     * Creates and schedules a specific fault with defined parameters
     */
    void scheduleFault(int robotId, FaultType type, FaultSeverity severity, 
                       double startTime, double duration, double intensity,
                       bool isIntermittent, double intermittentProb);
    
    /**
     * Generates random faults for testing robot resilience
     */
    void generateRandomFaults(int numRobots, double simulationDuration);
    
    /**
     * Updates fault states based on simulation time
     */
    void update(double timeStep);
    
    /**
     * Applies faults to robot sensor and control systems
     */
    bool applyBallSensorFault(int robotId, double &x, double &y);
    bool applyLocomotionFault(int robotId, double &xAmplitude, double &yAmplitude, double &aAmplitude);
    bool applyControlFault(int robotId, double &headPosition, double &neckPosition);
    
    /**
     * Helper methods for fault calculation and application
     */
    double calculateNoise(double baseValue, double intensity);
    FaultSeverity getSeverityFromIntensity(double intensity);
};

#endif // FAULT_MANAGER_HPP
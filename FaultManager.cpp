// FaultManager.cpp - Implements fault injection for robot testing
#include "FaultManager.hpp"
#include <iostream>

FaultManager::FaultManager() {
    mEnabled = true;
    currentTime = 0.0;
    std::srand(std::time(nullptr)); // Initialize random seed for fault generation
}

void FaultManager::scheduleFault(int robotId, FaultType type, FaultSeverity severity, 
                               double startTime, double duration, double intensity,
                               bool isIntermittent, double intermittentProb) {
    // Create and configure a new fault
    Fault fault;
    fault.robotId = robotId;
    fault.type = type;
    fault.severity = severity;
    fault.startTime = startTime;
    fault.duration = duration;
    fault.intensity = intensity;
    fault.isIntermittent = isIntermittent;
    fault.intermittentProb = intermittentProb;
    fault.isActive = false;
    
    // Add to scheduled faults list
    scheduledFaults.push_back(fault);
    std::cout << "Scheduled fault type " << type << " for robot " << robotId << std::endl;
}

void FaultManager::generateRandomFaults(int numRobots, double simulationDuration) {
    // Generate 1-3 random faults for system testing
    int numFaults = 1 + std::rand() % 3;
    
    for (int i = 0; i < numFaults; i++) {
        // Select random robot ID
        int robotId = std::rand() % numRobots;
        
        // Determine fault type with weighted distribution
        FaultType type;
        double typeRand = static_cast<double>(std::rand()) / RAND_MAX;
        if (typeRand < 0.3) type = BALL_SENSOR;
        else if (typeRand < 0.6) type = COMMUNICATION;
        else if (typeRand < 0.9) type = LOCOMOTION;
        else type = CONTROL_SYSTEM;
        
        // Determine severity with weighted distribution (biased toward medium)
        FaultSeverity severity;
        double sevRand = static_cast<double>(std::rand()) / RAND_MAX;
        if (sevRand < 0.3) severity = LOW;
        else if (sevRand < 0.7) severity = MEDIUM;
        else if (sevRand < 0.9) severity = HIGH;
        else severity = EXTREME;
        
        // Configure timing parameters
        double startTime = (static_cast<double>(std::rand()) / RAND_MAX) * simulationDuration * 0.8;
        double duration = simulationDuration * 0.05 + 
                        (static_cast<double>(std::rand()) / RAND_MAX) * simulationDuration * 0.25;
        
        // Configure intermittency parameters
        bool isIntermittent = (static_cast<double>(std::rand()) / RAND_MAX) < 0.3;
        double intermittentProb = isIntermittent ? 
                                0.2 + (static_cast<double>(std::rand()) / RAND_MAX) * 0.6 : 1.0;
        
        // Configure intensity parameter
        double intensity = static_cast<double>(std::rand()) / RAND_MAX;
        
        // Schedule the generated fault
        scheduleFault(robotId, type, severity, startTime, duration, 
                    intensity, isIntermittent, intermittentProb);
    }
}

void FaultManager::update(double timeStep) {
    // Update time and fault states
    currentTime += timeStep;
    
    for (auto &fault : scheduledFaults) {
        // Determine if fault should be active based on time window
        bool timeActive = (currentTime >= fault.startTime && 
                          currentTime <= fault.startTime + fault.duration);
        
        // Apply intermittency if configured
        if (timeActive && fault.isIntermittent) {
            double randomValue = static_cast<double>(std::rand()) / RAND_MAX;
            fault.isActive = randomValue < fault.intermittentProb;
        } else {
            fault.isActive = timeActive;
        }
        
        // Log state changes for monitoring
        static bool lastActive = false;
        if (fault.isActive != lastActive) {
            if (fault.isActive) {
                std::cout << "Fault activated for robot " << fault.robotId 
                          << " type: " << fault.type 
                          << " severity: " << fault.severity << std::endl;
            } else if (lastActive) {
                std::cout << "Fault deactivated for robot " << fault.robotId << std::endl;
            }
            lastActive = fault.isActive;
        }
    }
}

bool FaultManager::applyBallSensorFault(int robotId, double &x, double &y) {
    // Skip if fault system is disabled
    if (!mEnabled) return true;
    
    // Check for active faults affecting this robot's ball sensor
    for (const auto &fault : scheduledFaults) {
        if (fault.isActive && fault.robotId == robotId && fault.type == BALL_SENSOR) {
            // Complete sensor failure case
            if (fault.severity == EXTREME) {
                return false; // Ball not found
            }
            
            // Partial sensor failure - apply noise proportional to intensity
            double noiseX = calculateNoise(0.0, fault.intensity);
            double noiseY = calculateNoise(0.0, fault.intensity);
            
            x += noiseX;
            y += noiseY;
            return true;
        }
    }
    return true; // No fault, normal operation
}

bool FaultManager::applyLocomotionFault(int robotId, double &xAmplitude, double &yAmplitude, double &aAmplitude) {
    // Skip if fault system is disabled
    if (!mEnabled) return true;
    
    // Check for active faults affecting this robot's locomotion
    for (const auto &fault : scheduledFaults) {
        if (fault.isActive && fault.robotId == robotId && fault.type == LOCOMOTION) {
            // Complete locomotion failure case
            if (fault.severity == EXTREME) {
                xAmplitude = 0.0;
                yAmplitude = 0.0;
                aAmplitude = 0.0;
                return false;
            }
            
            // Partial locomotion degradation - scale by intensity
            double factor = 1.0 - fault.intensity;
            xAmplitude *= factor;
            yAmplitude *= factor;
            aAmplitude *= factor;
            return true;
        }
    }
    return true; // No fault, normal operation
}

bool FaultManager::applyControlFault(int robotId, double &headPosition, double &neckPosition) {
    // Skip if fault system is disabled
    if (!mEnabled) return true;
    
    // Check for active faults affecting this robot's control system
    for (const auto &fault : scheduledFaults) {
        if (fault.isActive && fault.robotId == robotId && fault.type == CONTROL_SYSTEM) {
            // Add tremor/noise to control signals
            double tremor = calculateNoise(0.0, fault.intensity);
            headPosition += tremor;
            neckPosition += tremor;
            return true;
        }
    }
    return true; // No fault, normal operation
}

double FaultManager::calculateNoise(double baseValue, double intensity) {
    // Generate Gaussian noise using Box-Muller transform
    double u1 = (static_cast<double>(std::rand()) / RAND_MAX);
    double u2 = (static_cast<double>(std::rand()) / RAND_MAX);
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    
    // Scale noise by intensity and apply calibration factor
    return z0 * intensity * 0.2;
}

FaultSeverity FaultManager::getSeverityFromIntensity(double intensity) {
    // Map continuous intensity values to discrete severity levels
    if (intensity < 0.25) return LOW;
    if (intensity < 0.5) return MEDIUM;
    if (intensity < 0.75) return HIGH;
    return EXTREME;
}
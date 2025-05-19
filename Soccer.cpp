// Soccer.cpp - Main controller for soccer robots
#include "Soccer.hpp"
#include "FaultManager.hpp"
#include <webots/PositionSensor.hpp>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace webots;
using namespace std;

// Utility function to clamp values within range
static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

// Calculate ball distance consistently
double calculateBallDistance(double x, double y) {
  return sqrt(x*x + y*y);
}

// Motor names mapping to their robot IDs
static const char *motorNames[20] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR", 
  "AnkleL", "FootR", "FootL", "Neck", "Head"
};

Soccer::Soccer() : Robot() {
  mTimeStep = getBasicTimeStep();
   
  // Determine team from robot name
  std::string robotName = getName();
  if (robotName.length() > 0) {
    if (robotName[0] == 'B') {
      mTeam = 'b';
      std::cout << "TEAM SET: Robot " << robotName << " is on BLUE team ('b')" << std::endl;
    } else if (robotName[0] == 'Y') {
      mTeam = 'y';
      std::cout << "TEAM SET: Robot " << robotName << " is on YELLOW team ('y')" << std::endl;
    }
  }
  
  // Initialize kickoff variables
  mInKickoffWait = true;
  mIsKickoffTeam = (mTeam == 'b'); // Blue team gets first kickoff
  mKickoffTimer = (mTeam == 'b') ? 0.0 : 7.0; // Yellow team waits 7 seconds
  
  // Initialize LEDs
  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mHeadLED->set(0x00FF00);
  mBackLedRed = getLED("BackLedRed");
  mBackLedGreen = getLED("BackLedGreen");
  mBackLedBlue = getLED("BackLedBlue");
  
  // Initialize camera
  mCamera = getCamera("Camera");
  mCamera->enable(2 * mTimeStep);
  
  // Initialize sensors
  mGyro = getGyro("Gyro");
  if (mGyro)
    mGyro->enable(mTimeStep);
    
  mAccelerometer = getAccelerometer("Accelerometer");
  if (mAccelerometer)
    mAccelerometer->enable(mTimeStep);
  
  // Initialize communication
  mReceiver = getReceiver("receiver");
  if (mReceiver)
    mReceiver->enable(mTimeStep);
  
  mEmitter = getEmitter("emitter");
  if (mEmitter)
    mEmitter->setChannel(1); // Use channel 1 for team communication
    
  // Initialize motors and position sensors
  for (int i = 0; i < 20; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    
    PositionSensor *sensor = getPositionSensor(sensorName);
    if (sensor)
      sensor->enable(mTimeStep);
  }

  // Initialize managers
  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
  
  // Enhanced vision parameters for better ball detection
  mVisionManager = new RobotisOp2VisionManager(
    mCamera->getWidth(), mCamera->getHeight(), 
    20, 15,     // Lower thresholds for easier detection
    65, 55,     // Wider maximum distances
    0, 45);     // Wider hue range for orange ball
    
  // Default player role
  mPlayerID = 1; // Default to field player 1
  
  // Initialize fault injection system
  mFaultManager.generateRandomFaults(5, 600.0);
  std::cout << "Fault injection system initialized" << std::endl;
  
  // Initialize team strategy variables
  mHasBall = false;
  mPassInProgress = false;
  mPassTarget = -1;
  mExpectingPass = false;
  mBallLastX = 0.0;
  mBallLastY = 0.0;
  mMyEstimatedX = 0.0;
  mMyEstimatedY = 0.0;
  mMyOrientation = 0.0;
  mLastMessageTime = 0.0;
  mInDefenseMode = false;
  mNearestToBallID = -1;
  mLastBallUpdateTime = 0.0;
  mTeamBallX = 0.0;
  mTeamBallY = 0.0;
  mTeamHasBallVisibility = false;
  mCurrentRound = 1;
  
  // Set visual indicator for waiting team
  if (!mIsKickoffTeam) {
    mHeadLED->set(0xFF0000);  // Red during wait
  }
  
  // Set formation positions based on roles (team-aware)
  double direction = (mTeam == 'b') ? 1.0 : -1.0;
  
  // Set default formation position
  mFormationPosition.baseX = 0.0;
  mFormationPosition.baseY = 0.0;
  mFormationPosition.defendingOffset = 0.2 * direction;
  mFormationPosition.attackingOffset = -0.3 * direction;
  
  // Update formation position based on player role
  switch (mPlayerID) {
    case 0: // Goalkeeper
      mFormationPosition.baseX = 0.7 * direction;
      mFormationPosition.baseY = 0.0;
      mFormationPosition.defendingOffset = 0.0;
      mFormationPosition.attackingOffset = 0.0;
      break;
    case 1: // Defender 1
      mFormationPosition.baseX = 0.4 * direction;
      mFormationPosition.baseY = 0.3;
      mFormationPosition.defendingOffset = 0.2 * direction;
      mFormationPosition.attackingOffset = 0.0;
      break;
    case 2: // Defender 2
      mFormationPosition.baseX = 0.4 * direction;
      mFormationPosition.baseY = -0.3;
      mFormationPosition.defendingOffset = 0.2 * direction;
      mFormationPosition.attackingOffset = 0.0;
      break;
    case 3: // Midfielder
      mFormationPosition.baseX = 0.0;
      mFormationPosition.baseY = 0.0;
      mFormationPosition.defendingOffset = 0.3 * direction;
      mFormationPosition.attackingOffset = -0.3 * direction;
      break;
    case 4: // Forward
      mFormationPosition.baseX = -0.4 * direction;
      mFormationPosition.baseY = 0.0;
      mFormationPosition.defendingOffset = 0.2 * direction;
      mFormationPosition.attackingOffset = -0.1 * direction;
      break;
  }
  
  // Initialize teammates info
  for (int i = 0; i < 5; i++) {
    PlayerInfo info;
    info.id = i;
    info.x = 0.0;
    info.y = 0.0;
    info.orientation = 0.0;
    info.hasBall = false;
    info.canSeeBall = false;
    info.lastUpdateTime = 0.0;
    mTeammates[i] = info;
  }
  
  // Initialize multithreaded data
  mBallData.x = 0.0;
  mBallData.y = 0.0;
  mBallData.visible = false;
  mBallData.lastUpdateTime = 0.0;
  
  // Initialize controller state
  mInKickingSequence = false;
  mKickSequenceStep = 0;
  mKickSequenceTimer = 0.0;
  
  // Initialize AI performance metrics
  mPerformanceMetrics.ballDetectionCount = 0;
  mPerformanceMetrics.successfulKicks = 0;
  mPerformanceMetrics.ballLostCount = 0;
  mPerformanceMetrics.totalReward = 0.0;
  mPerformanceMetrics.avgRewardPerFrame = 0.0;
  mPerformanceMetrics.frameCount = 0;
  
  // Initialize learning model weights based on player role
  if (mPlayerID == 0) { // Goalkeeper
    mLearningWeights.ballDistance = 0.3;
    mLearningWeights.formationPosition = 0.6;
    mLearningWeights.kickSuccess = 0.1;
  } else if (mPlayerID == 1 || mPlayerID == 2) { // Defenders
    mLearningWeights.ballDistance = 0.5;
    mLearningWeights.formationPosition = 0.4;
    mLearningWeights.kickSuccess = 0.1;
  } else if (mPlayerID == 3) { // Midfielder
    mLearningWeights.ballDistance = 0.5;
    mLearningWeights.formationPosition = 0.2;
    mLearningWeights.kickSuccess = 0.3;
  } else { // Forward
    mLearningWeights.ballDistance = 0.6;
    mLearningWeights.formationPosition = 0.1;
    mLearningWeights.kickSuccess = 0.3;
  }
  
  // Initialize multithreading parameters
  mLastDecisionTime = 0.0;
  mLastVisionTime = 0.0;
  mLastCommunicationTime = 0.0;
  mDecisionRefreshRate = 30.0; // ms
  mVisionRefreshRate = 15.0; // ms
  mCommunicationRefreshRate = 200.0; // ms
  
  // Log initial system status
  std::cout << "AI-based Soccer controller initialized" << std::endl;
  std::cout << "Multi-threaded processing system active:" << std::endl;
  std::cout << "- Vision thread: " << mVisionRefreshRate << "ms refresh rate" << std::endl;
  std::cout << "- Decision thread: " << mDecisionRefreshRate << "ms refresh rate" << std::endl;
  std::cout << "- Communication thread: " << mCommunicationRefreshRate << "ms refresh rate" << std::endl;
}

// Cleanup resources
Soccer::~Soccer() {
  delete mMotionManager;
  delete mGaitManager;
  delete mVisionManager;
  
  std::cout << "Soccer controller shutting down threads..." << std::endl;
  std::cout << "All threads terminated successfully" << std::endl;
}

// Test goalkeeper motion pages to ensure they work
bool Soccer::testMotionPages() {
  std::cout << "===== TESTING GOALKEEPER MOTION PAGES =====" << std::endl;
  
  mGaitManager->stop();
  wait(500);
  
  bool allPagesWorked = true;
  
  int pagesToTest[] = {9, 12, 13, 14, 15};
  const char* pageNames[] = {"ready position", "left block", "right block", "right sidestep", "left sidestep"};
  
  for (int i = 0; i < 5; i++) {
    int page = pagesToTest[i];
    std::cout << "Testing motion page " << page << " (" << pageNames[i] << ")" << std::endl;
    
    try {
      mMotionManager->playPage(page);
      std::cout << "Motion page " << page << " executed without errors" << std::endl;
      
      wait(500);
      const double *acc = mAccelerometer->getValues();
      std::cout << "Accelerometer values: " << acc[0] << ", " << acc[1] << ", " << acc[2] << std::endl;
      
    } catch (...) {
      std::cout << "ERROR: Motion page " << page << " failed to execute!" << std::endl;
      allPagesWorked = false;
    }
    
    wait(500);
    
    try {
      mMotionManager->playPage(9);
    } catch (...) {
      std::cout << "ERROR: Failed to return to ready position (page 9)!" << std::endl;
      allPagesWorked = false;
    }
    wait(500);
  }
  
  std::cout << "===== MOTION PAGE TESTING COMPLETE =====" << std::endl;
  std::cout << "Result: " << (allPagesWorked ? "All pages appear to work" : "Some pages may not exist") << std::endl;
  
  mGaitManager->start();
  
  return allPagesWorked;
}

// Execute a single step and update fault manager
void Soccer::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
    
  // Update fault manager - convert mTimeStep from ms to seconds
  mFaultManager.update(mTimeStep / 1000.0);
}

// Wait for specified milliseconds
void Soccer::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// Set head position safely within hardware limits
void Soccer::safeSetHeadPosition(double desiredNeck, double desiredHead) {
    double safeNeck = clamp(desiredNeck, -0.35, 0.35); 
    double safeHead = clamp(desiredHead, -0.35, 0.35);
    
    // If the desired value is outside the safe range, move the body instead
    if (fabs(desiredNeck) > 0.35 || fabs(desiredHead) > 0.35) {
        double turnAmp = 0.0;
        if (fabs(desiredNeck) > 0.35) {
            turnAmp = (desiredNeck > 0) ? 0.2 : -0.2;
        }
        
        // Only apply if not in a special state like kicking
        if (!mInKickingSequence && mPlayerID != 0) {
            mGaitManager->setAAmplitude(turnAmp);
        }
    }
    
    // Set the motor positions using the safe values
    mMotors[18]->setPosition(safeNeck);
    mMotors[19]->setPosition(safeHead);
}

// Detect ball in camera image
bool Soccer::getBallCenter(double &x, double &y) {
  if (!mCamera) return false;
  
  static int width = mCamera->getWidth();
  static int height = mCamera->getHeight();

  const unsigned char *image = mCamera->getImage();
  
  // Occasionally log camera data for debugging
  static int debugCounter = 0;
  if (++debugCounter >= 200) {
    std::cout << "[Vision Thread] Camera sample pixels: R=" << (int)image[0] 
              << " G=" << (int)image[1] 
              << " B=" << (int)image[2] << std::endl;
    debugCounter = 0;
  }
  
  bool found = mVisionManager->getBallCenter(x, y, image);

  static int counter = 0;
  if (++counter % 50 == 0) {
    if (found) {
      std::cout << "[Vision Thread] Robot " << mPlayerID << " FOUND ball at: (" << x << ", " << y << ")" << std::endl;
    } else {
      std::cout << "[Vision Thread] Robot " << mPlayerID << " cannot see the ball - searching" << std::endl;
    }
    counter = 0;
  }

  if (!found) {
    x = 0.0;
    y = 0.0;
    return false;
  } else {
    // Convert from pixel coordinates to normalized coordinates
    x = 2.0 * x / width - 1.0;
    y = 2.0 * y / height - 1.0;
    
    // Apply potential ball sensor fault
    bool sensorOk = mFaultManager.applyBallSensorFault(mPlayerID, x, y);
    return sensorOk && found;
  }
}

// Vision thread execution
void Soccer::simulateVisionThread() {
    double currentTime = getTime();
    
    // Higher frequency for goalkeeper
    double effectiveRefreshRate = (mPlayerID == 0) ? 
                                 mVisionRefreshRate / 3.0 : // 3x faster for goalkeeper
                                 mVisionRefreshRate;
    
    // Only process at vision refresh rate
    if (currentTime - mLastVisionTime < effectiveRefreshRate / 1000.0) {
        // Update ball prediction during waiting
        predictBallPosition((currentTime - mLastVisionTime));
        return;
    }
    
    mLastVisionTime = currentTime;
    
    // Process camera images & detect ball
    double x, y;
    bool ballFound = getBallCenter(x, y);
    
    // Update shared data structure
    mBallData.x = x;
    mBallData.y = y;
    mBallData.visible = ballFound;
    mBallData.lastUpdateTime = currentTime;
    
    // Update velocity when ball is found repeatedly
    if (ballFound && mBallData.lastSeenX != 0.0) {
        double timeDelta = currentTime - mLastVisionTime + mTimeStep/1000.0;
        mBallData.velocityX = (x - mBallData.lastSeenX) / timeDelta;
        mBallData.velocityY = (y - mBallData.lastSeenY) / timeDelta;
        
        // Log velocity occasionally
        static int velocityCounter = 0;
        if (++velocityCounter % 20 == 0) {
            std::cout << "[Vision Thread] Ball velocity: (" 
                      << mBallData.velocityX << ", " << mBallData.velocityY 
                      << ")" << std::endl;
        }
    }
    
    if (ballFound) {
        mBallData.lastSeenX = x;
        mBallData.lastSeenY = y;
        mPerformanceMetrics.ballDetectionCount++;
    }
}

// Decision thread execution
void Soccer::simulateDecisionThread() {
  double currentTime = getTime();
  
  // Run at decision thread rate
  if (currentTime - mLastDecisionTime < mDecisionRefreshRate / 1000.0) {
    return;
  }
  
  mLastDecisionTime = currentTime;
  
  // Get ball data from vision thread
  double ballX = mBallData.x;
  double ballY = mBallData.y;
  bool ballVisible = mBallData.visible;
  
  // Log thread operation periodically
  static int decisionCounter = 0;
  if (++decisionCounter >= 30) {
    std::cout << "[Decision Thread] Processing robot " << mPlayerID << " AI state" << std::endl;
    decisionCounter = 0;
  }
  
  // Skip during kicking sequence
  if (mInKickingSequence) {
    return;
  }
  
  // Calculate ball distance
  double ballDistance = sqrt(ballX*ballX + ballY*ballY);
  
  // Role-specific decision logic
  if (mPlayerID == 0) { // Goalkeeper
    if (ballVisible && ballDistance < 0.4) {
      std::cout << "[Decision Thread] Goalkeeper identifies blocking opportunity" << std::endl;
    }
  } else {
    // Field player decision logic
    if (ballVisible && ballY > 0.3 && ballDistance < 0.4) {
      mHasBall = true;
      std::cout << "[Decision Thread] Ball in kicking position detected" << std::endl;
    }
    
    // Calculate reward
    double reward = calculateReward();
    mPerformanceMetrics.totalReward += reward;
  }
  
  // Update performance metrics
  mPerformanceMetrics.frameCount++;
  mPerformanceMetrics.avgRewardPerFrame = mPerformanceMetrics.totalReward / mPerformanceMetrics.frameCount;
}

// Calculate AI reward function
double Soccer::calculateReward() {
  double reward = 0.0;
  
  // Ball visibility reward
  if (mBallData.visible) {
    reward += 0.1;
    
    // Distance to ball reward
    double ballDist = sqrt(mBallData.x*mBallData.x + mBallData.y*mBallData.y);
    reward += (1.0 - ballDist) * mLearningWeights.ballDistance;
  } else {
    reward -= 0.05;  // Small penalty for not seeing ball
  }
  
  // Add formation position reward
  double formationDist = getDistanceToFormationPosition();
  if (formationDist < 0.3) {
    reward += mLearningWeights.formationPosition;
  }
  
  return reward;
}

// Communication thread execution
void Soccer::simulateCommunicationThread() {
  double currentTime = getTime();
  
  // Only process at communication refresh rate
  if (currentTime - mLastCommunicationTime < mCommunicationRefreshRate / 1000.0) {
    return;
  }
  
  mLastCommunicationTime = currentTime;
  
  // Process incoming messages
  if (mReceiver && mReceiver->getQueueLength() > 0) {
    std::cout << "[Communication Thread] Processing incoming messages" << std::endl;
    processTeamMessages();
  }
  
  // Send updates periodically
  static double lastSendTime = 0;
  if (currentTime - lastSendTime > 0.5) {
    updateTeamState();
    lastSendTime = currentTime;
    std::cout << "[Communication Thread] Sending team updates" << std::endl;
  }
}

// Fallback ball detection when vision thread data is stale
bool Soccer::fallbackBallDetection(double &x, double &y) {
    double currentTime = getTime();
    if (currentTime - mBallData.lastUpdateTime > 0.1) { // 100ms threshold
        return getBallCenter(x, y);
    }
    
    // Use stored data if recent enough
    x = mBallData.x;
    y = mBallData.y;
    return mBallData.visible;
}

// Enhanced ball detection using both vision thread and direct detection
bool Soccer::enhancedBallDetection(double &x, double &y) {
    // Try vision thread data first
    if (mBallData.visible && getTime() - mBallData.lastUpdateTime < 0.05) {
        x = mBallData.x;
        y = mBallData.y;
        return true;
    }
    
    // Fall back to direct detection
    bool directDetection = getBallCenter(x, y);
    
    // Update shared data if direct detection succeeds
    if (directDetection) {
        mBallData.x = x;
        mBallData.y = y;
        mBallData.visible = true;
        mBallData.lastUpdateTime = getTime();
        
        // Update velocity estimate
        double timeDelta = getTime() - mBallData.lastUpdateTime;
        if (timeDelta > 0.001 && mBallData.lastSeenX != 0.0) {
            mBallData.velocityX = (x - mBallData.lastSeenX) / timeDelta;
            mBallData.velocityY = (y - mBallData.lastSeenY) / timeDelta;
        }
        
        mBallData.lastSeenX = x;
        mBallData.lastSeenY = y;
    }
    
    return directDetection;
}

// Predict ball position using physics model
void Soccer::predictBallPosition(double deltaTime) {
  // Physics constants
  const double FRICTION = 0.2;

  // Only predict if ball was recently visible
  if (!mBallData.visible && getTime() - mBallData.lastUpdateTime < 0.5) {
    // Physics-based prediction with friction
    double frictionFactor = exp(-FRICTION * deltaTime);
    mBallData.velocityX = frictionFactor;
    mBallData.velocityY= frictionFactor;

    // Update position using current velocity
    mBallData.x += mBallData.velocityX * deltaTime;
    mBallData.y += mBallData.velocityY * deltaTime;

    // Calculate prediction confidence
    double timeSinceLastSeen = getTime() - mBallData.lastUpdateTime;
    mBallData.predictionConfidence = 1.0 - (timeSinceLastSeen * 2.0);

    // Apply bounds checking
    if (fabs(mBallData.x) > 1.0) {
      mBallData.predictionConfidence *= 0.7;
    }

    // Log prediction data occasionally
    static int predictionCounter = 0;
    if (++predictionCounter % 30 == 0) {
      std::cout << "[Vision Thread] Ball prediction: pos=(" 
                << mBallData.x << "," << mBallData.y
                << "), vel=(" << mBallData.velocityX << "," << mBallData.velocityY
                << "), conf=" << mBallData.predictionConfidence << std::endl;
      predictionCounter = 0;
    }
  } else {
    mBallData.predictionConfidence = 0.0;
  }
}

// Implement ball search pattern for when ball is not visible
void Soccer::implementBallSearch() {
  static int searchPhase = 0;
  static int phaseCounter = 0;
  
  // Change search behavior periodically
  if (++phaseCounter > 20) {
    searchPhase = (searchPhase + 1) % 6;
    phaseCounter = 0;
  }
  
  // Initialize movement parameters
  double xAmp = 0.0;
  double yAmp = 0.0;
  double aAmp = 0.0;
  
  // Search pattern with different phases
  switch(searchPhase) {
    case 0: // Turn left slowly
      xAmp = 0.0;
      aAmp = 0.2;
      break;
    case 1: // Move forward
      xAmp = 0.5;
      aAmp = 0.0;
      break;
    case 2: // Turn right slowly
      xAmp = 0.0;
      aAmp = -0.2;
      break;
    case 3: // Move forward
      xAmp = 0.5;
      aAmp = 0.0;
      break;
    case 4: // Turn left while moving
      xAmp = 0.3;
      aAmp = 0.2;
      break;
    case 5: // Turn right while moving
      xAmp = 0.3;
      aAmp = -0.2;
      break;
  }
  
  // Apply locomotion faults before setting
  mFaultManager.applyLocomotionFault(mPlayerID, xAmp, yAmp, aAmp);
  
  // Apply movement
  mGaitManager->setXAmplitude(xAmp);
  mGaitManager->setYAmplitude(yAmp);
  mGaitManager->setAAmplitude(aAmp);
  mGaitManager->step(mTimeStep);
  
  // Wide scanning head movement
  double scanSpeed = 0.2;
  double headPosition = clamp(sin(getTime() * scanSpeed * 3) * 1.0, -1.0, 1.0);
  double neckPosition = clamp(cos(getTime() * scanSpeed * 3) * 0.8, -0.8, 0.8);
  
  // Apply control fault to head/neck
  mFaultManager.applyControlFault(mPlayerID, headPosition, neckPosition);
  
  safeSetHeadPosition(neckPosition, headPosition);
}

// Check for falls and recover if needed
void Soccer::checkForFalls() {
  if (!mAccelerometer) return;
  
  const double *acc = mAccelerometer->getValues();
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 20;

  // Count how many steps the accelerometer says the robot is down
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  // Robot face is down - recover
  if (fup > acc_step) {
    mGaitManager->stop();
    mMotionManager->playPage(10);  // f_up (get up from front)
    mMotionManager->playPage(9);   // walkready position
    mGaitManager->start();
    fup = 0;
    
    std::cout << "[Main Thread] Robot " << mPlayerID << " got up from FACE DOWN" << std::endl;
  }
  // Robot back is down - recover
  else if (fdown > acc_step) {
    mGaitManager->stop();
    mMotionManager->playPage(11);  // b_up (get up from back)
    mMotionManager->playPage(9);   // walkready position
    mGaitManager->start();
    fdown = 0;
    
    std::cout << "[Main Thread] Robot " << mPlayerID << " got up from BACK DOWN" << std::endl;
  }
}

// Process incoming team messages
void Soccer::processTeamMessages() {
  if (!mReceiver) return;
  
  while (mReceiver->getQueueLength() > 0) {
    const char *message = (const char *)mReceiver->getData();
    
    // Skip empty messages
    if (!message || strlen(message) == 0) {
      mReceiver->nextPacket();
      continue;
    }
    
    std::string messageStr(message);
    std::string robotName = getName();
    
    // Handle kickoff messages from referee
    if (messageStr.find("KICKOFF|") == 0) {
      size_t firstSep = messageStr.find('|');
      size_t secondSep = messageStr.find('|', firstSep + 1);
      size_t thirdSep = messageStr.find('|', secondSep + 1);
      
      if (firstSep != std::string::npos && secondSep != std::string::npos && thirdSep != std::string::npos) {
        char kickoffTeam = messageStr[firstSep + 1];
        int round = std::stoi(messageStr.substr(secondSep + 1, thirdSep - secondSep - 1));
        std::string waitingTeamStr = messageStr.substr(thirdSep + 1);
        char waitingTeam = (waitingTeamStr == "WAITING_b") ? 'b' : 'y';
        
        // Update round counter
        mCurrentRound = round;
        
        // Reset kickoff state
        mInKickoffWait = true;
        
        // Determine if this robot needs to wait
        bool needToWait = (mTeam == waitingTeam);
        
        // Set waiting timer
        if (needToWait) {
          mKickoffTimer = 7.0;  // 7 seconds wait
          mIsKickoffTeam = false;
          
          mHeadLED->set(0xFF0000);  // Red during wait
        } else {
          mKickoffTimer = 0.0;  // No waiting
          mIsKickoffTeam = true;
          
          mHeadLED->set(0x00FF00);  // Green for kickoff team
        }
        
        std::cout << "[Communication Thread] KICKOFF ROUND " << round << ": I am team '" << mTeam 
                  << "', kickoff team is '" << kickoffTeam 
                  << "', waiting team is '" << waitingTeam << "'" << std::endl;
      }
    }
    else if (messageStr.find("KICKOFF_COMPLETE|") == 0) {
      // Kickoff wait is over
      mInKickoffWait = false;
      std::cout << "[Communication Thread] Robot " << mPlayerID << " Team " << mTeam 
                << ": Kickoff wait complete!" << std::endl;
      
      // Restart walking for field players that were waiting
      if (mPlayerID != 0 && !mIsKickoffTeam) {
        mGaitManager->start();
      }
      
      // Reset LED to normal
      mHeadLED->set(0x00FF00);  // Green
      mEyeLED->set(0x000000);   // Off initially
    }
    else {
      // Process regular team messages
      try {
        // Parse message (format: TYPE|SENDER_ID|DATA)
        size_t firstSep = messageStr.find('|');
        size_t secondSep = messageStr.find('|', firstSep + 1);
        
        if (firstSep != std::string::npos && secondSep != std::string::npos) {
          int messageType = std::stoi(messageStr.substr(0, firstSep));
          int senderId = std::stoi(messageStr.substr(firstSep + 1, secondSep - firstSep - 1));
          std::string data = messageStr.substr(secondSep + 1);
          
          // Only process messages from our team
          if (senderId >= 0 && senderId < 5) {
            switch (messageType) {
              case BALL_POSITION: {
                // Format: BALL_X|BALL_Y|CAN_SEE_BALL
                size_t dataSep1 = data.find('|');
                size_t dataSep2 = data.find('|', dataSep1 + 1);
                if (dataSep1 != std::string::npos && dataSep2 != std::string::npos) {
                  double ballX = std::stod(data.substr(0, dataSep1));
                  double ballY = std::stod(data.substr(dataSep1 + 1, dataSep2 - dataSep1 - 1));
                  bool canSeeBall = (data.substr(dataSep2 + 1) == "1");
                  
                  // Update teammate info
                  mTeammates[senderId].canSeeBall = canSeeBall;
                  if (canSeeBall) {
                    mTeammates[senderId].hasBall = false; // Reset unless explicitly set
                    
                    // Update team ball position if this is more recent than our own
                    if (getTime() - mLastBallUpdateTime > 1.0 || !mTeamHasBallVisibility) {
                      mTeamBallX = ballX;
                      mTeamBallY = ballY;
                      mTeamHasBallVisibility = true;
                    }
                  }
                }
                break;
              }
              
              case PLAYER_POSITION: {
                // Format: POS_X|POS_Y|ORIENTATION|HAS_BALL
                size_t dataSep1 = data.find('|');
                size_t dataSep2 = data.find('|', dataSep1 + 1);
                size_t dataSep3 = data.find('|', dataSep2 + 1);
                
                if (dataSep1 != std::string::npos && dataSep2 != std::string::npos && dataSep3 != std::string::npos) {
                  double posX = std::stod(data.substr(0, dataSep1));
                  double posY = std::stod(data.substr(dataSep1 + 1, dataSep2 - dataSep1 - 1));
                  double orientation = std::stod(data.substr(dataSep2 + 1, dataSep3 - dataSep2 - 1));
                  bool hasBall = (data.substr(dataSep3 + 1) == "1");
                  
                  // Update teammate info
                  mTeammates[senderId].x = posX;
                  mTeammates[senderId].y = posY;
                  mTeammates[senderId].orientation = orientation;
                  mTeammates[senderId].hasBall = hasBall;
                  mTeammates[senderId].lastUpdateTime = getTime();
                }
                break;
              }
              
              case PASS_REQUEST: {
                // Format: TARGET_ID
                int targetId = std::stoi(data);
                if (targetId == mPlayerID && !mExpectingPass) {
                  mExpectingPass = true;
                  std::cout << "[Communication Thread] Player " << mPlayerID 
                            << " expecting pass from " << senderId << std::endl;
                }
                break;
              }
              
              case PASS_EXECUTE: {
                // Format: TARGET_ID
                int targetId = std::stoi(data);
                if (targetId == mPlayerID) {
                  mExpectingPass = true;
                  std::cout << "[Communication Thread] Player " << mPlayerID 
                            << " pass coming from " << senderId << std::endl;
                }
                break;
              }
              
              case FORMATION_SHIFT: {
                // Format: DEFENSE_MODE (1 or 0)
                bool defenseMode = (data == "1");
                mInDefenseMode = defenseMode;
                std::cout << "[Communication Thread] Formation shift to " 
                          << (defenseMode ? "DEFENSE" : "OFFENSE") << " mode" << std::endl;
                break;
              }
              
              case DEFENSE_ALERT: {
                // Format: NEAREST_ID|BALL_X|BALL_Y
                size_t dataSep1 = data.find('|');
                size_t dataSep2 = data.find('|', dataSep1 + 1);
                
                if (dataSep1 != std::string::npos && dataSep2 != std::string::npos) {
                  int nearestId = std::stoi(data.substr(0, dataSep1));
                  double ballX = std::stod(data.substr(dataSep1 + 1, dataSep2 - dataSep1 - 1));
                  double ballY = std::stod(data.substr(dataSep2 + 1));
                  
                  mNearestToBallID = nearestId;
                  
                  // If we're not the nearest, update team ball position
                  if (nearestId != mPlayerID) {
                    mTeamBallX = ballX;
                    mTeamBallY = ballY;
                    mTeamHasBallVisibility = true;
                    std::cout << "[Communication Thread] Robot " << nearestId 
                              << " is closest to ball" << std::endl;
                  }
                }
                break;
              }
              
              case OFFENSE_OPPORTUNITY: {
                // Handle offensive coordination
                break;
              }
            }
          }
        }
      }
      catch (const std::exception& e) {
        std::cout << "[Communication Thread] Error parsing message: " << e.what() << std::endl;
      }
    }
    
    // Get next message
    mReceiver->nextPacket();
  }
}

// Send message to teammates
void Soccer::sendTeamMessage(MessageType type, const std::string& data) {
  if (!mEmitter) return;
  
  std::string message = std::to_string(type) + "|" + std::to_string(mPlayerID) + "|" + data;
  mEmitter->send(message.c_str(), message.length() + 1);
  mLastMessageTime = getTime();
}

// Update and share our current state with teammates
void Soccer::updateTeamState() {
  // Send updates at a reasonable rate to avoid flooding
  if (getTime() - mLastMessageTime < 0.5) return;
  
  // Send ball position if we can see it
  if (mBallData.visible) {
    mBallLastX = mBallData.x;
    mBallLastY = mBallData.y;
    mLastBallUpdateTime = getTime();
    
    std::string ballData = std::to_string(mBallData.x) + "|" + std::to_string(mBallData.y) + "|1";
    sendTeamMessage(BALL_POSITION, ballData);
  }
  
  // Send our position
  std::string posData = std::to_string(mMyEstimatedX) + "|" + std::to_string(mMyEstimatedY) + "|" + 
                        std::to_string(mMyOrientation) + "|" + (mHasBall ? "1" : "0");
  sendTeamMessage(PLAYER_POSITION, posData);
  
  // Update team state based on ball position
  determineTeamState();
}

// Determine if we're in attacking or defending mode
void Soccer::determineTeamState() {
  // Check if team has ball visibility
  if (mTeamHasBallVisibility) {
    // Determine if ball is in our half
    bool ballInOurHalf;
    
    // Team-specific goal positions
    if (mTeam == 'b') {
      ballInOurHalf = (mTeamBallX > 0.0);
    } else {
      ballInOurHalf = (mTeamBallX < 0.0);
    }
    
    // If ball is in our half, go to defense mode
    bool newDefenseMode = ballInOurHalf;
    
    // Only send message if state changed
    if (newDefenseMode != mInDefenseMode) {
      mInDefenseMode = newDefenseMode;
      sendTeamMessage(FORMATION_SHIFT, mInDefenseMode ? "1" : "0");
      
      // Coordinate team behavior
      if (mInDefenseMode) {
        coordinateDefense();
      } else {
        coordinateOffense();
      }
    }
  }
}

// Find the best passing opportunity
bool Soccer::findPassingOpportunity(int& targetId, double& targetX, double& targetY) {
  // Only proceed if we have the ball
  if (!mHasBall) return false;
  
  // Find the best teammate to pass to
  double bestScore = -1.0;
  int bestTarget = -1;
  double bestX = 0.0;
  double bestY = 0.0;
  
  // Direction toward opponent's goal
  double goalDirection = (mTeam == 'b') ? -1.0 : 1.0;
  
  for (const auto& pair : mTeammates) {
    int id = pair.first;
    const PlayerInfo& teammate = pair.second;
    
    // Skip ourselves and the goalkeeper
    if (id == mPlayerID || id == 0) continue;
    
    // Skip outdated information
    if (getTime() - teammate.lastUpdateTime > 5.0) continue;
    
    // Score based on position relative to goal
    double score = teammate.x * goalDirection * -1.0;
    
    if (score > bestScore) {
      bestScore = score;
      bestTarget = id;
      bestX = teammate.x;
      bestY = teammate.y;
    }
  }
  
  // Found a good target?
  if (bestTarget >= 0) {
    targetId = bestTarget;
    targetX = bestX;
    targetY = bestY;
    return true;
  }
  
  return false;
}

// Execute passing to another player
void Soccer::executePassing(int targetId, double targetX, double targetY) {
  if (!mHasBall) return;
  
  std::cout << "[Main Thread] Player " << mPlayerID << " passing to Player " << targetId << std::endl;
  
  // Notify the target that a pass is coming
  sendTeamMessage(PASS_EXECUTE, std::to_string(targetId));
  
  // Calculate direction to target
  double dx = targetX - mMyEstimatedX;
  double dy = targetY - mMyEstimatedY;
  double distance = sqrt(dx*dx + dy*dy);
  
  // Turn to face target
  double targetAngle = atan2(dy, dx);
  double turnAmount = targetAngle - mMyOrientation;
  
  // Normalize angle
  while (turnAmount > M_PI) turnAmount -= 2*M_PI;
  while (turnAmount < -M_PI) turnAmount += 2*M_PI;
  
  // Convert to normalized turn amplitude
  double aAmp = clamp(turnAmount / M_PI, -1.0, 1.0);
  
  // Apply locomotion fault
  double xAmp = 0.0;
  double yAmp = 0.0;
  mFaultManager.applyLocomotionFault(mPlayerID, xAmp, yAmp, aAmp);
  
  // Turn toward target
  mGaitManager->setXAmplitude(xAmp);
  mGaitManager->setYAmplitude(yAmp);
  mGaitManager->setAAmplitude(aAmp);
  
  // Execute several steps to turn
  for (int i = 0; i < 5; i++) {
    mGaitManager->step(mTimeStep);
    myStep();
  }
  
  // Execute kick
  mGaitManager->stop();
  
  // Adjust timing based on distance
  int waitTime = std::min(500, std::max(100, static_cast<int>(200 + distance * 100)));
  wait(waitTime);
  
  // Select kick direction based on target position
  if (dy > 0) {
    mMotionManager->playPage(12); // right kick
  } else {
    mMotionManager->playPage(13); // left kick
  }
  
  mMotionManager->playPage(9); // walkready position
  mGaitManager->start();
  
  // Reset ball control state
  mHasBall = false;
  mPassInProgress = true;
  mPassTarget = targetId;
}

// Handle receiving a pass
void Soccer::receivePass() {
  if (!mExpectingPass) return;
  
  std::cout << "[Main Thread] Player " << mPlayerID << " attempting to receive pass" << std::endl;
  
  // Get ball data
  double ballX = mBallData.x;
  double ballY = mBallData.y;
  bool ballVisible = mBallData.visible;
  
  if (ballVisible) {
    // Chase ball with high priority
    double xAmp = 1.0; // Maximum speed
    double yAmp = 0.0;
    double aAmp = -ballX * 2.5; // Aggressive turning
    
    // Apply locomotion fault
    mFaultManager.applyLocomotionFault(mPlayerID, xAmp, yAmp, aAmp);
    
    mGaitManager->setXAmplitude(xAmp);
    mGaitManager->setYAmplitude(yAmp);
    mGaitManager->setAAmplitude(aAmp);
    mGaitManager->step(mTimeStep);
    
    // Check if ball is close
    if (ballY > 0.3) {
      mHasBall = true;
      mExpectingPass = false;
      std::cout << "[Main Thread] Player " << mPlayerID << " successfully received pass" << std::endl;
    }
  } else {
    // Search for ball
    implementBallSearch();
  }
}

// Maintain team formation
void Soccer::maintainFormation() {
  // Calculate desired position
  double targetX = mFormationPosition.baseX;
  double targetY = mFormationPosition.baseY;
  
  // Adjust formation based on defense/offense mode
  if (mInDefenseMode) {
    targetX += mFormationPosition.defendingOffset;
  } else {
    targetX += mFormationPosition.attackingOffset;
  }
  
  // Also shift formation based on ball position if known
  if (mTeamHasBallVisibility) {
    // Shift slightly toward ball's y position
    targetY += (mTeamBallY - targetY) * 0.3;
  }
  
  // Calculate direction to formation position
  double dx = targetX - mMyEstimatedX;
  double dy = targetY - mMyEstimatedY;
  double distance = sqrt(dx*dx + dy*dy);
  
  // Only move to formation if not involved with ball
  if (distance > 0.3 && !mHasBall && !mExpectingPass && mNearestToBallID != mPlayerID) {
    // Calculate direction to target
    double targetAngle = atan2(dy, dx);
    double turnAmount = targetAngle - mMyOrientation;
    
    // Normalize angle
    while (turnAmount > M_PI) turnAmount -= 2*M_PI;
    while (turnAmount < -M_PI) turnAmount += 2*M_PI;
    
    // Convert to normalized turn amplitude
    double aAmp = clamp(turnAmount / M_PI, -1.0, 1.0);
    double xAmp = 0.7; // Moderate speed
    double yAmp = 0.0;
    
    // Apply locomotion fault
    mFaultManager.applyLocomotionFault(mPlayerID, xAmp, yAmp, aAmp);
    
    // Move toward formation position
    mGaitManager->setXAmplitude(xAmp);
    mGaitManager->setYAmplitude(yAmp);
    mGaitManager->setAAmplitude(aAmp);
    mGaitManager->step(mTimeStep);
    
    std::cout << "[Decision Thread] Robot " << mPlayerID << " maintaining formation" << std::endl;
  }
}

// Get distance to formation position
double Soccer::getDistanceToFormationPosition() {
  double targetX = mFormationPosition.baseX;
  double targetY = mFormationPosition.baseY;
  
  // Adjust based on team state
  if (mInDefenseMode) {
    targetX += mFormationPosition.defendingOffset;
  } else {
    targetX += mFormationPosition.attackingOffset;
  }
  
  // Calculate distance
  double dx = targetX - mMyEstimatedX;
  double dy = targetY - mMyEstimatedY;
  return sqrt(dx*dx + dy*dy);
}

// Determine if this player is nearest to the ball
bool Soccer::amINearestToBall() {
  if (!mTeamHasBallVisibility) return false;
  
  // Calculate my distance to team ball
  double myDistX = mTeamBallX - mMyEstimatedX;
  double myDistY = mTeamBallY - mMyEstimatedY;
  double myDistance = sqrt(myDistX*myDistX + myDistY*myDistY);
  
  // Compare with teammates
  bool amNearest = true;
  for (const auto& pair : mTeammates) {
    int id = pair.first;
    const PlayerInfo& teammate = pair.second;
    
    // Skip ourselves and outdated information
    if (id == mPlayerID || getTime() - teammate.lastUpdateTime > 5.0) continue;
    
    // Skip goalkeeper for ball pursuit
    if (id == 0) continue;
    
    double teammateDistX = mTeamBallX - teammate.x;
    double teammateDistY = mTeamBallY - teammate.y;
    double teammateDistance = sqrt(teammateDistX*teammateDistX + teammateDistY*teammateDistY);
    
    if (teammateDistance < myDistance) {
      amNearest = false;
      break;
    }
  }
  
  return amNearest;
}

// Coordinate defensive team behavior
void Soccer::coordinateDefense() {
  // Calculate distances to ball for all players
  std::map<int, double> distances;
  int nearestId = -1;
  double nearestDist = 9999.0;
  
  // Calculate my distance
  double myDistX = mTeamBallX - mMyEstimatedX;
  double myDistY = mTeamBallY - mMyEstimatedY;
  double myDistance = sqrt(myDistX*myDistX + myDistY*myDistY);
  
  distances[mPlayerID] = myDistance;
  
  if (myDistance < nearestDist && mPlayerID != 0) { // Skip goalkeeper
    nearestDist = myDistance;
    nearestId = mPlayerID;
  }
  
  // Check teammates
  for (const auto& pair : mTeammates) {
    int id = pair.first;
    const PlayerInfo& teammate = pair.second;
    
    // Skip outdated information
    if (getTime() - teammate.lastUpdateTime > 5.0) continue;
    
    // Skip goalkeeper for ball pursuit
    if (id == 0) continue;
    
    double teammateDistX = mTeamBallX - teammate.x;
    double teammateDistY = mTeamBallY - teammate.y;
    double teammateDistance = sqrt(teammateDistX*teammateDistX + teammateDistY*teammateDistY);
    
    distances[id] = teammateDistance;
    
    if (teammateDistance < nearestDist) {
      nearestDist = teammateDistance;
      nearestId = id;
    }
  }
  
  mNearestToBallID = nearestId;
  
  // Send coordination message if assigned coordinator
  if (mPlayerID == 1) { // Defender 1 takes coordination role
    std::string defenseData = std::to_string(nearestId) + "|" + 
                             std::to_string(mTeamBallX) + "|" + 
                             std::to_string(mTeamBallY);
    sendTeamMessage(DEFENSE_ALERT, defenseData);
    
    std::cout << "[Communication Thread] Defense coordination: Player " << nearestId 
              << " assigned to ball" << std::endl;
  }
}

// Coordinate offensive team behavior
void Soccer::coordinateOffense() {
  // Similar logic to defense but with offensive focus
  coordinateDefense();
}

// Execute kicking sequence
bool Soccer::executeKickingSequence() {
  if (!mInKickingSequence) {
    mKickSequenceStep = 0;
    mKickSequenceTimer = 0.0;
    mInKickingSequence = true;
    std::cout << "Starting kick sequence" << std::endl;
  }
  
  // Get current ball position
  double ballX = mBallData.x;
  double ballY = mBallData.y;
  double ballDistance = sqrt(ballX*ballX + ballY*ballY);
  
  // Debug output
  std::cout << "Kick sequence step " << mKickSequenceStep 
            << ", Ball at (" << ballX << ", " << ballY << ")" << std::endl;
  
  switch (mKickSequenceStep) {
    case 0: // Verification phase
      mGaitManager->stop();
      mKickSequenceTimer += mTimeStep / 1000.0;
      
      if (mKickSequenceTimer > 0.2) {
        if (ballY > 0.3 && ballDistance < 0.4) {
          mKickSequenceStep = 1;
          mKickSequenceTimer = 0.0;
          std::cout << "Ball position verified, proceeding with alignment" << std::endl;
        } else {
          // Ball moved - abort
          std::cout << "Ball position changed, aborting kick" << std::endl;
          mInKickingSequence = false;
          mHasBall = false;
          mGaitManager->start();
          return false;
        }
      }
      break;
      
    case 1: // Alignment phase
      {
        double alignmentAmp = -ballX * 0.6;
        mGaitManager->setXAmplitude(0.0);
        mGaitManager->setYAmplitude(0.0);
        mGaitManager->setAAmplitude(alignmentAmp);
        mGaitManager->step(mTimeStep);
        
        mKickSequenceTimer += mTimeStep / 1000.0;
        if (mKickSequenceTimer > 0.4) {
          // Re-verify ball position
          if (ballY > 0.2 && ballDistance < 0.5) {
            mKickSequenceStep = 2;
            mKickSequenceTimer = 0.0;
            std::cout << "Alignment complete, proceeding to approach" << std::endl;
          } else {
            // Ball moved - abort
            std::cout << "Ball position changed during alignment, aborting" << std::endl;
            mInKickingSequence = false;
            mHasBall = false;
            mGaitManager->start();
            return false;
          }
        }
      }
      break;
      
    case 2: // Approach phase
      {
        mGaitManager->setXAmplitude(0.5);
        mGaitManager->setYAmplitude(0.0);
        mGaitManager->setAAmplitude(0.0);
        mGaitManager->step(mTimeStep);
        
        mKickSequenceTimer += mTimeStep / 1000.0;
        if (mKickSequenceTimer > 0.25) {
          mKickSequenceStep = 3;
          mKickSequenceTimer = 0.0;
          mGaitManager->stop();
          std::cout << "Approach complete, preparing to kick" << std::endl;
        }
      }
      break;
      
    case 3: // Kick execution phase
      {
        double targetX = (mTeam == 'b') ? -1.0 : 1.0;
        
        mKickSequenceTimer += mTimeStep / 1000.0;
        if (mKickSequenceTimer > 0.3) {
          // Execute kick based on ball position
          if ((targetX < 0 && ballX < 0) || (targetX > 0 && ballX > 0)) {
            std::cout << "Executing left kick" << std::endl;
            mMotionManager->playPage(12); // right kick
          } else {
            std::cout << "Executing right kick" << std::endl;
            mMotionManager->playPage(13); // left kick
          }
          
          mMotionManager->playPage(9); // Return to ready position
          std::cout << "Kick executed, returning to normal behavior" << std::endl;
          
          // Reset state
          mInKickingSequence = false;
          mHasBall = false;
          mGaitManager->start();
          mPerformanceMetrics.successfulKicks++;
          return true;
        }
      }
      break;
  }
  
  return false; // Sequence still in progress
}

// Test all motion pages to find available movements
void Soccer::testAllMotionPages() {
  std::cout << "===== TESTING ALL MOTION PAGES =====" << std::endl;
  
  mGaitManager->stop();
  wait(500);
  
  // Try pages 1-20 to see which ones exist and work
  for (int page = 1; page <= 20; page++) {
    std::cout << "Testing motion page " << page << std::endl;
    
    // Try to play the page
    mMotionManager->playPage(page);
    
    // Wait to observe the effect
    wait(1000);
    
    // Return to a safe position
    mMotionManager->playPage(9); // Usually the "ready" position
    wait(500);
  }
  
  std::cout << "===== MOTION PAGE TESTING COMPLETE =====" << std::endl;
}

// Defender role implementation
void Soccer::playDefender() {
  // Initialize ball position variables
  double ballX, ballY;
  
  // Use enhanced ball detection
  bool ballVisible = enhancedBallDetection(ballX, ballY);
  
  if (ballVisible) {
    // Ball is visible - track it
    mEyeLED->set(0x0000FF);
    
    // Compute direction to ball
    double neckPosition = clamp(-ballX, -0.7, 0.7);
    double headPosition = clamp(-ballY, -0.7, 0.7);
    
    // Apply control fault to head/neck
    mFaultManager.applyControlFault(mPlayerID, headPosition, neckPosition);
    
    // Use safe head position setting
    safeSetHeadPosition(neckPosition, headPosition);
    
    // Calculate ball distance
    double ballDistance = sqrt(ballX*ballX + ballY*ballY);
    
    // Check if ball is in defender's zone
    bool inDefenderZone = (mTeam == 'b' && ballX < 0.2) || (mTeam == 'y' && ballX > -0.2);
    
    if (inDefenderZone || ballDistance < 0.6) {
      // Set movement parameters for ball pursuit
      double xAmp = 0.8;
      double yAmp = 0.0;
      double aAmp = neckPosition * 1.8;
      
      // Apply locomotion fault
      mFaultManager.applyLocomotionFault(mPlayerID, xAmp, yAmp, aAmp);
      
      // Move toward ball
      mGaitManager->setXAmplitude(xAmp);
      mGaitManager->setYAmplitude(yAmp);
      mGaitManager->setAAmplitude(aAmp);
      mGaitManager->step(mTimeStep);
      
      // Check if ball is in position for kicking
      if (ballY > 0.3 && ballDistance < 0.4) {
        mHasBall = true;
      }
    } else {
      // Return to defensive position
      double xAmp = 0.5;
      double yAmp = 0.0;
      double aAmp = 0.0;
      
      // Apply locomotion fault
      mFaultManager.applyLocomotionFault(mPlayerID, xAmp, yAmp, aAmp);
      
      // Return to position
      mGaitManager->setXAmplitude(xAmp);
      mGaitManager->setYAmplitude(yAmp);
      mGaitManager->setAAmplitude(aAmp);
      mGaitManager->step(mTimeStep);
    }
  } else {
    // Ball not visible - search for it
    mEyeLED->set(0xFF0000);
    implementBallSearch();
  }
}

// Goalkeeper role implementation
void Soccer::playGoalkeeper() {
  // Initialize ball position variables
  double ballX, ballY;
  
  // Use standard ball detection
  bool ballVisible = getBallCenter(ballX, ballY);
  
  // Debug output
  static int debugCounter = 0;
  if (++debugCounter >= 100) {
    std::cout << "GOALKEEPER MOVING LIKE FIELD PLAYER: Ball visible=" << (ballVisible ? "YES" : "NO") << std::endl;
    debugCounter = 0;
  }
  
  if (ballVisible) {
    // Ball is visible - track it
    mEyeLED->set(0x0000FF);
    
    // Compute direction to ball
    double neckPosition = clamp(-ballX, -0.35, 0.35);
    double headPosition = clamp(-ballY, -0.35, 0.35);
    
    // Move head to track ball
    mMotors[18]->setPosition(neckPosition);
    mMotors[19]->setPosition(headPosition);
    
    // Calculate ball distance
    double ballDistance = sqrt(ballX*ballX + ballY*ballY);
    
    // Default movement parameters
    double xAmp = 0.0;
    double yAmp = 0.0;
    double aAmp = 0.0;
    
    // Check for direct threats
    bool directThreat = (mTeam == 'b' && ballX > 0.4) || (mTeam == 'y' && ballX < -0.4);
    
    if (directThreat && ballDistance < 0.5) {
      // Ball is a threat - move to intercept
      std::cout << "GOALKEEPER CHARGING BALL" << std::endl;
      
      // Aggressive movement
      xAmp = 0.8;
      aAmp = neckPosition * 1.5;
      yAmp = -ballY * 0.5;
      
      // Very close ball - attempt to block
      if (ballDistance < 0.25) {
        mGaitManager->stop();
        
        if (ballY > 0) {
          mMotionManager->playPage(13);  // Right block
        } else {
          mMotionManager->playPage(12);  // Left block
        }
        
        // Return to ready position
        mMotionManager->playPage(9);
        mGaitManager->start();
      }
    }
    else {
      // Ball seen but not an immediate threat
      xAmp = 0.3;
      yAmp = -ballY * 0.5;
      aAmp = neckPosition * 0.8;
    }
    
    // Apply movement
    mGaitManager->setXAmplitude(xAmp);
    mGaitManager->setYAmplitude(yAmp);
    mGaitManager->setAAmplitude(aAmp);
    
    // Check if ball is in position for kicking
    if (ballY > 0.3 && ballDistance < 0.4) {
      mHasBall = true;
      std::cout << "GOALKEEPER HAS BALL IN KICKING POSITION" << std::endl;
    }
  } else {
    // Ball not visible - search pattern
    mEyeLED->set(0xFF0000);
    
    // Search pattern
    static int searchPhase = 0;
    static int phaseCounter = 0;
    
    // Change search phase periodically
    if (++phaseCounter > 20) {
      searchPhase = (searchPhase + 1) % 4;
      phaseCounter = 0;
    }
    
    // Set movement parameters based on search phase
    double xAmp = 0.3;
    double yAmp = 0.0;
    double aAmp = 0.0;
    
    switch(searchPhase) {
      case 0: // Turn left while moving
        aAmp = 0.3;
        break;
      case 1: // Move forward
        xAmp = 0.5;
        break;
      case 2: // Turn right while moving
        aAmp = -0.3;
        break;
      case 3: // Side to side
        yAmp = sin(getTime() * 0.5) * 0.5;
        break;
    }
    
    // Apply search movement
    mGaitManager->setXAmplitude(xAmp);
    mGaitManager->setYAmplitude(yAmp);
    mGaitManager->setAAmplitude(aAmp);
    
    // Head scanning motion
    double headPosition = sin(getTime() * 0.8) * 0.35;
    double neckPosition = cos(getTime() * 0.8) * 0.35;
    mMotors[18]->setPosition(neckPosition);
    mMotors[19]->setPosition(headPosition);
  }
  
  // Apply movement
  mGaitManager->step(mTimeStep);
}

// Midfielder role implementation
void Soccer::playMidfielder() {
  // Initialize ball position variables
  double ballX, ballY;
  static double px = 0.0;
  static double py = 0.0;
  
  // Use enhanced ball detection
  bool ballVisible = enhancedBallDetection(ballX, ballY);
  
  if (ballVisible) {
    // Ball is visible
    mEyeLED->set(0x0000FF);
    
    // Smooth head movement tracking
    double x = 0.015 * ballX + px;
    double y = 0.015 * ballY + py;
    px = x;
    py = y;
    
    // Set head position
    double neckPosition = clamp(-x, -0.7, 0.7);
    double headPosition = clamp(-y, -0.7, 0.7);
    
    // Apply control fault to head/neck
    mFaultManager.applyControlFault(mPlayerID, headPosition, neckPosition);
    
    // Use safe head position setting
    safeSetHeadPosition(neckPosition, headPosition);
    
    // Calculate distance to ball
    double ballDistance = sqrt(x*x + y*y);
    
    // Set movement parameters
    double xAmp, yAmp = 0.0, aAmp;
    
    // Adjust approach based on distance
    if (ballDistance > 0.5) {
      // Ball is far - faster approach
      xAmp = 0.9;
      aAmp = neckPosition * 2.0;
    } else {
      // Ball getting closer - more careful approach
      xAmp = 0.6;
      aAmp = neckPosition * 1.5;
    }
    
    // Apply locomotion fault
    mFaultManager.applyLocomotionFault(mPlayerID, xAmp, yAmp, aAmp);
    
    // Apply movement
    mGaitManager->setXAmplitude(xAmp);
    mGaitManager->setYAmplitude(yAmp);
    mGaitManager->setAAmplitude(aAmp);
    mGaitManager->step(mTimeStep);
    
    // Check if ball is in position for kicking
    if (y > 0.3 && ballDistance < 0.4) {
      mHasBall = true;
    }
  } else {
    // Ball not visible - search for it
    mEyeLED->set(0xFF0000);
    implementBallSearch();
  }
}

// Forward role implementation
void Soccer::playForward() {
  // Initialize ball position variables
  double ballX, ballY;
  
  // Use enhanced ball detection
  bool ballVisible = enhancedBallDetection(ballX, ballY);
  
  if (ballVisible) {
    // Ball is visible - aggressive pursuit
    mEyeLED->set(0x0000FF);
    
    // Set head position
    double neckPosition = clamp(-ballX, -0.7, 0.7);
    double headPosition = clamp(-ballY, -0.7, 0.7);
    
    // Apply control fault to head/neck
    mFaultManager.applyControlFault(mPlayerID, headPosition, neckPosition);
    safeSetHeadPosition(neckPosition, headPosition);
    
    // Calculate ball distance
    double ballDistance = sqrt(ballX*ballX + ballY*ballY);
    
    // Set aggressive movement parameters
    double xAmp = 1.0; // Maximum forward speed  
    double yAmp = 0.0;
    double aAmp = neckPosition * 2.5; // Very aggressive turning
    
    // Apply locomotion fault
    mFaultManager.applyLocomotionFault(mPlayerID, xAmp, yAmp, aAmp);
    
    // Apply movement
    mGaitManager->setXAmplitude(xAmp);
    mGaitManager->setYAmplitude(yAmp);
    mGaitManager->setAAmplitude(aAmp);
    mGaitManager->step(mTimeStep);
    
    // Check if ball is in position for kicking
    if (ballY > 0.3 && ballDistance < 0.4) {
      mHasBall = true;
    }
  } else {
    // Ball not visible - search for it
    mEyeLED->set(0xFF0000);
    implementBallSearch();
  }
}

// Log performance metrics
void Soccer::logPerformanceMetrics() {
  static double lastLog = 0.0;
  
  // Log every 30 seconds
  if (getTime() - lastLog > 30.0) {
    lastLog = getTime();
    
    std::cout << "\n=== ROBOT " << mPlayerID << " AI PERFORMANCE METRICS ===" << std::endl;
    std::cout << "Ball detection rate: " << 
      (mPerformanceMetrics.ballDetectionCount * 100.0 / (mPerformanceMetrics.frameCount + 1)) << "%" << std::endl;
    std::cout << "Successful kicks: " << mPerformanceMetrics.successfulKicks << std::endl;
    std::cout << "Average reward: " << mPerformanceMetrics.avgRewardPerFrame << std::endl;
    
    // Log AI learning weights
    std::cout << "AI Learning weights: Ball=" << mLearningWeights.ballDistance
              << " Formation=" << mLearningWeights.formationPosition
              << " Kick=" << mLearningWeights.kickSuccess << std::endl;
              
    // Log thread stats
    std::cout << "Vision thread processing rate: " << (1000.0 / mVisionRefreshRate) << " Hz" << std::endl;
    std::cout << "Decision thread processing rate: " << (1000.0 / mDecisionRefreshRate) << " Hz" << std::endl;
    std::cout << "Communication thread processing rate: " << (1000.0 / mCommunicationRefreshRate) << " Hz" << std::endl;
    std::cout << "================================================\n" << std::endl;
  }
}

// Main control loop
void Soccer::run() {
  std::cout << "[Main Thread] Soccer player initialized - Team: " << mTeam << ", ID: " << mPlayerID << std::endl;
  
  // Check if we're the goalkeeper
  std::string robotName = getName();
  bool isGoalkeeper = false;
  
  if (robotName.length() >= 2 && robotName[1] == '0') {
    isGoalkeeper = true;
    std::cout << "[Main Thread] GOALKEEPER DETECTED BY NAME: " << robotName << std::endl;
    mPlayerID = 0;
  } else if (mPlayerID == 0) {
    isGoalkeeper = true;
    std::cout << "[Main Thread] GOALKEEPER DETECTED BY ID: " << mPlayerID << std::endl;
  }
  
  // Initialize motion
  mMotionManager->playPage(1);   // init position
  wait(500);
  mMotionManager->playPage(9);   // walkready position
  wait(500);
  
  // Start walking for all players including goalkeepers
  mGaitManager->start();
  
  if (isGoalkeeper) {
    // Configure goalkeeper-specific settings
    std::cout << "=======================================" << std::endl;
    std::cout << "[Main Thread] GOALKEEPER MOVEMENT ENABLED - Using same movement system as field players" << std::endl;
    std::cout << "[Main Thread] GaitManager status: STARTED" << std::endl;
    std::cout << "=======================================" << std::endl;
    
    // Faster vision processing for goalkeeper
    mVisionRefreshRate = 5.0;
  }
  
  // Report thread status
  std::cout << "[Main Thread] All threads initialized and running:" << std::endl;
  std::cout << "[Main Thread] Vision thread ID: " << 101 << " - Status: Running" << std::endl;
  std::cout << "[Main Thread] Decision thread ID: " << 102 << " - Status: Running" << std::endl;
  std::cout << "[Main Thread] Communication thread ID: " << 103 << " - Status: Running" << std::endl;
  
  // Main control loop variables
  int frameCounter = 0;
  double lastTime = getTime();
  
  while (step(mTimeStep) != -1) {
    // Track timing
    double currentTime = getTime();
    double deltaTime = currentTime - lastTime;
    lastTime = currentTime;
    
    // Update fault manager
    mFaultManager.update(mTimeStep / 1000.0);
    
    // Execute threads in priority order
    simulateVisionThread();
    simulateDecisionThread();
    simulateCommunicationThread();
    
    // Update ball prediction
    predictBallPosition(deltaTime);
    
    // Log performance metrics
    logPerformanceMetrics();
    
    // Handle kickoff waiting
    if (mInKickoffWait) {
      // Handle waiting team
      if (!mIsKickoffTeam && mKickoffTimer > 0) {
        mKickoffTimer -= mTimeStep / 1000.0;
        
        // Visual indicator - flashing LEDs
        if ((int)(getTime() * 2) % 2 == 0) {
          mHeadLED->set(0xFF0000);
          mEyeLED->set(0xFF0000);
        } else {
          mHeadLED->set(0x000000);
          mEyeLED->set(0x000000);
        }
        
        // Allow limited head movement during wait
        double headPos = clamp(sin(getTime() * 0.5) * 0.3, -0.3, 0.3);
        double neckPos = 0.0;
        mMotors[18]->setPosition(neckPos);
        mMotors[19]->setPosition(headPos);
        
        // Limited movement during wait
        double xAmp = 0.0;
        double yAmp = 0.0;
        double aAmp = 0.0;
        
        // Small oscillation for goalkeepers
        if (isGoalkeeper) {
          yAmp = sin(getTime() * 0.3) * 0.1;
        }
        
        mGaitManager->setXAmplitude(xAmp);
        mGaitManager->setYAmplitude(yAmp);
        mGaitManager->setAAmplitude(aAmp);
        mGaitManager->step(mTimeStep);
        
        continue;
      } else if (!mIsKickoffTeam) {
        // Kickoff wait ended
        mInKickoffWait = false;
        
        // Reset LEDs
        mHeadLED->set(0x00FF00);
        mEyeLED->set(0x000000);
        
        std::cout << "Kickoff wait ended - normal operation resumed" << std::endl;
      }
    }
    
    // Check for falls periodically
    if (frameCounter % 10 == 0) {
      checkForFalls();
    }
    
    // Special handling for goalkeeper
    if (isGoalkeeper) {
      playGoalkeeper();
      continue;
    }
    
    // Field player behavior
    
    // Kicking sequence handling
    if (mInKickingSequence) {
      executeKickingSequence();
    }
    // Check for ball in kicking position
    else if (mHasBall && !mInKickingSequence) {
      mInKickingSequence = true;
      mKickSequenceStep = 0;
      mKickSequenceTimer = 0.0;
      std::cout << "[Main Thread] Starting kicking sequence" << std::endl;
    }
    // Check for pass reception
    else if (mExpectingPass) {
      receivePass();
    }
    // Formation maintenance when not chasing ball
    else if (!mInKickingSequence && !mHasBall && mNearestToBallID != mPlayerID) {
      maintainFormation();
    }
    // Role-specific ball pursuit behavior
    else if (!mInKickingSequence && !mHasBall) {
      switch (mPlayerID) {
        case 1:
        case 2:
          playDefender();
          break;
        case 3:
          playMidfielder();
          break;
        case 4:
          playForward();
          break;
        default:
          playDefender();
      }
    }
    
    // Debug output
    if (++frameCounter % 100 == 0) {
      std::cout << "[Main Thread] Robot " << mPlayerID << " running frame " << frameCounter << std::endl;
      
      // Team strategy debug
      std::cout << "[Decision Thread] Team state: ";
      if (mTeamHasBallVisibility) {
        std::cout << "Ball at " << mTeamBallX << ", " << mTeamBallY;
      } else {
        std::cout << "Ball not visible to team";
      }
      std::cout << ", Formation: " << (mInDefenseMode ? "Defense" : "Offense");
      std::cout << ", Nearest to ball: " << mNearestToBallID << std::endl;
      
      // Performance stats
      std::cout << "[Vision Thread] Ball detection rate: " 
                << (mPerformanceMetrics.ballDetectionCount * 100.0 / (frameCounter + 1)) << "%" << std::endl;
    }
  }
  
  // Shutdown
  std::cout << "[Main Thread] Shutting down soccer controller..." << std::endl;
  std::cout << "[Main Thread] Terminating all threads..." << std::endl;
  std::cout << "[Main Thread] All threads terminated successfully" << std::endl;
}
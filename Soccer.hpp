#ifndef SOCCER_HPP
#define SOCCER_HPP

#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <RobotisOp2VisionManager.hpp>
#include <webots/PositionSensor.hpp>
#include "FaultManager.hpp"
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <cstdlib>

using namespace webots;
using namespace managers;

// Message types for team communication
enum MessageType {
  BALL_POSITION,
  PLAYER_POSITION,
  PASS_REQUEST,
  PASS_EXECUTE,
  FORMATION_SHIFT,
  DEFENSE_ALERT,
  OFFENSE_OPPORTUNITY
};

// Structure to hold player information
struct PlayerInfo {
  int id;
  double x;
  double y;
  double orientation;
  bool hasBall;
  bool canSeeBall;
  double lastUpdateTime;
};

// Structure to define a team formation position
struct FormationPosition {
  double baseX;
  double baseY;
  double defendingOffset;
  double attackingOffset;
};

// Structure for AI performance metrics
struct PerformanceMetrics {
  int ballDetectionCount;
  int successfulKicks;
  int ballLostCount;
  double totalReward;
  double avgRewardPerFrame;
  int frameCount;
};

// Ball data structure for thread communication
struct BallData {
  double x;
  double y;
  bool visible;
  double lastUpdateTime;
  double velocityX;
  double velocityY;
  double lastSeenX;
  double lastSeenY;
  double predictionConfidence;
};

// Learning weights for AI behavior
struct LearningWeights {
  double ballDistance;
  double formationPosition;
  double kickSuccess;
};

class Soccer : public Robot {
public:
  Soccer();
  virtual ~Soccer();
  
  // Initialization
  void setTeam(char team) { mTeam = team; }
  void setPlayerID(int id) { mPlayerID = id; }
  
  // Main control function
  void run();
  
protected:
  // Time step and device management
  int mTimeStep;
  void myStep();
  void wait(int ms);
  
  // Ball detection
  bool getBallCenter(double &x, double &y);
  
  // Role-specific behaviors
  void playGoalkeeper();
  void playDefender();
  void playMidfielder();
  void playForward();
  void updateLearningWeights();
  
  // Fall detection and recovery
  void checkForFalls();
  
  // Ball search implementation
  void implementBallSearch();
  void safeSetHeadPosition(double desiredNeck, double desiredHead);
  
  // Team strategy methods
  void processTeamMessages();
  void sendTeamMessage(MessageType type, const std::string& data);
  void updateTeamState();
  bool findPassingOpportunity(int& targetId, double& targetX, double& targetY);
  void executePassing(int targetId, double targetX, double targetY);
  void receivePass();
  void maintainFormation();
  double getDistanceToFormationPosition();
  void determineTeamState();
  void coordinateDefense();
  void coordinateOffense();
  bool amINearestToBall();
  void testAllMotionPages();
  bool fallbackBallDetection(double &x, double &y);
  bool testMotionPages();
  bool enhancedBallDetection(double &x, double &y);
  void predictBallPosition(double deltaTime); 
   
  // Devices
  Camera *mCamera;
  LED *mEyeLED;
  LED *mHeadLED;
  LED *mBackLedRed;
  LED *mBackLedGreen;
  LED *mBackLedBlue;
  Receiver *mReceiver;
  Emitter *mEmitter;
  Motor *mMotors[20];
  Accelerometer *mAccelerometer;
  Gyro *mGyro;
  
  // Managers
  RobotisOp2MotionManager *mMotionManager;
  RobotisOp2GaitManager *mGaitManager;
  RobotisOp2VisionManager *mVisionManager;
  
  // Fault Manager
  FaultManager mFaultManager;
  
  // Robot identity
  char mTeam;    // 'b' for blue, 'y' for yellow
  int mPlayerID; // 0 for goalkeeper, 1-4 for field players
  
  // Team strategy members
  std::map<int, PlayerInfo> mTeammates;
  bool mHasBall;
  bool mPassInProgress;
  int mPassTarget;
  bool mExpectingPass;
  double mBallLastX;
  double mBallLastY;
  double mMyEstimatedX;
  double mMyEstimatedY;
  double mMyOrientation;
  FormationPosition mFormationPosition;
  double mLastMessageTime;
  bool mInDefenseMode;
  int mNearestToBallID;
  double mLastBallUpdateTime;
  double mTeamBallX;
  double mTeamBallY;
  bool mTeamHasBallVisibility;
  
  // Kickoff handling
  bool mInKickoffWait;
  bool mIsKickoffTeam;
  double mKickoffTimer;
  int mCurrentRound;
  
  // Multi-threading implementation
  BallData mBallData;
  
  // Thread timing
  double mLastVisionTime;
  double mLastDecisionTime;
  double mLastCommunicationTime;
  double mVisionRefreshRate;
  double mDecisionRefreshRate;
  double mCommunicationRefreshRate;
  
  // Thread functions
  void visionThread();
  void decisionThread();
  void communicationThread();
  
  // Kicking sequence control
  bool mInKickingSequence;
  int mKickSequenceStep;
  double mKickSequenceTimer;
  bool executeKickingSequence();
  
  // Performance and learning
  LearningWeights mLearningWeights;
  PerformanceMetrics mPerformanceMetrics;
  double calculateReward();
  void logPerformanceMetrics();
};

#endif
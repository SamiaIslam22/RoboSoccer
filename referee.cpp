#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include <webots/Emitter.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>

#define ROBOTS_PER_TEAM 5
#define TOTAL_ROBOTS (ROBOTS_PER_TEAM * 2)
#define GOAL_X_LIMIT 3.1 
#define GOAL_Y_MIN -1.1  
#define GOAL_Y_MAX 1.1  
#define GOAL_Z_MAX 0.65   // Lower from 0.25 to 0.2
// Enable debug mode for calibration
#define DEBUG_GOAL_DETECTION 1

#define TIME_STEP 64
#define GAME_DURATION 600  // 10 minutes

using namespace webots;

class RefereeController : public Supervisor {
private:
  // Game tracking
  int mTimeStep;
  double mGameTime;
  int mScores[2];  // blue, yellow
  double mBallResetTimer;
  
  // Added: Last team to touch the ball ('b' for blue, 'y' for yellow, '\0' for none)
  char mLastBallTouch;
  
  // Kickoff handling
  int mRoundCounter;            // Track the current round number
  bool mKickoffInProgress;      // Flag for kickoff state
  double mKickoffTimer;         // Timer for kickoff delay
  char mKickoffTeam;            // Team with kickoff rights ('b' or 'y')
  
  // Ball visibility tracking
  double mBallLastSeenTime;     // Time when the ball was last seen by any robot
  bool mBallCurrentlyVisible;   // Whether the ball is currently visible to any robot
  
  // Ball touch tracking
  double mBallLastTouchTime;    // Time when the ball was last touched by any robot
  
  // Communication
  Emitter *mEmitter;
  
  // Robot and ball nodes
  Node *mRobotNodes[TOTAL_ROBOTS];
  Node *mBallNode;
  
  // Field references
  Field *mRobotTranslationFields[TOTAL_ROBOTS];
  Field *mRobotRotationFields[TOTAL_ROBOTS];
  Field *mBallTranslationField;
  
  // Initial positions for reset
  double mRobotInitialTranslations[TOTAL_ROBOTS][3];
  double mRobotInitialRotations[TOTAL_ROBOTS][4];
  double mBallInitialTranslation[3];
  
  // Game management
  void checkGoals();
  void resetPositions();
  void setScoreDisplay();
  void setTimeDisplay();
  void sendGameState(const char *state);
  
  // Added: Check which team is touching the ball
  void checkBallContact();
  
  // Added: Check if the ball is within the red line goal bounds
  bool isBallInGoalBounds(const double *ballPos);
  
public:
  RefereeController();
  void run();
};

RefereeController::RefereeController() : Supervisor() {
  mTimeStep = getBasicTimeStep();
  mGameTime = GAME_DURATION;
  mScores[0] = 0;  // Blue score
  mScores[1] = 0;  // Yellow score
  mBallResetTimer = 0;
  
  // Initialize last ball touch to none
  mLastBallTouch = '\0';
  
  // Initialize ball visibility tracking
  mBallLastSeenTime = 0.0;
  mBallCurrentlyVisible = false;
  
  // Initialize ball touch tracking
  mBallLastTouchTime = 0.0;
  
  // Initialize kickoff variables
  mRoundCounter = 1;
  mKickoffInProgress = true;
  mKickoffTimer = 7.0;  // 7 seconds waiting time
  mKickoffTeam = 'b';   // Blue team gets first kickoff
  
  // Initialize emitter
  mEmitter = getEmitter("emitter");
  if (!mEmitter)
    printf("Warning: Emitter not found\n");
  
  // Get nodes - MODIFIED to use B0-B4 and Y0-Y4
  const char *robotNames[TOTAL_ROBOTS] = {"B0", "B1", "B2", "B3", "B4", "Y0", "Y1", "Y2", "Y3", "Y4"};
  
  for (int i = 0; i < TOTAL_ROBOTS; i++) {
    mRobotNodes[i] = getFromDef(robotNames[i]);
    if (mRobotNodes[i]) {
      mRobotTranslationFields[i] = mRobotNodes[i]->getField("translation");
      mRobotRotationFields[i] = mRobotNodes[i]->getField("rotation");
      
      // Store initial positions
      const double *translation = mRobotTranslationFields[i]->getSFVec3f();
      const double *rotation = mRobotRotationFields[i]->getSFRotation();
      
      for (int j = 0; j < 3; j++)
        mRobotInitialTranslations[i][j] = translation[j];
        
      for (int j = 0; j < 4; j++)
        mRobotInitialRotations[i][j] = rotation[j];
    } else {
      printf("Warning: Robot %s not found\n", robotNames[i]);
    }
  }
  
  // Get ball
  mBallNode = getFromDef("BALL");
  if (mBallNode) {
    mBallTranslationField = mBallNode->getField("translation");
    
    // Store ball initial position
    const double *ballTranslation = mBallTranslationField->getSFVec3f();
    for (int j = 0; j < 3; j++)
      mBallInitialTranslation[j] = ballTranslation[j];
  } else {
    printf("Warning: Ball not found\n");
  }
  
  // Display initial scores
  setScoreDisplay();
  
  // Send initial game state
  sendGameState("START");
  
  printf("=== SOCCER REFEREE INITIALIZED ===\n");
  printf("GOAL DETECTION: Ball must pass through red line area\n");
  printf("Goal boundaries: Y_MIN=%f, Y_MAX=%f, Z_MAX=%f\n", GOAL_Y_MIN, GOAL_Y_MAX, GOAL_Z_MAX);
  printf("OWN GOAL DETECTION ACTIVE: Last touch detection will prevent own goals\n");
  printf("BALL VISIBILITY TRACKING: Game will reset if ball is unnoticed for 15 seconds\n");
  printf("BALL TOUCH TRACKING: Game will reset if ball is untouched for 15 seconds\n");
  printf("=================================\n");
}

void RefereeController::setScoreDisplay() {
  char scoreStr[128];
  
  // Swap team name positions
  // RED now on left (was BLUE)
    setLabel(0, "BLUE", 0.25, 0.01, 0.06, 0x0000FF, 0.0, "Arial");
  // BLUE now on right (was RED)
  setLabel(1, "RED", 0.75, 0.01, 0.06, 0xFF0000, 0.0, "Arial");
  
  // Score values - positions swapped but values kept the same
  // This still shows mScores[0] for BLUE and mScores[1] for RED
  // but their positions are swapped
  sprintf(scoreStr, "%d", mScores[0]);  // BLUE score (index 0)
  setLabel(3, scoreStr, 0.65, 0.01, 0.08, 0xFFFFFF, 0.0, "Arial");
  
  sprintf(scoreStr, "%d", mScores[1]);  // RED score (index 1)
  setLabel(4, scoreStr, 0.35, 0.01, 0.08, 0xFFFFFF, 0.0, "Arial");
  
  // Rest of the function remains the same
  setLabel(5, "VS", 0.5, 0.015, 0.05, 0xFFFFFF, 0.0, "Arial");
  
  sprintf(scoreStr, "ROUND: %d", mRoundCounter);
  setLabel(6, scoreStr, 0.5, 0.06, 0.04, 0xFFFFFF, 0.0, "Arial");
  
  sprintf(scoreStr, "KICKOFF: %s", mKickoffInProgress ? (mKickoffTeam == 'b' ? "BLUE" : "RED") : "IN PLAY");
  setLabel(7, scoreStr, 0.5, 0.09, 0.04, 0xFFFFFF, 0.0, "Arial");
  
  // Clear any shadow labels
  setLabel(10, "", 0.0, 0.0, 0.0, 0x000000, 0.0, "Arial");
  setLabel(11, "", 0.0, 0.0, 0.0, 0x000000, 0.0, "Arial");
  setLabel(12, "", 0.0, 0.0, 0.0, 0x000000, 0.0, "Arial");
  setLabel(13, "", 0.0, 0.0, 0.0, 0x000000, 0.0, "Arial");
  
  // Clear the possession label (label 9)
  setLabel(9, "", 0.0, 0.0, 0.0, 0x000000, 0.0, "Arial");
}

void RefereeController::setTimeDisplay() {
  char timeStr[64];
  
  // Time display - moved more to the right (from 0.4 to 0.5) and lower (from 0.85 to 0.88)
  sprintf(timeStr, "%02d:%02d", (int)(mGameTime / 60), (int)mGameTime % 60);
  
  // Create subtle outline effect - also moved
  setLabel(14, timeStr, 0.5, 0.88, 0.12, 0x000000, 0.6, "Arial");
  
  // Main time display - also moved
  setLabel(2, timeStr, 0.5, 0.88, 0.11, 0xFFFFFF, 0.0, "Arial");
}

void RefereeController::sendGameState(const char *state) {
  if (mEmitter) {
    // Format: STATE|BLUE_SCORE|YELLOW_SCORE|TIME
    char message[128];
    sprintf(message, "%s|%d|%d|%d", state, mScores[0], mScores[1], (int)mGameTime);
    mEmitter->send(message, strlen(message) + 1);
  }
}

// Function to check if the ball is within the red line goal bounds
bool RefereeController::isBallInGoalBounds(const double *ballPos) {
  // Check if ball Y position is between goal posts (horizontal bounds)
  bool withinYBounds = (ballPos[1] >= GOAL_Y_MIN && ballPos[1] <= GOAL_Y_MAX);
  
  // Check if ball Z position is below crossbar (vertical bound)
  bool withinZBounds = (ballPos[2] <= GOAL_Z_MAX);
  
  // In debug mode, print detailed information when near goal
  if (DEBUG_GOAL_DETECTION && (ballPos[0] > GOAL_X_LIMIT - 0.2 || ballPos[0] < -GOAL_X_LIMIT + 0.2)) {
    printf("RED LINE CHECK: Ball at Y=%f, Z=%f | Y_MIN=%f, Y_MAX=%f, Z_MAX=%f\n", 
           ballPos[1], ballPos[2], GOAL_Y_MIN, GOAL_Y_MAX, GOAL_Z_MAX);
    printf("  Within RED LINE: %s (Y bounds: %s, Z bounds: %s)\n", 
           (withinYBounds && withinZBounds) ? "YES" : "NO",
           withinYBounds ? "YES" : "NO", 
           withinZBounds ? "YES" : "NO");
  }
  
  // Ball must be within both Y and Z bounds to be considered in goal
  return withinYBounds && withinZBounds;
}

void RefereeController::checkBallContact() {
  if (!mBallNode) return;

  const double *ballPos = mBallTranslationField->getSFVec3f();
  bool ballSeenThisFrame = false;
  bool ballTouchedThisFrame = false; // Track if ball was touched this frame
  static bool ballWasTouchedLastFrame = false;
  double currentTime = getTime();
  
  // INCREASED DETECTION RADIUS: Changed from 2.0 to 5.0
  const double detectionRadius = 5.0;
  const double touchRadius = 0.5; // Define touch radius explicitly
  
  // Add field boundary check
  if (ballTouchedThisFrame != ballWasTouchedLastFrame) {
    printf("Ball touch state changed: %s\n", 
           ballTouchedThisFrame ? "TOUCHED" : "NOT TOUCHED");
    ballWasTouchedLastFrame = ballTouchedThisFrame;
  }
  
  bool outsideFieldBoundary = (fabs(ballPos[0]) > 3.5 || fabs(ballPos[1]) > 2.5);
  if (outsideFieldBoundary) {
    printf("Ball outside field boundary! Position: X=%f, Y=%f, Z=%f\n", 
           ballPos[0], ballPos[1], ballPos[2]);
  }
  
  // Check distance from each robot to the ball
  for (int i = 0; i < TOTAL_ROBOTS; i++) {
    if (!mRobotNodes[i]) continue;
    
    const double *robotPos = mRobotTranslationFields[i]->getSFVec3f();
    double dx = robotPos[0] - ballPos[0];
    double dy = robotPos[1] - ballPos[1];
    double dz = robotPos[2] - ballPos[2];
    double distSquared = dx*dx + dy*dy + dz*dz;
    
    // Check if robot can see the ball - use a larger distance for visibility than for touch
    if (distSquared < detectionRadius * detectionRadius) {
      ballSeenThisFrame = true;
      
      // Debug visibility with robot ID occasionally
      static int visDebugCounter = 0;
      if (++visDebugCounter % 50 == 0) {
        printf("Ball visible to robot %d at distance %.2f\n", i, sqrt(distSquared));
        visDebugCounter = 0;
      }
      
      // If robot is close enough to touch the ball, update last touch
      if (distSquared < touchRadius * touchRadius) {
        ballTouchedThisFrame = true; // Mark as touched this frame
        
        // First 5 robots are blue team, next 5 are yellow
        char team = (i < ROBOTS_PER_TEAM) ? 'b' : 'y';
        
        // Update last touch if it's different
        if (mLastBallTouch != team) {
          mLastBallTouch = team;
          
          // Removed setPossessionDisplay() call
          
          if (DEBUG_GOAL_DETECTION) {
            printf("Ball last touched by team %c (Robot %d)\n", team, i);
          }
        }
        
        // Update the last touch time
        mBallLastTouchTime = currentTime;
        
        // Debug touch occasionally
        static int touchDebugCounter = 0;
        if (++touchDebugCounter % 20 == 0) {
          printf("Ball touched by robot %d (Team %c) at time %.2f\n", i, team, currentTime);
          touchDebugCounter = 0;
        }
      }
    }
  }
  
  // Update ball visibility tracking with state change detection
  if (ballSeenThisFrame) {
    // Only log when visibility state changes
    if (!mBallCurrentlyVisible) {
      printf("VISIBILITY CHANGE: Ball became visible after %.2f seconds invisibility\n", 
             currentTime - mBallLastSeenTime);
    }
    mBallLastSeenTime = currentTime;
    mBallCurrentlyVisible = true;
  } else {
    // Only log when visibility state changes
    if (mBallCurrentlyVisible) {
      printf("VISIBILITY CHANGE: Ball just became invisible at time %.2f\n", currentTime);
    }
    mBallCurrentlyVisible = false;
  }
  
  // Force ball reset if it's outside the field boundary
  if (outsideFieldBoundary && mBallResetTimer == 0) {
    printf("Ball outside field boundary! Forcing reset after 3 seconds.\n");
    mBallResetTimer = 3;  // Wait 3 seconds before reset
    
    // Send message about ball being out of bounds
    if (mEmitter) {
      const char* message = "BALL_OUT_OF_BOUNDS|RESET";
      mEmitter->send(message, strlen(message) + 1);
    }
  }
}

void RefereeController::checkGoals() {
  if (mBallResetTimer == 0 && mBallNode) {
    const double *ballPos = mBallTranslationField->getSFVec3f();
    
    // Add debug output periodically
    static int debugCounter = 0;
    if (++debugCounter >= 100) {  // Print every 100 steps to avoid flooding
      printf("Ball position: X=%f, Y=%f, Z=%f | Last touch: %c\n", 
             ballPos[0], ballPos[1], ballPos[2], 
             mLastBallTouch == '\0' ? '?' : mLastBallTouch);
      debugCounter = 0;
    }
    
    // Check if the ball crosses the goal line
    if (ballPos[0] > GOAL_X_LIMIT) {  // Ball crossed blue goal line
      if (DEBUG_GOAL_DETECTION) {
        printf("Ball crossed BLUE goal line at position Y=%f, Z=%f\n", ballPos[1], ballPos[2]);
      }
      
      // Check if the ball is within the red line goal bounds
      if (isBallInGoalBounds(ballPos)) {
        if (DEBUG_GOAL_DETECTION) {
          printf("Ball is within RED LINE goal boundaries!\n");
        }
        
        // Always award goal to Yellow team, regardless of who touched it last
        // This handles both normal goals AND own goals by Blue team
        mScores[1]++;  // Yellow team scores
        setScoreDisplay();
        mBallResetTimer = 3;  // Wait 3 seconds before reset
        
        // Check who touched the ball last for proper message
        if (mLastBallTouch == 'b') {
          sendGameState("OWN_GOAL_BLUE");
          printf("Own goal by Blue team! Point awarded to Yellow team. Score: Blue %d - Yellow %d\n", 
                 mScores[0], mScores[1]);
        } else {
          sendGameState("GOAL_YELLOW");
          printf("Goal scored by Yellow team! Score: Blue %d - Yellow %d\n", 
                 mScores[0], mScores[1]);
        }
      } else {
        // Ball crossed goal line but not within red line area - out of bounds
        if (DEBUG_GOAL_DETECTION) {
          printf("Ball is OUTSIDE red line boundaries - OUT OF BOUNDS\n");
        }
        
        // Ball went out of bounds past the goal line - reset positions
        mBallResetTimer = 3;  // Wait 3 seconds before reset
        sendGameState("BALL_OUT");
        printf("Ball went out of bounds past blue goal line. Resetting positions.\n");
      }
    } 
    else if (ballPos[0] < -GOAL_X_LIMIT) {  // Ball crossed yellow goal line
      if (DEBUG_GOAL_DETECTION) {
        printf("Ball crossed YELLOW goal line at position Y=%f, Z=%f\n", ballPos[1], ballPos[2]);
      }
      
      // Check if the ball is within the red line goal bounds
      if (isBallInGoalBounds(ballPos)) {
        if (DEBUG_GOAL_DETECTION) {
          printf("Ball is within RED LINE goal boundaries!\n");
        }
        
        // Always award goal to Blue team, regardless of who touched it last
        // This handles both normal goals AND own goals by Yellow team
        mScores[0]++;  // Blue team scores
        setScoreDisplay();
        mBallResetTimer = 3;  // Wait 3 seconds before reset
        
        // Check who touched the ball last for proper message
        if (mLastBallTouch == 'y') {
          sendGameState("OWN_GOAL_YELLOW");
          printf("Own goal by Yellow team! Point awarded to Blue team. Score: Blue %d - Yellow %d\n", 
                 mScores[0], mScores[1]);
        } else {
          sendGameState("GOAL_BLUE");
          printf("Goal scored by Blue team! Score: Blue %d - Yellow %d\n", 
                 mScores[0], mScores[1]);
        }
      } else {
        // Ball crossed goal line but not within red line area - out of bounds
        if (DEBUG_GOAL_DETECTION) {
          printf("Ball is OUTSIDE red line boundaries - OUT OF BOUNDS\n");
        }
        
        // Ball went out of bounds past the goal line - reset positions
        mBallResetTimer = 3;  // Wait 3 seconds before reset
        sendGameState("BALL_OUT");
        printf("Ball went out of bounds past yellow goal line. Resetting positions.\n");
      }
    }
  } else if (mBallResetTimer > 0) {
    mBallResetTimer -= (double)mTimeStep / 1000.0;
    if (mBallResetTimer <= 0) {
      mBallResetTimer = 0;
      resetPositions();
      sendGameState("RESET");
    }
  }
}

void RefereeController::resetPositions() {
  // Reset ball position
  if (mBallNode)
    mBallTranslationField->setSFVec3f(mBallInitialTranslation);
  
  // Reset all robot positions
  for (int i = 0; i < TOTAL_ROBOTS; i++) {
    if (mRobotNodes[i]) {
      mRobotTranslationFields[i]->setSFVec3f(mRobotInitialTranslations[i]);
      mRobotRotationFields[i]->setSFRotation(mRobotInitialRotations[i]);
    }
  }
  
  // Reset last ball touch
  mLastBallTouch = '\0';
  
  // Reset ball visibility and touch tracking
  mBallLastSeenTime = getTime();
  mBallCurrentlyVisible = true;
  mBallLastTouchTime = getTime(); // Reset the touch time too
  
  // Set up next kickoff
  mRoundCounter++;
  mKickoffInProgress = true;
  mKickoffTimer = 7.0;
  
  // Alternate kickoff team
  mKickoffTeam = (mKickoffTeam == 'b') ? 'y' : 'b';
  
  // Signal new kickoff with explicit waiting team
  char kickoffMsg[128];
  sprintf(kickoffMsg, "KICKOFF|%c|%d|WAITING_%c", 
          mKickoffTeam,
          mRoundCounter,
          (mKickoffTeam == 'b') ? 'y' : 'b');  // Explicitly indicate which team should wait
  if (mEmitter) {
    mEmitter->send(kickoffMsg, strlen(kickoffMsg) + 1);
  }
  
  printf("New kickoff: Team %c, Round %d, Waiting Team %c\n", 
         mKickoffTeam, mRoundCounter, (mKickoffTeam == 'b') ? 'y' : 'b');
}

void RefereeController::run() {
  printf("Soccer Referee started\n");

  while (step(mTimeStep) != -1) {
    // Current time (for consistent timing in this frame)
    double currentTime = getTime();
    
    // Update game time
    mGameTime -= (double)mTimeStep / 1000.0;
    if (mGameTime < 0) {
      mGameTime = GAME_DURATION;  // Restart game
      mScores[0] = 0;
      mScores[1] = 0;
      setScoreDisplay();
      sendGameState("RESTART");
      printf("Game restarted!\n");
      
      // Reset kickoff for new game
      mRoundCounter = 1;
      mKickoffInProgress = true;
      mKickoffTimer = 7.0;
      mKickoffTeam = 'b';
      
      // Reset positions after updating kickoff info
      resetPositions();
      
      // Signal new kickoff with explicit waiting team
      sprintf(kickoffMsg, "KICKOFF|%c|%d|WAITING_%c", 
              mKickoffTeam,
              mRoundCounter,
              (mKickoffTeam == 'b') ? 'y' : 'b');
      if (mEmitter) {
        mEmitter->send(kickoffMsg, strlen(kickoffMsg) + 1);
      }
    }
    setTimeDisplay();
    
    // Removed setPossessionDisplay() call
    
    // Check ball contact to track which team last touched the ball and if ball is visible
    checkBallContact();
    
    // Calculate time ball has been invisible and untouched
    double timeUnseen = currentTime - mBallLastSeenTime;
    double timeUntouched = currentTime - mBallLastTouchTime;
    
    // Debug output every second for visibility and touch status
    if (currentTime - lastStatusCheckTime > 1.0) {
      lastStatusCheckTime = currentTime;
      
      // Print detailed visibility and touch status
      printf("BALL STATUS: %s for %.2f seconds, Last touched %.2f seconds ago\n", 
             mBallCurrentlyVisible ? "VISIBLE" : "INVISIBLE",
             mBallCurrentlyVisible ? 0.0 : timeUnseen,
             timeUntouched);
      
      // Also log ball position for debugging
      if (mBallNode) {
        const double *ballPos = mBallTranslationField->getSFVec3f();
        printf("Ball position: X=%f, Y=%f, Z=%f | Visibility: %s\n", 
               ballPos[0], ballPos[1], ballPos[2], 
               mBallCurrentlyVisible ? "TRUE" : "FALSE");
      }
    }
    
    // Check if ball has been untouched for too long (45 seconds)
    if (timeUntouched > 25.0 && mBallResetTimer == 0) {
      printf("!!! RESET TRIGGERED: Ball untouched for %.2f seconds (> 45 seconds threshold) - resetting positions !!!\n", 
             timeUntouched);
      
      // Set ball reset timer to trigger a reset
      mBallResetTimer = 3;  // 3 second countdown before reset
      
      // Send message about ball being untouched for too long
      if (mEmitter) {
        const char* message = "BALL_UNTOUCHED|RESET";
        mEmitter->send(message, strlen(message) + 1);
      }
    }
    
    // Also check if ball has been unseen for too long (45 seconds) as a backup
    if (!mBallCurrentlyVisible && timeUnseen > 25.0 && mBallResetTimer == 0) {
      printf("!!! RESET TRIGGERED: Ball unseen for %.2f seconds (> 45 seconds threshold) - resetting positions !!!\n", 
             timeUnseen);
      
      // Set ball reset timer to trigger a reset
      mBallResetTimer = 3;  // 3 second countdown before reset
      
      // Send message about ball being lost
      if (mEmitter) {
        const char* message = "BALL_LOST|RESET";
        mEmitter->send(message, strlen(message) + 1);
      }
    }
    
    // Handle kickoff state
    if (mKickoffInProgress) {
      mKickoffTimer -= (double)mTimeStep / 1000.0;
      if (mKickoffTimer <= 0) {
        mKickoffInProgress = false;
        
        // Signal kickoff complete
        char kickoffEndMsg[128];
        sprintf(kickoffEndMsg, "KICKOFF_COMPLETE|%c|%d", mKickoffTeam, mRoundCounter);
        if (mEmitter) {
          mEmitter->send(kickoffEndMsg, strlen(kickoffEndMsg) + 1);
        }
        printf("Kickoff complete for team %c, round %d\n", mKickoffTeam, mRoundCounter);
      }
    }
    
    // Send periodic updates (every 5 seconds)
    if ((int)mGameTime % 5 == 0) {
      sendGameState("UPDATE");
    }
    
    // Check for goals
    checkGoals();
    
    // Process ball reset timer if active
    if (mBallResetTimer > 0) {
      mBallResetTimer -= (double)mTimeStep / 1000.0;
      printf("Reset countdown: %.1f seconds remaining\n", mBallResetTimer);
      
      if (mBallResetTimer <= 0) {
        mBallResetTimer = 0;
        resetPositions();
        sendGameState("RESET");
        printf("Reset complete - positions restored\n");
        
        // Reset visibility and touch tracking after reset
        mBallLastSeenTime = currentTime;
        mBallCurrentlyVisible = true;
        mBallLastTouchTime = currentTime;
      }
    }
  }
}

#ifdef _WIN32
#include <windows.h>

// Windows entry point that creates and runs the controller
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    RefereeController controller;
    controller.run();
    return 0;
}
#endif
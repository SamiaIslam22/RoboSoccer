#include "Soccer.hpp"
#include <iostream>
#include <cstring>  // For string comparison
using namespace std;

/**
 * Main entry point for the soccer robot controller
 * Processes command line arguments to set team and player role
 */
int main(int argc, char **argv) {
  cout << "Soccer controller starting with " << argc << " arguments" << endl;
  
  // Log received arguments for debugging
  for (int i = 0; i < argc; i++) {
    cout << "  Arg " << i << ": " << argv[i] << endl;
  }
  
  // Initialize soccer player
  Soccer player;
  
  // Process command line arguments if available
  if (argc >= 3) {
    // Parse team parameter
    char team = 'b'; // Default to blue
    if (strcmp(argv[1], "blue") == 0) {
      team = 'b';
    } else if (strcmp(argv[1], "yellow") == 0) {
      team = 'y';
    } else {
      team = argv[1][0]; // Use first character as fallback
    }
    
    // Parse player ID and detect goalkeeper role
    int id = 99; // Invalid default
    bool isGoalkeeper = false;
    
    if (strcmp(argv[2], "0") == 0) {
      id = 0;
      isGoalkeeper = true;
      cout << "*** GOALKEEPER DETECTED ***" << endl;
    } else {
      id = atoi(argv[2]);
    }
    
    cout << "Setting team to " << team << " and ID to " << id << endl;
    player.setTeam(team);
    player.setPlayerID(id);
    
    if (isGoalkeeper) {
      cout << "GOALKEEPER MODE ENABLED - Goalkeeper can move" << endl;
    }
  } else {
    cout << "Not enough arguments, using defaults (blue team, player 1)" << endl;
  }
  
  // Start the main control loop
  player.run();
  
  return 0;
}
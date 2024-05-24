#include "solver.h"
#include "API.h"

Action solver() {
    return leftWallFollower();
}

// This is an example of a simple left wall following algorithm.
Action leftWallFollower() {
    if(API_wallFront()) {
        if(API_wallLeft()){
            return RIGHT;
        }
        return LEFT;
    }
    return FORWARD;
}


// Put your implementation of floodfill here!
Action floodFill() {
    return IDLE;
}
/*#include "solver.h"
#include "API.h"
#include <stdlib.h>
#include <stdbool.h>
typedef struct {
    int x, y;
} Coordinate;

#define MAX_SIZE 256
Coordinate queue[MAX_SIZE];
int front = 0, rear = 0;

void enqueue(Coordinate coord) {
    queue[rear] = coord;
    rear = (rear + 1) % MAX_SIZE;
}

Coordinate dequeue() {
    Coordinate coord = queue[front];
    front = (front + 1) % MAX_SIZE;
    return coord;
}

void initializeMaze(int width, int height, int distances[][width], char states[][width]) {
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            distances[i][j] = 999;  // High initial value
            states[i][j] = 'B';    // 'B' for Blank state
        }
    }
    // Setting goal cell, assume it's at (0,0) for now
    distances[0][0] = 0;
    states[0][0] = 'P';    // 'P' for Processed
    enqueue((Coordinate){0, 0});
}

void floodFill(int width, int height, int distances[][width], char states[][width]) {
    while (front != rear) {
        Coordinate current = dequeue();
        int currentDist = distances[current.y][current.x];
        Coordinate directions[4] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};  // N, E, S, W

        for (int i = 0; i < 4; i++) {
            Coordinate next = {current.x + directions[i].x, current.y + directions[i].y};
            if (next.x >= 0 && next.x < width && next.y >= 0 && next.y < height &&
                states[next.y][next.x] == 'B') {
                distances[next.y][next.x] = currentDist + 1;
                states[next.y][next.x] = 'P';
                enqueue(next);
            }
        }
    }
}
#include "solver.h"
#include "API.h"
#include "stdlib.h"
Action solver() {
    int width = API_mazeWidth();
    int height = API_mazeHeight();
    int distances[height][width];
    char states[height][width];

    //initializeMaze(width, height, distances, states);
   // floodFill(width, height, distances, states);

    // Placeholder for movement logic, can use distances to decide actions
    while (1) {
        if (API_wallFront()) {
            API_turnLeft();
        } else {
            API_moveForward();
        }
    }
    return FORWARD;
}

/*#include "solver.h"
#include "API.h"
#include <stdlib.h>

typedef struct {
    int x, y;
} Coordinate;
typedef struct {
    Coordinate position;
    Heading heading;
} Robot;

#define MAX_SIZE 256
Coordinate queue[MAX_SIZE];
int front = 0, rear = 0;

void enqueue(Coordinate coord) {
    queue[rear] = coord;
    rear = (rear + 1) % MAX_SIZE;
}

Coordinate dequeue() {
    Coordinate coord = queue[front];
    front = (front + 1) % MAX_SIZE;
    return coord;
}
void initializeMaze(int distances[][API_mazeWidth()], char states[][API_mazeWidth()], Coordinate start) {
    for (int i = 0; i < API_mazeHeight(); i++) {
        for (int j = 0; j < API_mazeWidth(); j++) {
            distances[i][j] = 999;  // High initial value
            states[i][j] = 'B';    // 'B' for Blank state
        }
    }
    // Setting the goal cell
    distances[start.y][start.x] = 0;
    states[start.y][start.x] = 'P';    // 'P' for Processed
    enqueue(start);
}


void floodFill(int distances[][API_mazeWidth()], char states[][API_mazeWidth()]) {
    while (front != rear) {
        Coordinate current = dequeue();
        int currentDist = distances[current.y][current.x];
        Coordinate directions[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};  // NORTH, EAST, SOUTH, WEST

        for (int i = 0; i < 4; i++) {
            Coordinate next = {current.x + directions[i].x, current.y + directions[i].y};
            if (next.x >= 0 && next.x < API_mazeWidth() && next.y >= 0 && next.y < API_mazeHeight() &&
                states[next.y][next.x] == 'B') {
                distances[next.y][next.x] = currentDist + 1;
                states[next.y][next.x] = 'P';
                enqueue(next);
            }
        }
    }
}
Action determineNextAction(Robot *robot, int distances[][API_mazeWidth()]) {
    Coordinate current = robot->position;
    int minDistance = distances[current.y][current.x];
    Action bestAction = FORWARD; // Default action is to move forward
    Coordinate directions[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // NORTH, EAST, SOUTH, WEST

    for (int i = 0; i < 4; i++) {
        Coordinate next = {current.x + directions[i].x, current.y + directions[i].y};
        if (next.x >= 0 && next.x < API_mazeWidth() && next.y >= 0 && next.y < API_mazeHeight() &&
            distances[next.y][next.x] < minDistance && !API_wallFront()) {
            minDistance = distances[next.y][next.x];
            int directionDifference = (i - robot->heading + 4) % 4;
            if (directionDifference == 0) bestAction = FORWARD;
            else if (directionDifference == 1) bestAction = RIGHT;
            else if (directionDifference == 3) bestAction = LEFT;
        }
    }

    return bestAction;
}

Action solver() {
    static Robot robot = {{0, 0}, NORTH}; // Initialize the robot's position and heading
    int distances[API_mazeHeight()][API_mazeWidth()];
    char states[API_mazeHeight()][API_mazeWidth()]; // Tracking cell states

    initializeMaze(distances, states, robot.position);
    floodFill(distances, states);

    Action nextAction = determineNextAction(&robot, distances);
    // Execute the decided action and update the robot's state accordingly
    switch (nextAction) {
        case FORWARD:
            API_moveForward();
            // Update robot's position based on heading
            break;
        case RIGHT:
            API_turnRight();
            robot.heading = (robot.heading + 1) % 4; // Update heading
            break;
        case LEFT:
            API_turnLeft();
            robot.heading = (robot.heading + 3) % 4; // Update heading clockwise
            break;
        default:
            break;
    }

    return nextAction; // Return the action for possible use or logging
}

*/

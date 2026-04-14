// Labirent Çözücü — Online Flood Fill
// BY fox
// Algorithm: Real-time flood fill. No exploration phase needed.
// Senses walls, updates flood distances, moves to lowest neighbor. Repeat.

#include <Arduino.h>

// ── Sensor Pins ──────────────────────────────────────────────────────────────
const int HCSR_TRIG  = 6;
const int HCSR_FRONT = 2;   // Echo — front sensor
const int HCSR_LEFT  = 3;   // Echo — left sensor
const int HCSR_RIGHT = 4;   // Echo — right sensor

// ── Motor Pins ────────────────────────────────────────────────────────────────
const int R_IN1 = 15;
const int R_IN2 = 16;
const int R_PWM = 17;
const int L_IN1 = 18;
const int L_IN2 = 19;
const int L_PWM = 20;

// ── Start Button ──────────────────────────────────────────────────────────────
const int START_BTN = 7;   // Pulled LOW to start (active LOW with INPUT_PULLUP)

// ── Maze Config ───────────────────────────────────────────────────────────────
const int MAZE_SIZE = 16;

// Wall bitmask flags — stored per cell as OR combinations
const uint8_t NORTH = 1;
const uint8_t EAST  = 2;
const uint8_t SOUTH = 4;
const uint8_t WEST  = 8;

// Goal = center 4 cells of a 16x16 maze
const int GOAL_COUNT = 4;
const int GOAL_CELLS[GOAL_COUNT][2] = {{7,7},{7,8},{8,7},{8,8}};

// ── Timing ────────────────────────────────────────────────────────────────────
// Tune these for your robot's physical speed
const int MOTOR_SPEED      = 200;    // PWM 0–255
const int CELL_TRAVEL_MS   = 400;    // ms to cross one full cell forward
const int TURN_90_MS       = 290;    // ms for a 90° in-place pivot
const long SENSOR_TIMEOUT  = 25000;  // pulseIn timeout in µs (~4m max range)
const int WALL_THRESHOLD   = 18;     // cm — closer than this = wall detected

// ── State ─────────────────────────────────────────────────────────────────────
// Coordinate system: (0,0) = start corner, X increases East, Y increases North
uint8_t distances[MAZE_SIZE][MAZE_SIZE]; // flood values — 255 = unreachable
uint8_t walls[MAZE_SIZE][MAZE_SIZE];     // bitmask of known walls per cell

int currentX       = 0;
int currentY       = 0;
int currentHeading = 0;   // 0=North  1=East  2=South  3=West

bool goalReached = false;

// Static BFS queue — avoids any heap allocation on the microcontroller
static int qx[MAZE_SIZE * MAZE_SIZE];
static int qy[MAZE_SIZE * MAZE_SIZE];


// ─────────────────────────────────────────────────────────────────────────────
// SENSOR FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────

// Fire shared TRIG, read one echo pin, return distance in cm
int readDistance(int echoPin) {
    digitalWrite(HCSR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(HCSR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(HCSR_TRIG, LOW);
    long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);
    if (duration == 0) return 999;       // timeout → nothing in range
    return (int)(duration / 58);         // µs → cm
}

bool checkFrontSensor() { return readDistance(HCSR_FRONT) < WALL_THRESHOLD; }
bool checkLeftSensor()   { return readDistance(HCSR_LEFT)  < WALL_THRESHOLD; }
bool checkRightSensor()  { return readDistance(HCSR_RIGHT) < WALL_THRESHOLD; }


// ─────────────────────────────────────────────────────────────────────────────
// WALL MAP
// ─────────────────────────────────────────────────────────────────────────────

// Record a wall on cell (x,y) in direction dir, and mirror it on the neighbor
void setWall(int x, int y, uint8_t dir) {
    walls[x][y] |= dir;

    // Find the neighbor and the opposite flag
    int nx = x, ny = y;
    uint8_t opposite;
    if      (dir == NORTH) { ny++;  opposite = SOUTH; }
    else if (dir == EAST)  { nx++;  opposite = WEST;  }
    else if (dir == SOUTH) { ny--;  opposite = NORTH; }
    else                   { nx--;  opposite = EAST;  }

    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE)
        walls[nx][ny] |= opposite;
}

// Convert the robot-relative sensor hits (front/left/right) into
// absolute cardinal walls based on the robot's current heading
void updateWallsMap(int x, int y, bool front, bool left, bool right) {
    const uint8_t cardinals[4] = {NORTH, EAST, SOUTH, WEST};

    uint8_t frontDir = cardinals[currentHeading];
    uint8_t leftDir  = cardinals[(currentHeading + 3) % 4];  // CCW 90°
    uint8_t rightDir = cardinals[(currentHeading + 1) % 4];  // CW  90°

    if (front) setWall(x, y, frontDir);
    if (left)  setWall(x, y, leftDir);
    if (right) setWall(x, y, rightDir);
}


// ─────────────────────────────────────────────────────────────────────────────
// FLOOD FILL — BFS outward from all goal cells simultaneously
// Every call fully recomputes distances using current wall knowledge.
// Runs in ~1–2 ms on STM32 for a 16x16 grid.
// ─────────────────────────────────────────────────────────────────────────────

void updateDistances() {
    // Reset everything to 255 (unreachable)
    memset(distances, 255, sizeof(distances));

    int head = 0, tail = 0;

    // Seed all goal cells at distance 0
    for (int i = 0; i < GOAL_COUNT; i++) {
        int gx = GOAL_CELLS[i][0];
        int gy = GOAL_CELLS[i][1];
        distances[gx][gy] = 0;
        qx[tail] = gx;
        qy[tail] = gy;
        tail++;
    }

    // BFS — propagate distances through open passages
    while (head < tail) {
        int x = qx[head];
        int y = qy[head];
        head++;
        uint8_t d = distances[x][y];

        // North
        if (y+1 < MAZE_SIZE && !(walls[x][y] & NORTH) && distances[x][y+1] > d+1) {
            distances[x][y+1] = d+1;
            qx[tail] = x; qy[tail] = y+1; tail++;
        }
        // East
        if (x+1 < MAZE_SIZE && !(walls[x][y] & EAST) && distances[x+1][y] > d+1) {
            distances[x+1][y] = d+1;
            qx[tail] = x+1; qy[tail] = y; tail++;
        }
        // South
        if (y-1 >= 0 && !(walls[x][y] & SOUTH) && distances[x][y-1] > d+1) {
            distances[x][y-1] = d+1;
            qx[tail] = x; qy[tail] = y-1; tail++;
        }
        // West
        if (x-1 >= 0 && !(walls[x][y] & WEST) && distances[x-1][y] > d+1) {
            distances[x-1][y] = d+1;
            qx[tail] = x-1; qy[tail] = y; tail++;
        }
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// FIND BEST MOVE
// Returns the absolute heading (0–3) of the open neighbor with the lowest
// flood distance. Returns -1 only if the robot is completely walled in.
// ─────────────────────────────────────────────────────────────────────────────

int findLowestNeighbor(int x, int y) {
    const int      dx[4]      = { 0,  1,  0, -1};
    const int      dy[4]      = { 1,  0, -1,  0};
    const uint8_t  wallBit[4] = {NORTH, EAST, SOUTH, WEST};

    int     bestDir  = -1;
    uint8_t bestDist = 255;

    for (int dir = 0; dir < 4; dir++) {
        if (walls[x][y] & wallBit[dir]) continue;    // wall blocks this side
        int nx = x + dx[dir];
        int ny = y + dy[dir];
        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
        if (distances[nx][ny] < bestDist) {
            bestDist = distances[nx][ny];
            bestDir  = dir;
        }
    }
    return bestDir;
}

bool isAtGoal(int x, int y) {
    for (int i = 0; i < GOAL_COUNT; i++)
        if (x == GOAL_CELLS[i][0] && y == GOAL_CELLS[i][1]) return true;
    return false;
}


// ─────────────────────────────────────────────────────────────────────────────
// MOTOR CONTROL
// L298N: forward = IN1 HIGH + IN2 LOW, backward = IN1 LOW + IN2 HIGH
// ─────────────────────────────────────────────────────────────────────────────

void motorStop() {
    digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, LOW);  analogWrite(L_PWM, 0);
    digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, LOW);  analogWrite(R_PWM, 0);
}

void ileri() {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);  analogWrite(L_PWM, MOTOR_SPEED);
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);  analogWrite(R_PWM, MOTOR_SPEED);
}

void geri() {
    digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH); analogWrite(L_PWM, MOTOR_SPEED);
    digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH); analogWrite(R_PWM, MOTOR_SPEED);
}

// Pivot 90° right in place: left motor forward, right motor backward
void sagaDon() {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);  analogWrite(L_PWM, MOTOR_SPEED);
    digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH); analogWrite(R_PWM, MOTOR_SPEED);
    delay(TURN_90_MS);
    motorStop();
    delay(30); // brief settle before next command
}

// Pivot 90° left in place: left motor backward, right motor forward
void solaDon() {
    digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH); analogWrite(L_PWM, MOTOR_SPEED);
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);  analogWrite(R_PWM, MOTOR_SPEED);
    delay(TURN_90_MS);
    motorStop();
    delay(30);
}

// 180° U-turn: two right pivots
void geriDon() {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);  analogWrite(L_PWM, MOTOR_SPEED);
    digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH); analogWrite(R_PWM, MOTOR_SPEED);
    delay(TURN_90_MS * 2);
    motorStop();
    delay(30);
}


// ─────────────────────────────────────────────────────────────────────────────
// EXECUTE MOVE
// Turns to face nextHeading, then drives straight one full cell
// ─────────────────────────────────────────────────────────────────────────────

void executeMove(int nextHeading) {
    int diff = (nextHeading - currentHeading + 4) % 4;

    if      (diff == 1) sagaDon();   // 90° right
    else if (diff == 3) solaDon();   // 90° left
    else if (diff == 2) geriDon();   // 180°
    // diff == 0: already facing the right way, no turn needed

    ileri();
    delay(CELL_TRAVEL_MS);
    motorStop();
}

// Update tracked grid position after a move in the given heading
void updatePositionTracker(int heading) {
    const int dx[4] = { 0,  1,  0, -1};
    const int dy[4] = { 1,  0, -1,  0};
    currentX += dx[heading];
    currentY += dy[heading];
    currentHeading = heading;
}


// ─────────────────────────────────────────────────────────────────────────────
// SEARCH ALGORITHM — the main per-cell decision loop
// ─────────────────────────────────────────────────────────────────────────────

void search_algorithm() {
    if (goalReached) return;

    // Step 1 — Sense walls around the current cell
    bool wallFront = checkFrontSensor();
    bool wallLeft  = checkLeftSensor();
    bool wallRight = checkRightSensor();

    // Step 2 — Record newly discovered walls in the map (both sides mirrored)
    updateWallsMap(currentX, currentY, wallFront, wallLeft, wallRight);

    // Step 3 — Recompute all flood distances given updated wall knowledge
    updateDistances();

    // Step 4 — Check if we've arrived at the goal
    if (isAtGoal(currentX, currentY)) {
        motorStop();
        goalReached = true;
        Serial.println("GOAL!");
        return;
    }

    // Step 5 — Pick the open neighbor with the lowest flood value
    int nextHeading = findLowestNeighbor(currentX, currentY);

    if (nextHeading == -1) {
        // All neighbors are walled off — should never happen in a valid maze
        motorStop();
        Serial.println("ERR: no valid move from current cell");
        return;
    }

    // Step 6 — Turn to face that direction, drive one cell forward
    executeMove(nextHeading);

    // Step 7 — Update our tracked position
    updatePositionTracker(nextHeading);

    Serial.print("-> (");
    Serial.print(currentX); Serial.print(",");
    Serial.print(currentY); Serial.print(")  dist=");
    Serial.println(distances[currentX][currentY]);
}


// ─────────────────────────────────────────────────────────────────────────────
// SETUP & LOOP
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println("Hello guys this is solvex");

    // Sensor pins
    pinMode(HCSR_TRIG,  OUTPUT);
    pinMode(HCSR_FRONT, INPUT);
    pinMode(HCSR_LEFT,  INPUT);
    pinMode(HCSR_RIGHT, INPUT);

    // Motor pins
    pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_PWM, OUTPUT);
    pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_PWM, OUTPUT);

    // Start button — internal pull-up, press pulls to GND
    pinMode(START_BTN, INPUT_PULLUP);

    // Initialise maze state — no walls known yet, full flood fill
    memset(walls, 0, sizeof(walls));
    updateDistances();
    motorStop();

    Serial.println("Ready. Press start button...");
    while (digitalRead(START_BTN) == HIGH) delay(10);
    delay(500); // debounce + brief pause before moving
    Serial.println("Go!");
}

void loop() {
    search_algorithm();
}
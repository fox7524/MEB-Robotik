#include <Arduino.h>  // Arduino core for STM32F103C8T6 (Blue Pill) provides GPIO + timing + Serial APIs
#include <Wire.h>     // I2C support for MPU6050 on STM32F103C8T6 (3.3V logic)

/* =============================================================================
   PIN DEFINITIONS
   =============================================================================
   Hardware constraints:
   - STM32F103C8T6 GPIO is 3.3V. Any 5V signal must be shifted down.
   - "Exactly three level shifters" means we only shift the three HC-SR04 ECHO
     lines (one per sensor). All other signals must be native 3.3V (MPU6050 I2C
     pullups to 3.3V, KY-040 powered/pulled up to 3.3V).
   - Pin numbers below follow your existing sketch style; if your STM32 core uses
     PAx/PBx names, replace these constants accordingly.
*/

static const uint32_t SERIAL_BAUD = 115200;  // Serial monitor speed for diagnostics on Blue Pill

static const int START_BTN = 7;              // Start button (active LOW with INPUT_PULLUP)

static const int HCSR_TRIG = 6;              // Shared TRIG output (3.3V logic is OK for HC-SR04 TRIG)
static const int HCSR_FRONT_ECHO = 2;        // ECHO front (requires shifter CH1: 5V -> 3.3V)
static const int HCSR_LEFT_ECHO  = 3;        // ECHO left  (requires shifter CH2: 5V -> 3.3V)
static const int HCSR_RIGHT_ECHO = 4;        // ECHO right (requires shifter CH3: 5V -> 3.3V)

static const int R_IN1 = 15;                 // L298N right motor IN1 (3.3V logic OK)
static const int R_IN2 = 16;                 // L298N right motor IN2 (3.3V logic OK)
static const int R_PWM = 17;                 // L298N right motor ENA (PWM)
static const int L_IN1 = 18;                 // L298N left motor IN1 (3.3V logic OK)
static const int L_IN2 = 19;                 // L298N left motor IN2 (3.3V logic OK)
static const int L_PWM = 20;                 // L298N left motor ENB (PWM)

static const uint8_t MPU6050_ADDR = 0x68;    // MPU6050 I2C address (AD0 low)

static const int PIN_KY040_CLK = -1;         // Set to a real pin only if KY-040 is powered/pulled up to 3.3V
static const int PIN_KY040_DT  = -1;         // Set to a real pin only if KY-040 is powered/pulled up to 3.3V
static const int PIN_KY040_SW  = -1;         // Set to a real pin only if KY-040 is powered/pulled up to 3.3V

/* =============================================================================
   GLOBAL VARIABLES
   ============================================================================= */

static const unsigned long HCSR_ECHO_TIMEOUT_US = 25000;  // Timeout (us) prevents lockups; ~4m max range
static const int WALL_THRESHOLD_CM_DEFAULT = 18;          // Wall detection threshold; tune per maze + sensor
static int g_wallThresholdCm = WALL_THRESHOLD_CM_DEFAULT; // Runtime threshold (KY-040 can adjust in dev builds)

static const int TURN_90_MS = 350;                        // Turn duration (ms) for sag360/sol360 motion
static const int CELL_TRAVEL_MS = 700;                    // Forward duration (ms) to approximate one 20cm cell

static const int MAZE_W = 8;                              // Competition grid width
static const int MAZE_H = 16;                             // Competition grid height

static const uint8_t NORTH = 1;                           // Wall bit: north side blocked
static const uint8_t EAST  = 2;                           // Wall bit: east side blocked
static const uint8_t SOUTH = 4;                           // Wall bit: south side blocked
static const uint8_t WEST  = 8;                           // Wall bit: west side blocked

static uint8_t g_dist[MAZE_W][MAZE_H];                    // Flood distances; 255 means unknown/unreachable
static uint8_t g_walls[MAZE_W][MAZE_H];                   // Wall map bitmask per cell

static int g_x = 0;                                       // Current cell X (0..MAZE_W-1)
static int g_y = 0;                                       // Current cell Y (0..MAZE_H-1)
static int g_heading = 0;                                 // Heading: 0=N,1=E,2=S,3=W

static const int GOAL_COUNT = 4;                          // Temporary goal: center 4 cells (development)
static const int g_goalX[GOAL_COUNT] = {3, 3, 4, 4};      // Goal X coordinates
static const int g_goalY[GOAL_COUNT] = {7, 8, 7, 8};      // Goal Y coordinates

static int g_qx[MAZE_W * MAZE_H];                         // BFS queue X buffer (no heap allocation)
static int g_qy[MAZE_W * MAZE_H];                         // BFS queue Y buffer (no heap allocation)

static bool g_started = false;                            // Start latch; prevents accidental motion

/* =============================================================================
   SETUP
   ============================================================================= */

void setup() {
  Serial.begin(SERIAL_BAUD);                               // Enable serial diagnostics for shifters/sensors

  pinMode(START_BTN, INPUT_PULLUP);                        // Active-low start button; keeps robot stopped by default

  pinMode(HCSR_TRIG, OUTPUT);                              // TRIG is an output pulse from STM32 (3.3V is fine)
  digitalWrite(HCSR_TRIG, LOW);                            // Keep TRIG low when idle to avoid false pings

  pinMode(HCSR_FRONT_ECHO, INPUT);                         // ECHO must be 3.3V due to STM32; shifted externally
  pinMode(HCSR_LEFT_ECHO, INPUT);                          // ECHO must be 3.3V due to STM32; shifted externally
  pinMode(HCSR_RIGHT_ECHO, INPUT);                         // ECHO must be 3.3V due to STM32; shifted externally

  pinMode(R_IN1, OUTPUT);                                  // Motor direction pin (right)
  pinMode(R_IN2, OUTPUT);                                  // Motor direction pin (right)
  pinMode(R_PWM, OUTPUT);                                  // Motor PWM pin (right enable)
  pinMode(L_IN1, OUTPUT);                                  // Motor direction pin (left)
  pinMode(L_IN2, OUTPUT);                                  // Motor direction pin (left)
  pinMode(L_PWM, OUTPUT);                                  // Motor PWM pin (left enable)

  Wire.begin();                                            // Start I2C peripheral on STM32F103 (3.3V)
  Wire.setClock(400000);                                   // 400kHz reduces blocking time for IMU reads

  (void)mpu6050Init();                                     // Initialize MPU6050; safe even if disconnected

  if (PIN_KY040_CLK >= 0 && PIN_KY040_DT >= 0) {            // Only enable KY-040 if pins are configured
    pinMode(PIN_KY040_CLK, INPUT_PULLUP);                  // Pullups must be to 3.3V to avoid extra shifters
    pinMode(PIN_KY040_DT, INPUT_PULLUP);                   // Pullups must be to 3.3V to avoid extra shifters
  }
  if (PIN_KY040_SW >= 0) {                                 // Optional button pin (must also be 3.3V)
    pinMode(PIN_KY040_SW, INPUT_PULLUP);                   // Keep stable state without external resistors
  }

  motorStop();                                             // Fail-safe: ensure motors are off at boot

  memset(g_walls, 0, sizeof(g_walls));                     // Unknown walls at start; exploration fills this
  updateDistances();                                       // Seed flood distances from goals for first decision

  printShifterDiagnostics();                               // Print initial "SHIFTER OK/FAULT" for 3 channels

  Serial.println("READY");                                 // User feedback in serial monitor
}

/* =============================================================================
   LOOP
   ============================================================================= */

void loop() {
#ifdef UNIT_TEST
  unitTestMovement();                                      // Hardware-free movement tests (does not need sensors)
  while (true) delay(1000);                                // Freeze after test to avoid repeated runs
#endif

  if (!g_started) {                                        // Wait here until user presses start button
    if (digitalRead(START_BTN) == LOW) {                   // Active-low start detection
      delay(50);                                           // Debounce to avoid false starts
      if (digitalRead(START_BTN) == LOW) {                 // Confirm stable press before moving
        g_started = true;                                  // Latch start state so it doesn't re-arm
        Serial.println("START");                            // Diagnostic marker for run start
      }
    }

    ky040Service();                                        // Optional threshold tuning in development mode
    motorStop();                                           // Safety: keep motors off while waiting
    delay(5);                                              // Small delay to reduce CPU usage
    return;                                                // Stay in idle until started
  }

  printShifterDiagnostics();                               // Continuously verify the three shifted ECHO lines
  searchAlgorithmStep();                                   // One cell: sense -> map -> plan -> move -> update pose
}

/* =============================================================================
   USER-DEFINED FUNCTIONS
   ============================================================================= */

/* -----------------------------------------------------------------------------
   SAFETY & MOTOR CONTROL
   -----------------------------------------------------------------------------
   Algorithm (safety stop):
   - Input: none
   - Output: motors disabled and PWM set to 0
   - Timing: immediate; prevents runaway if sensors/logic fail
*/
static void motorStop() {
  digitalWrite(L_IN1, LOW);                                // Ensure left direction pins are low for stop
  digitalWrite(L_IN2, LOW);                                // Ensure left direction pins are low for stop
  digitalWrite(R_IN1, LOW);                                // Ensure right direction pins are low for stop
  digitalWrite(R_IN2, LOW);                                // Ensure right direction pins are low for stop
  analogWrite(L_PWM, 0);                                   // Disable left PWM channel
  analogWrite(R_PWM, 0);                                   // Disable right PWM channel
}

/* -----------------------------------------------------------------------------
   MOVEMENT FUNCTIONS (UNTOUCHED ileri() and geri())
   -----------------------------------------------------------------------------
   NOTE:
   - You requested that every line inside ileri() and geri() remains unchanged.
   - Therefore, there are intentionally no inline comments inside these two
     functions, and their content is byte-for-byte identical to the prior file.
*/

void ileri(){
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);


}

void geri(){
digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, LOW);


digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, HIGH);


analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);


}

void sag360(){
digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);

digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);

analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);

    
}
void sol360(){

digitalWrite(L_IN1, LOW);
digitalWrite(R_IN1, HIGH);

digitalWrite(L_IN1, HIGH);
digitalWrite(R_IN1, LOW);

analogWrite(L_PWM, 255);
analogWrite(R_PWM, 255);
    
}

void sag(){
digitalWrite(R_IN1, HIGH);
digitalWrite(R_IN1, HIGH);

analogWrite(R_PWM, 255);
    
}
void sol(){
digitalWrite(L_IN1, HIGH);
digitalWrite(L_IN1, HIGH);

analogWrite(L_PWM, 255);
    
}

/* -----------------------------------------------------------------------------
   SENSOR READING (HC-SR04 via exactly three shifted ECHO channels)
   -----------------------------------------------------------------------------
   Algorithm:
   - Input: one ECHO pin (already shifted to 3.3V), shared TRIG output pin
   - Output: distance in centimeters, or -1 on timeout/fault
   - Voltage constraints:
     - Only the three ECHO lines are shifted (exactly 3 shifters).
     - TRIG is 3.3V output; HC-SR04 typically accepts this as HIGH.
   - Timing constraints:
     - Each ping blocks up to HCSR_ECHO_TIMEOUT_US; keep this small to avoid
       delaying motion control too long.
*/

static unsigned long hcsr04ReadEchoUs(int echoPin) {
  digitalWrite(HCSR_TRIG, LOW);                            // Ensure clean low pulse before trigger
  delayMicroseconds(2);                                    // 2us low settles the HC-SR04 trigger circuit
  digitalWrite(HCSR_TRIG, HIGH);                           // Start 10us trigger pulse (spec requirement)
  delayMicroseconds(10);                                   // Keep high for 10us to initiate ultrasonic burst
  digitalWrite(HCSR_TRIG, LOW);                            // End trigger pulse
  return pulseIn(echoPin, HIGH, HCSR_ECHO_TIMEOUT_US);      // Measure echo high time (us) with timeout
}

static int hcsr04ReadCm(int echoPin) {
  unsigned long us = hcsr04ReadEchoUs(echoPin);             // Read echo pulse duration in microseconds
  if (us == 0) return -1;                                   // Timeout means no echo or wiring/level fault
  return (int)(us / 58);                                    // Convert us to cm (speed-of-sound approximation)
}

static bool isWallFront() {
  int cm = hcsr04ReadCm(HCSR_FRONT_ECHO);                   // Read shifted ECHO channel 1 (front)
  if (cm < 0) return true;                                  // Fail-safe: treat sensor fault as wall
  return cm < g_wallThresholdCm;                            // Wall if closer than threshold
}

static bool isWallLeft() {
  int cm = hcsr04ReadCm(HCSR_LEFT_ECHO);                    // Read shifted ECHO channel 2 (left)
  if (cm < 0) return true;                                  // Fail-safe: treat sensor fault as wall
  return cm < g_wallThresholdCm;                            // Wall if closer than threshold
}

static bool isWallRight() {
  int cm = hcsr04ReadCm(HCSR_RIGHT_ECHO);                   // Read shifted ECHO channel 3 (right)
  if (cm < 0) return true;                                  // Fail-safe: treat sensor fault as wall
  return cm < g_wallThresholdCm;                            // Wall if closer than threshold
}

/* -----------------------------------------------------------------------------
   SHIFTER DIAGNOSTICS (SHIFTER OK / SHIFTER FAULT)
   -----------------------------------------------------------------------------
   Algorithm:
   - Input: three ECHO pins that must be shifted to 3.3V.
   - Output: serial diagnostics lines:
       "SHIFTER OK  CHx" or "SHIFTER FAULT CHx"
   - Why this matters:
     - If an ECHO line is not shifted, it can read stuck HIGH or behave erratically,
       and can damage the STM32F103C8T6 input.
   - Implementation note:
     - Software cannot measure voltage; it checks plausible digital behavior:
       idle low + a measurable pulse within the timeout window.
*/

static void printOneShifterStatus(int ch, int echoPin) {
  int idle = digitalRead(echoPin);                           // Idle state should normally be LOW
  int cm = hcsr04ReadCm(echoPin);                            // Quick functional test ping on that channel
  bool ok = (idle == LOW) && (cm >= 0) && (cm <= 400);       // 400cm is typical HC-SR04 spec max range

  Serial.print("SHIFTER ");                                  // Prefix required by your troubleshooting spec
  Serial.print(ok ? "OK" : "FAULT");                         // OK if signal looks plausible; else FAULT
  Serial.print("  CH");                                      // Channel label for the three shifters
  Serial.print(ch);                                          // Channel number (1..3)
  Serial.print("  cm=");                                     // Show measured distance (or -1) for context
  Serial.println(cm);                                        // End line for serial monitor readability
}

static void printShifterDiagnostics() {
  static unsigned long lastMs = 0;                            // Rate limiting prevents serial spam and delays
  unsigned long now = millis();                               // Use millisecond timer from Arduino core
  if ((now - lastMs) < 300) return;                           // Print at ~3Hz to keep control loop responsive
  lastMs = now;                                               // Update timestamp for next cycle

  printOneShifterStatus(1, HCSR_FRONT_ECHO);                  // Shifter channel 1 = front echo
  printOneShifterStatus(2, HCSR_LEFT_ECHO);                   // Shifter channel 2 = left echo
  printOneShifterStatus(3, HCSR_RIGHT_ECHO);                  // Shifter channel 3 = right echo
}

/* -----------------------------------------------------------------------------
   MAZE MAPPING (wall bitmask storage with mirroring)
   -----------------------------------------------------------------------------
   Algorithm:
   - Input: robot cell (g_x, g_y), boolean walls in front/left/right, heading
   - Output: updates g_walls[x][y] and also mirrors the wall to the neighbor cell
   - Timing: constant time per call; safe for real-time loop
*/

static void setWall(int x, int y, uint8_t dir) {
  if (x < 0 || x >= MAZE_W || y < 0 || y >= MAZE_H) return;   // Bounds check prevents memory corruption
  g_walls[x][y] |= dir;                                       // Mark wall in current cell

  int nx = x;                                                 // Neighbor X to mirror the wall
  int ny = y;                                                 // Neighbor Y to mirror the wall
  uint8_t opp = 0;                                            // Opposite direction bit for neighbor

  if (dir == NORTH) { ny++; opp = SOUTH; }                    // North wall implies neighbor south wall
  else if (dir == EAST) { nx++; opp = WEST; }                 // East wall implies neighbor west wall
  else if (dir == SOUTH) { ny--; opp = NORTH; }               // South wall implies neighbor north wall
  else { nx--; opp = EAST; }                                  // West wall implies neighbor east wall

  if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) return;// Ignore if neighbor is outside the maze
  g_walls[nx][ny] |= opp;                                     // Mirror the wall for consistency
}

static void updateWallsFromSensors(bool front, bool left, bool right) {
  static const uint8_t card[4] = {NORTH, EAST, SOUTH, WEST};   // Map heading index to wall direction bit
  uint8_t frontDir = card[g_heading & 3];                     // Convert robot-front to absolute direction
  uint8_t leftDir  = card[(g_heading + 3) & 3];               // Left is heading-1 (mod 4)
  uint8_t rightDir = card[(g_heading + 1) & 3];               // Right is heading+1 (mod 4)

  if (front) setWall(g_x, g_y, frontDir);                     // Store front wall if detected
  if (left)  setWall(g_x, g_y, leftDir);                      // Store left wall if detected
  if (right) setWall(g_x, g_y, rightDir);                     // Store right wall if detected
}

/* -----------------------------------------------------------------------------
   FLOOD FILL (BFS) — recompute distances to goal based on discovered walls
   -----------------------------------------------------------------------------
   Algorithm:
   - Input: g_walls map, goal cells list
   - Output: g_dist grid filled with shortest-path distances (0 at goal)
   - Timing: O(MAZE_W*MAZE_H); small for 8x16 on STM32F103
*/

static void updateDistances() {
  memset(g_dist, 255, sizeof(g_dist));                        // Reset distances to "unknown"

  int head = 0;                                               // BFS queue head index
  int tail = 0;                                               // BFS queue tail index

  for (int i = 0; i < GOAL_COUNT; i++) {                      // Seed all goal cells at distance 0
    int gx = g_goalX[i];                                      // Goal x
    int gy = g_goalY[i];                                      // Goal y
    if (gx < 0 || gx >= MAZE_W || gy < 0 || gy >= MAZE_H) continue; // Ignore invalid goals
    g_dist[gx][gy] = 0;                                       // Distance to goal from goal is 0
    g_qx[tail] = gx;                                          // Push into BFS queue
    g_qy[tail] = gy;                                          // Push into BFS queue
    tail++;                                                   // Advance tail
  }

  while (head < tail) {                                       // BFS over reachable cells
    int x = g_qx[head];                                       // Pop x
    int y = g_qy[head];                                       // Pop y
    head++;                                                   // Advance head
    uint8_t d = g_dist[x][y];                                 // Current distance

    if (y + 1 < MAZE_H && !(g_walls[x][y] & NORTH) && g_dist[x][y + 1] > (uint8_t)(d + 1)) {
      g_dist[x][y + 1] = d + 1;                               // Relax north neighbor
      g_qx[tail] = x;                                         // Enqueue
      g_qy[tail] = y + 1;                                     // Enqueue
      tail++;                                                 // Advance tail
    }
    if (x + 1 < MAZE_W && !(g_walls[x][y] & EAST) && g_dist[x + 1][y] > (uint8_t)(d + 1)) {
      g_dist[x + 1][y] = d + 1;                               // Relax east neighbor
      g_qx[tail] = x + 1;                                     // Enqueue
      g_qy[tail] = y;                                         // Enqueue
      tail++;                                                 // Advance tail
    }
    if (y - 1 >= 0 && !(g_walls[x][y] & SOUTH) && g_dist[x][y - 1] > (uint8_t)(d + 1)) {
      g_dist[x][y - 1] = d + 1;                               // Relax south neighbor
      g_qx[tail] = x;                                         // Enqueue
      g_qy[tail] = y - 1;                                     // Enqueue
      tail++;                                                 // Advance tail
    }
    if (x - 1 >= 0 && !(g_walls[x][y] & WEST) && g_dist[x - 1][y] > (uint8_t)(d + 1)) {
      g_dist[x - 1][y] = d + 1;                               // Relax west neighbor
      g_qx[tail] = x - 1;                                     // Enqueue
      g_qy[tail] = y;                                         // Enqueue
      tail++;                                                 // Advance tail
    }
  }
}

/* -----------------------------------------------------------------------------
   MOVE DECISION (choose neighbor with lowest flood value)
   -----------------------------------------------------------------------------
   Algorithm:
   - Input: current cell (g_x,g_y), g_dist map, g_walls map
   - Output: next heading (0..3) or -1 if no move exists
*/

static int chooseNextHeading() {
  static const int dx[4] = {0, 1, 0, -1};                     // Heading to X delta
  static const int dy[4] = {1, 0, -1, 0};                     // Heading to Y delta
  static const uint8_t wb[4] = {NORTH, EAST, SOUTH, WEST};    // Heading to wall bit

  if (g_x < 0 || g_x >= MAZE_W || g_y < 0 || g_y >= MAZE_H) return -1; // Bounds fail-safe

  int bestDir = -1;                                           // Track best direction
  uint8_t best = 255;                                         // Track best flood distance (lower is better)

  for (int dir = 0; dir < 4; dir++) {                         // Evaluate 4 neighbors
    if (g_walls[g_x][g_y] & wb[dir]) continue;                // Skip if wall blocks this direction
    int nx = g_x + dx[dir];                                   // Neighbor x
    int ny = g_y + dy[dir];                                   // Neighbor y
    if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue; // Skip out-of-maze moves
    uint8_t nd = g_dist[nx][ny];                              // Neighbor distance value
    if (nd < best) {                                          // Choose minimum flood value
      best = nd;                                              // Store best value
      bestDir = dir;                                          // Store best heading
    }
  }

  return bestDir;                                             // Return chosen heading
}

/* -----------------------------------------------------------------------------
   TURN EXECUTION (uses existing sag360/sol360 functions unchanged)
   -----------------------------------------------------------------------------
   Algorithm:
   - Input: desired heading (0..3), current heading g_heading
   - Output: updates g_heading after turning in place
   - Timing:
     - Uses TURN_90_MS delay for each 90° step because the motor functions are
       open-loop. This must be tuned on real hardware.
*/

static void turnToHeading(int nextHeading) {
  int diff = (nextHeading - g_heading + 4) & 3;               // Normalize to 0..3 for turn selection

  if (diff == 1) {                                            // Need a 90° right turn
    sag360();                                                 // Start right turn motion (existing function)
    delay(TURN_90_MS);                                        // Hold motion for calibrated time
    motorStop();                                              // Stop after turn to reduce drift
  } else if (diff == 3) {                                     // Need a 90° left turn
    sol360();                                                 // Start left turn motion (existing function)
    delay(TURN_90_MS);                                        // Hold motion for calibrated time
    motorStop();                                              // Stop after turn to reduce drift
  } else if (diff == 2) {                                     // Need a 180° turn
    sag360();                                                 // First 90° right
    delay(TURN_90_MS);                                        // Time for first 90°
    motorStop();                                              // Stop between turns for stability
    delay(30);                                                // Small settle time
    sag360();                                                 // Second 90° right
    delay(TURN_90_MS);                                        // Time for second 90°
    motorStop();                                              // Stop after full 180°
  }

  g_heading = nextHeading & 3;                                // Update heading state after turn completes
}

/* -----------------------------------------------------------------------------
   FORWARD EXECUTION (uses existing ileri() unchanged)
   -----------------------------------------------------------------------------
   Algorithm:
   - Input: none (assumes robot faces the chosen heading)
   - Output: advances approximately one cell, then updates (g_x,g_y)
   - Timing:
     - Uses CELL_TRAVEL_MS because movement is open-loop in the preserved code.
*/

static void driveOneCellAndUpdatePose() {
  static const int dx[4] = {0, 1, 0, -1};                     // Heading to X delta
  static const int dy[4] = {1, 0, -1, 0};                     // Heading to Y delta

  ileri();                                                    // Start forward motion (existing function)
  delay(CELL_TRAVEL_MS);                                      // Keep moving long enough to cross ~20cm
  motorStop();                                                // Stop at cell boundary to stabilize mapping

  int nx = g_x + dx[g_heading & 3];                           // Compute next X after move
  int ny = g_y + dy[g_heading & 3];                           // Compute next Y after move

  if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) {     // Prevent invalid pose from out-of-maze move
    motorStop();                                              // Stop immediately for safety
    Serial.println("POSE FAULT");                             // Diagnostics: attempted to leave maze bounds
    while (true) delay(50);                                   // Fail-stop to avoid repeated damage
  }

  g_x = nx;                                                   // Commit pose update after bounds check
  g_y = ny;                                                   // Commit pose update after bounds check
}

/* -----------------------------------------------------------------------------
   GOAL CHECK (development goal until finish sensor is implemented)
   -----------------------------------------------------------------------------
   Algorithm:
   - Input: current pose (g_x,g_y)
   - Output: true if at one of the goal cells
*/

static bool isAtGoal() {
  for (int i = 0; i < GOAL_COUNT; i++) {                      // Check each goal cell
    if (g_x == g_goalX[i] && g_y == g_goalY[i]) return true;  // Match means goal reached
  }
  return false;                                               // Not at goal
}

/* -----------------------------------------------------------------------------
   SEARCH STEP (sense -> map -> plan -> move)
   -----------------------------------------------------------------------------
   Algorithm (plain language):
   - Read 3 ultrasonic distances (front/left/right) on shifted ECHO lines.
   - Convert to wall booleans using a threshold in centimeters.
   - Update wall map for current cell, mirroring walls into neighbor cells.
   - Recompute flood distances from goal cells via BFS.
   - Choose neighbor direction with minimum flood value and move one cell.
   Inputs:
   - Shifted ECHO signals (3 channels), TRIG output, current pose, heading.
   Outputs:
   - Updated map, updated pose, motor commands via existing movement functions.
   Timing/voltage constraints:
   - Exactly 3 shifted ECHO channels; any additional 5V signals are forbidden.
   - Each ping can block up to HCSR_ECHO_TIMEOUT_US; use conservative timeouts.
*/

static void searchAlgorithmStep() {
  if (g_x < 0 || g_x >= MAZE_W || g_y < 0 || g_y >= MAZE_H) { // Sanity check pose
    motorStop();                                              // Fail-safe stop on invalid pose
    Serial.println("STATE FAULT");                            // Diagnostics marker for debug
    while (true) delay(50);                                   // Halt to protect hardware
  }

  bool wF = isWallFront();                                    // Wall detected in front (shifted ECHO CH1)
  bool wL = isWallLeft();                                     // Wall detected on left (shifted ECHO CH2)
  bool wR = isWallRight();                                    // Wall detected on right (shifted ECHO CH3)

  updateWallsFromSensors(wF, wL, wR);                         // Convert relative walls into absolute map bits
  updateDistances();                                          // Recompute flood map using discovered walls

  if (isAtGoal()) {                                           // If reached goal, stop robot safely
    motorStop();                                              // Motors off
    Serial.println("GOAL");                                   // Diagnostics marker
    while (true) delay(50);                                   // Hold position (competition requires stop)
  }

  int nextHeading = chooseNextHeading();                      // Decide next move based on flood values
  if (nextHeading < 0) {                                      // If no valid move, stop to avoid crash
    motorStop();                                              // Motors off
    Serial.println("NO MOVE");                                // Diagnostics marker
    while (true) delay(50);                                   // Fail-stop
  }

  turnToHeading(nextHeading);                                 // Turn robot to face chosen neighbor direction
  driveOneCellAndUpdatePose();                                // Drive forward one cell and update pose

  Serial.print("POSE ");                                      // Human-readable pose output
  Serial.print(g_x);                                          // X coordinate
  Serial.print(',');                                          // Separator
  Serial.print(g_y);                                          // Y coordinate
  Serial.print("  H=");                                       // Heading label
  Serial.print(g_heading);                                    // Heading value (0..3)
  Serial.print("  D=");                                       // Distance label
  Serial.println(g_dist[g_x][g_y]);                           // Flood distance at new pose
}

/* -----------------------------------------------------------------------------
   MPU6050 SUPPORT (minimal, no external library)
   -----------------------------------------------------------------------------
   Algorithm:
   - Initialize MPU6050 by clearing sleep bit.
   - Read a couple of registers only for sanity (optional).
   - This code does not control the three shifters; it must operate at 3.3V I2C.
*/

static bool mpu6050Init() {
  Wire.beginTransmission(MPU6050_ADDR);                       // Start I2C transaction to MPU6050
  Wire.write(0x6B);                                           // PWR_MGMT_1 register
  Wire.write(0x00);                                           // Wake up device (clear sleep bit)
  if (Wire.endTransmission() != 0) {                          // End transaction; nonzero indicates error
    Serial.println("MPU6050 FAULT");                          // Diagnostic: IMU not responding
    return false;                                             // Report failure to caller
  }
  Serial.println("MPU6050 OK");                               // Diagnostic: IMU responded
  return true;                                                // Report success to caller
}

/* -----------------------------------------------------------------------------
   KY-040 SUPPORT (optional; must be 3.3V to keep shifter count at 3)
   -----------------------------------------------------------------------------
   Algorithm:
   - Poll CLK/DT and detect edges to increment/decrement wall threshold.
   - Only active before start; during competition runs you should disable it.
*/

static void ky040Service() {
  if (PIN_KY040_CLK < 0 || PIN_KY040_DT < 0) return;          // Feature disabled unless pins are configured
  static int lastClk = HIGH;                                   // Store last CLK state
  int clk = digitalRead(PIN_KY040_CLK);                        // Read current CLK
  if (clk != lastClk && clk == LOW) {                          // Detect falling edge (common KY-040 decoding)
    int dt = digitalRead(PIN_KY040_DT);                         // Read DT to determine direction
    if (dt == HIGH) g_wallThresholdCm = constrain(g_wallThresholdCm + 1, 5, 50); // Increase threshold
    else g_wallThresholdCm = constrain(g_wallThresholdCm - 1, 5, 50);            // Decrease threshold
    Serial.print("THRESH=");                                   // Diagnostic prefix
    Serial.println(g_wallThresholdCm);                          // Print new threshold
  }
  lastClk = clk;                                               // Update last CLK state
}

/* -----------------------------------------------------------------------------
   UNIT TESTS (compile-time stubs)
   -----------------------------------------------------------------------------
   Algorithm:
   - Exercise ileri() and geri() without requiring HC-SR04 hardware.
   - Does not modify the preserved movement functions.
*/

#ifdef UNIT_TEST
static void unitTestMovement() {
  Serial.begin(SERIAL_BAUD);                                   // Ensure serial is active for test output
  Serial.println("UNIT_TEST BEGIN");                            // Marker for test start
  motorStop();                                                 // Ensure clean start state
  ileri();                                                     // Call preserved ileri() function
  delay(200);                                                  // Short forward pulse for test
  motorStop();                                                 // Stop after pulse
  delay(200);                                                  // Pause between actions
  geri();                                                      // Call preserved geri() function
  delay(200);                                                  // Short backward pulse for test
  motorStop();                                                 // Stop after pulse
  Serial.println("UNIT_TEST END");                              // Marker for test end
}
#endif

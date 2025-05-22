#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <queue>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Forward declarations for BLE callbacks
class MyServerCallbacks;
class MyCallbacks;
void handleCommand(const std::string &value);

// ===== Pin Definitions =====
namespace Pins
{
    // Motor Pins
    constexpr uint8_t PWMA = 23; // Motor A PWM
    constexpr uint8_t AIN1 = 26; // Motor A Direction 1
    constexpr uint8_t AIN2 = 27; // Motor A Direction 2
    constexpr uint8_t PWMB = 22; // Motor B PWM
    constexpr uint8_t BIN1 = 21; // Motor B Direction 1
    constexpr uint8_t BIN2 = 19; // Motor B Direction 2
    constexpr uint8_t STBY = 18; // Motor Standby

    // I2C Pins
    constexpr uint8_t I2C_SDA = 4; // I2C Data
    constexpr uint8_t I2C_SCL = 5; // I2C Clock

    // Other Pins
    constexpr uint8_t BUZZER_PIN = 32; // Buzzer
    constexpr uint8_t BUILTIN_LED = 2; // Built-in LED
}

// ===== Display Configuration =====
namespace Display
{
    constexpr uint8_t SCREEN_WIDTH = 128;
    constexpr uint8_t SCREEN_HEIGHT = 64;
    constexpr uint8_t OLED_RESET = -1;
    constexpr uint8_t SCREEN_ADDRESS = 0x3C;
    constexpr uint8_t STATUS_LED_PIN = 2; // Built-in LED for Bluetooth status
}

// ===== Robot Configuration =====
namespace Robot
{
    // Motor Parameters
    constexpr uint8_t MOTOR_SPEED = 100;   // Default motor speed
    constexpr uint8_t TURN_SPEED = 100;    // Turning speed
    constexpr float MOTOR_A_FACTOR = 0.858; // Left motor factor ซ้าย
    constexpr float MOTOR_B_FACTOR = 1.000; // Right motor factor ขวา

    // TB6612FNG Configuration
    constexpr uint16_t PWM_FREQUENCY = 500;    // PWM frequency in Hz
    constexpr uint8_t PWM_RESOLUTION = 8;      // PWM resolution (8-bit = 0-255)
    constexpr uint8_t PWM_CHANNEL_A = 0;       // PWM channel for Motor A
    constexpr uint8_t PWM_CHANNEL_B = 1;       // PWM channel for Motor B
    constexpr uint8_t MIN_MOTOR_SPEED = 50;    // Minimum motor speed
    constexpr uint8_t MAX_MOTOR_SPEED = 255;   // Maximum motor speed
    constexpr uint8_t ACCELERATION_STEP = 5;   // Speed change per step for smooth acceleration

    // Timing Parameters
    uint32_t MOVEMENT_DURATION = 4000;         // Default movement duration
    uint32_t ROTATE_DURATION = 2000;           // Default rotation duration
    constexpr uint32_t MOTOR_TIMEOUT = 5000;   // Motor timeout
    constexpr float MAX_MOVEMENT_TIME = 30.0f; // Maximum movement time

    // Physical Parameters
    constexpr float WHEEL_DIAMETER = 3; // Wheel diameter in cm
    constexpr float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
}

// ===== BLE Configuration =====
namespace BLEConfig
{
    constexpr char DEVICE_NAME[] = "PUMO-ROBOT";
    constexpr char SERVICE_UUID[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    constexpr char CHARACTERISTIC_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
}

// ===== Global Objects =====
Adafruit_SSD1306 display(Display::SCREEN_WIDTH, Display::SCREEN_HEIGHT, &Wire, Display::OLED_RESET);
bool buzzerState = false;
constexpr float VERSION = 0.16;

// ===== Command Structure =====
struct Command
{
    char type;      // Command type (F, B, L, R, S, E)
    float duration; // Duration in seconds (for timed commands)
    bool isTimed;   // Whether the command is timed
};

// ===== Robot State =====
struct RobotState
{
    bool isTimedMovement = false;      // Whether currently executing timed movement
    uint32_t movementStartTime = 0;    // Start time of current movement
    uint32_t lastMotorCommandTime = 0; // Last time motor command was received
    bool emergencyStop = false;        // Emergency stop state
    char lastCommand = '\0';           // Last executed command
    bool isCommandExecuting = false;   // Whether a command is currently executing
} robotState;

// ===== Command Queue =====
std::queue<Command> commandQueue;
bool isProcessingQueue = false;

// ===== BLE Objects =====
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;

// ===== Display Functions =====
namespace EyeDisplay {
    constexpr int EYE_RADIUS = 24; // รัศมีตาใหญ่
    constexpr int EYE_X_LEFT = 40; // ตาซ้าย
    constexpr int EYE_X_RIGHT = 88; // ตาขวา
    constexpr int EYE_Y = 32; // ตรงกลางจอ
}

// สถานะการกระพริบตา
uint8_t eyeBlinkStep = 0; // 0 = ลืมตา, 1 = หลับครึ่ง, 2 = หลับเต็ม
uint32_t lastEyeBlinkTime = 0;
uint32_t eyeBlinkInterval = 300; // ms ต่อสเต็ป

void drawEyes(uint8_t step) {
    display.clearDisplay();
    int r = EyeDisplay::EYE_RADIUS;
    int y = EyeDisplay::EYE_Y;
    int xL = EyeDisplay::EYE_X_LEFT;
    int xR = EyeDisplay::EYE_X_RIGHT;

    // ตาซ้าย
    if (step == 0) {
        display.fillCircle(xL, y, r, SSD1306_WHITE);
        display.fillCircle(xL, y, r-6, SSD1306_BLACK);
        display.fillCircle(xL-6, y-6, 6, SSD1306_WHITE); // highlight
    } else if (step == 1) {
        display.fillCircle(xL, y, r, SSD1306_WHITE);
        display.fillRect(xL-r, y, 2*r, r, SSD1306_BLACK); // ปิดครึ่งล่าง
        display.fillCircle(xL-6, y-6, 6, SSD1306_WHITE);
    } else {
        display.fillCircle(xL, y, r, SSD1306_WHITE);
        display.fillRect(xL-r, y-r/2, 2*r, r+r/2, SSD1306_BLACK); // ปิดเกือบหมด
    }
    // ตาขวา
    if (step == 0) {
        display.fillCircle(xR, y, r, SSD1306_WHITE);
        display.fillCircle(xR, y, r-6, SSD1306_BLACK);
        display.fillCircle(xR-6, y-6, 6, SSD1306_WHITE);
    } else if (step == 1) {
        display.fillCircle(xR, y, r, SSD1306_WHITE);
        display.fillRect(xR-r, y, 2*r, r, SSD1306_BLACK);
        display.fillCircle(xR-6, y-6, 6, SSD1306_WHITE);
    } else {
        display.fillCircle(xR, y, r, SSD1306_WHITE);
        display.fillRect(xR-r, y-r/2, 2*r, r+r/2, SSD1306_BLACK);
    }
    display.display();
}

// ===== Function Declarations =====
void setupDisplay();
void setupMotors();
void setupBLE();
void updateDisplay(const char *motorAStatus, const char *motorBStatus);
void controlMotorA(uint8_t speed, bool isForward);
void controlMotorB(uint8_t speed, bool isForward);
void stopAllMotors();
void playBuzzerPattern(uint8_t pattern);
void controlBuzzer(bool isOn);
void checkMotorTimeout();
float parseSeconds(const std::string &value, size_t startPos);
void parseCommandSequence(const std::string &sequence);
void executeCommand(const Command &cmd);
void processCommandQueue();
void setMotorSpeed(uint8_t targetSpeed, bool isForward, bool isMotorA);

// ===== BLE Callbacks =====
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        digitalWrite(Display::STATUS_LED_PIN, HIGH); // Turn on LED when connected
        updateDisplay("Ready", "Ready");
        Serial.println("Bluetooth Connected");
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        digitalWrite(Display::STATUS_LED_PIN, LOW); // Turn off LED when disconnected
        updateDisplay("Disconnected", "Disconnected");
        Serial.println("Bluetooth Disconnected");
        pServer->getAdvertising()->start();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0)
        {
            handleCommand(value);
        }
    }
};

// ===== Setup Functions =====
void setupDisplay()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, Display::SCREEN_ADDRESS))
    {
        Serial.println("OLED initialization failed!");
    }
    else
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        updateDisplay("Ready", "Ready");
    }
}

void setupMotors()
{
    // Configure motor pins
    const uint8_t motorPins[] = {
        Pins::PWMA, Pins::AIN1, Pins::AIN2,
        Pins::PWMB, Pins::BIN1, Pins::BIN2,
        Pins::STBY
    };

    for (uint8_t pin : motorPins) {
        pinMode(pin, OUTPUT);
    }

    // Configure PWM channels
    ledcSetup(Robot::PWM_CHANNEL_A, Robot::PWM_FREQUENCY, Robot::PWM_RESOLUTION);
    ledcSetup(Robot::PWM_CHANNEL_B, Robot::PWM_FREQUENCY, Robot::PWM_RESOLUTION);
    
    // Attach PWM channels to pins
    ledcAttachPin(Pins::PWMA, Robot::PWM_CHANNEL_A);
    ledcAttachPin(Pins::PWMB, Robot::PWM_CHANNEL_B);

    // Enable motor driver
    digitalWrite(Pins::STBY, HIGH);
}

void setupBLE()
{
    Serial.println("Setting up BLE...");

    // Create BLE Device
    BLEDevice::init(BLEConfig::DEVICE_NAME);

    // Create BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Service
    BLEService *pService = pServer->createService(BLEConfig::SERVICE_UUID);

    // Create BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        BLEConfig::CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY);

    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());

    // Start Service and Advertising
    pService->start();
    pServer->getAdvertising()->start();

    Serial.println("BLE setup complete!");
    Serial.println("Device name: " + String(BLEConfig::DEVICE_NAME));
}

// ===== Motor Control Functions =====
void controlMotorA(uint8_t speed, bool isForward)
{
    // Limit speed to valid range
    speed = constrain(speed, Robot::MIN_MOTOR_SPEED, Robot::MAX_MOTOR_SPEED);
    
    if (speed == 0) {
        digitalWrite(Pins::AIN1, LOW);
        digitalWrite(Pins::AIN2, LOW);
        ledcWrite(Robot::PWM_CHANNEL_A, 0);
    } else {
        digitalWrite(Pins::AIN1, isForward);
        digitalWrite(Pins::AIN2, !isForward);
        ledcWrite(Robot::PWM_CHANNEL_A, speed * Robot::MOTOR_A_FACTOR);
    }
}

void controlMotorB(uint8_t speed, bool isForward)
{
    // Limit speed to valid range
    speed = constrain(speed, Robot::MIN_MOTOR_SPEED, Robot::MAX_MOTOR_SPEED);
    
    if (speed == 0) {
        digitalWrite(Pins::BIN1, LOW);
        digitalWrite(Pins::BIN2, LOW);
        ledcWrite(Robot::PWM_CHANNEL_B, 0);
    } else {
        digitalWrite(Pins::BIN1, !isForward);
        digitalWrite(Pins::BIN2, isForward);
        ledcWrite(Robot::PWM_CHANNEL_B, speed * Robot::MOTOR_B_FACTOR);
    }
}

void stopAllMotors()
{
    // Electrical brake by shorting motor terminals
    digitalWrite(Pins::AIN1, HIGH);
    digitalWrite(Pins::AIN2, HIGH);
    digitalWrite(Pins::BIN1, HIGH);
    digitalWrite(Pins::BIN2, HIGH);

    // Stop PWM
    ledcWrite(Robot::PWM_CHANNEL_A, 0);
    ledcWrite(Robot::PWM_CHANNEL_B, 0);
}

// New function for smooth acceleration
void setMotorSpeed(uint8_t targetSpeed, bool isForward, bool isMotorA)
{
    static uint8_t currentSpeedA = 0;
    static uint8_t currentSpeedB = 0;
    
    uint8_t &currentSpeed = isMotorA ? currentSpeedA : currentSpeedB;
    
    if (targetSpeed > currentSpeed) {
        // Accelerate
        for (uint8_t speed = currentSpeed; speed <= targetSpeed; speed += Robot::ACCELERATION_STEP) {
            if (isMotorA) {
                controlMotorA(speed, isForward);
            } else {
                controlMotorB(speed, isForward);
            }
            delay(50);
        }
    } else if (targetSpeed < currentSpeed) {
        // Decelerate
        for (uint8_t speed = currentSpeed; speed >= targetSpeed; speed -= Robot::ACCELERATION_STEP) {
            if (isMotorA) {
                controlMotorA(speed, isForward);
            } else {
                controlMotorB(speed, isForward);
            }
            delay(50);
        }
    }
    
    currentSpeed = targetSpeed;
}

// ===== Buzzer Control =====
void controlBuzzer(bool isOn)
{
    if (isOn != buzzerState)
    {
        if (isOn)
        {
            pinMode(Pins::BUZZER_PIN, OUTPUT);
            digitalWrite(Pins::BUZZER_PIN, HIGH);
        }
        else
        {
            digitalWrite(Pins::BUZZER_PIN, LOW);
            delay(5);
            pinMode(Pins::BUZZER_PIN, INPUT);
            digitalWrite(Pins::BUZZER_PIN, LOW);
        }
        buzzerState = isOn;
    }
}

void playBuzzerPattern(uint8_t pattern)
{
    controlBuzzer(false);
    delay(20);

    switch (pattern)
    {
    case 1: // Forward
        controlBuzzer(true);
        delay(50);
        controlBuzzer(false);
        break;

    case 2: // Backward
        controlBuzzer(true);
        delay(50);
        controlBuzzer(false);
        delay(50);
        controlBuzzer(true);
        delay(50);
        controlBuzzer(false);
        break;

    case 3: // Turn
        for (int i = 0; i < 3; i++)
        {
            controlBuzzer(true);
            delay(30);
            controlBuzzer(false);
            delay(30);
        }
        break;

    case 4: // Stop/Error
        controlBuzzer(true);
        delay(200);
        controlBuzzer(false);
        break;

    default:
        controlBuzzer(false);
        break;
    }

    controlBuzzer(false);
    delay(20);
}

// ===== Display Functions =====
void updateDisplay(const char *motorAStatus, const char *motorBStatus)
{
    display.clearDisplay();

    // Title and Version
    display.setCursor(0, 0);
    display.print("PUMO ROBOT v.");
    display.print(VERSION, 1);

    // Bluetooth Status
    display.setCursor(0, 16);
    display.print("BLE: ");
    display.print(BLEConfig::DEVICE_NAME);
    display.print(" (");
    display.print(deviceConnected ? "ON" : "OFF");
    display.print(")");

    // Last Command
    display.setCursor(0, 32);
    display.print("Last Command: ");
    if (robotState.lastCommand != '\0')
    {
        display.print(robotState.lastCommand);
        if (robotState.isTimedMovement)
        {
            display.print("T");
        }
    }
    else
    {
        display.print("None");
    }

    display.display();
}

// ===== Command Processing Functions =====
float parseSeconds(const std::string &value, size_t startPos)
{
    if (value.length() <= startPos)
    {
        Serial.println("Invalid time format");
        return 0;
    }
    try
    {
        std::string timeStr = value.substr(startPos);
        float seconds = std::stof(timeStr);
        if (seconds <= 0 || seconds > Robot::MAX_MOVEMENT_TIME)
        {
            char response[50];
            snprintf(response, sizeof(response), "Time must be between 0 and %.1f seconds", Robot::MAX_MOVEMENT_TIME);
            Serial.println(response);
            return 0;
        }
        return seconds;
    }
    catch (...)
    {
        Serial.println("Invalid time format");
        return 0;
    }
}

void handleCommand(const std::string &value)
{
    if (value.find('-') != std::string::npos)
    {
        parseCommandSequence(value);
    }
    else
    {
        Command singleCmd;
        if (value.length() >= 2 && (value[1] == 'T' || value[1] == 'R'))
        {
            singleCmd.type = value[0];
            singleCmd.isTimed = true;
            singleCmd.duration = parseSeconds(value, 2);
            if (singleCmd.duration > 0)
            {
                commandQueue.push(singleCmd);
                char response[50];
                snprintf(response, sizeof(response), "Command %cT%.1f added to queue", singleCmd.type, singleCmd.duration);
                Serial.println(response);
            }
        }
        else if (value.length() == 1)
        {
            singleCmd.type = value[0];
            singleCmd.isTimed = false;
            singleCmd.duration = 0;
            commandQueue.push(singleCmd);
            char response[50];
            snprintf(response, sizeof(response), "Command %c added to queue", singleCmd.type);
            Serial.println(response);
        }

        if (!commandQueue.empty() && !isProcessingQueue)
        {
            isProcessingQueue = true;
            processCommandQueue();
        }
    }
}

void parseCommandSequence(const std::string &sequence)
{
    size_t start = 0;
    size_t end = sequence.find('-');

    while (end != std::string::npos)
    {
        std::string cmd = sequence.substr(start, end - start);
        Command newCmd;

        if (cmd.length() >= 2 && (cmd[1] == 'T' || cmd[1] == 'R'))
        {
            newCmd.type = cmd[0];
            newCmd.isTimed = true;
            newCmd.duration = parseSeconds(cmd, 2);
            if (newCmd.duration > 0)
            {
                commandQueue.push(newCmd);
            }
        }
        else if (cmd.length() == 1)
        {
            newCmd.type = cmd[0];
            newCmd.isTimed = false;
            newCmd.duration = 0;
            commandQueue.push(newCmd);
        }

        start = end + 1;
        end = sequence.find('-', start);
    }

    // Process last command
    std::string lastCmd = sequence.substr(start);
    Command newCmd;

    if (lastCmd.length() >= 2 && (lastCmd[1] == 'T' || lastCmd[1] == 'R'))
    {
        newCmd.type = lastCmd[0];
        newCmd.isTimed = true;
        newCmd.duration = parseSeconds(lastCmd, 2);
        if (newCmd.duration > 0)
        {
            commandQueue.push(newCmd);
        }
    }
    else if (lastCmd.length() == 1)
    {
        newCmd.type = lastCmd[0];
        newCmd.isTimed = false;
        newCmd.duration = 0;
        commandQueue.push(newCmd);
    }

    if (!commandQueue.empty() && !isProcessingQueue)
    {
        isProcessingQueue = true;
        processCommandQueue();
    }
}

void executeCommand(const Command &cmd)
{
    robotState.isCommandExecuting = true;
    robotState.lastCommand = cmd.type;
    robotState.lastMotorCommandTime = millis();

    switch (cmd.type)
    {
    case 'F':
        if (cmd.isTimed)
        {
            Robot::MOVEMENT_DURATION = (uint32_t)(cmd.duration * 1000);
            controlMotorA(Robot::MOTOR_SPEED, true);
            controlMotorB(Robot::MOTOR_SPEED, true);
            robotState.movementStartTime = millis();
            robotState.isTimedMovement = true;
            playBuzzerPattern(1);
            char response[50];
            snprintf(response, sizeof(response), "Moving Forward for %.1f seconds", cmd.duration);
            Serial.println(response);
        }
        else
        {
            controlMotorA(Robot::MOTOR_SPEED, true);
            controlMotorB(Robot::MOTOR_SPEED, true);
            playBuzzerPattern(1);
            Serial.println("Moving Forward");
            // Add a default duration for non-timed commands
            Robot::MOVEMENT_DURATION = 1000; // 1 second default duration
            robotState.movementStartTime = millis();
            robotState.isTimedMovement = true;
        }
        break;

    case 'B':
        if (cmd.isTimed)
        {
            Robot::MOVEMENT_DURATION = (uint32_t)(cmd.duration * 1000);
            controlMotorA(Robot::MOTOR_SPEED, false);
            controlMotorB(Robot::MOTOR_SPEED, false);
            robotState.movementStartTime = millis();
            robotState.isTimedMovement = true;
            playBuzzerPattern(2);
            char response[50];
            snprintf(response, sizeof(response), "Moving Backward for %.1f seconds", cmd.duration);
            Serial.println(response);
        }
        else
        {
            controlMotorA(Robot::MOTOR_SPEED, false);
            controlMotorB(Robot::MOTOR_SPEED, false);
            playBuzzerPattern(2);
            Serial.println("Moving Backward");
            // Add a default duration for non-timed commands
            Robot::MOVEMENT_DURATION = 1000; // 1 second default duration
            robotState.movementStartTime = millis();
            robotState.isTimedMovement = true;
        }
        break;

    case 'L':
        if (cmd.isTimed)
        {
            Robot::ROTATE_DURATION = (uint32_t)(cmd.duration * 1000);
            controlMotorA(Robot::TURN_SPEED, false);
            controlMotorB(Robot::TURN_SPEED, true);
            robotState.movementStartTime = millis();
            robotState.isTimedMovement = true;
            playBuzzerPattern(3);
            char response[50];
            snprintf(response, sizeof(response), "Rotating Left for %.1f seconds", cmd.duration);
            Serial.println(response);
        }
        else
        {
            controlMotorA(0, true);
            controlMotorB(Robot::MOTOR_SPEED, true);
            playBuzzerPattern(3);
            Serial.println("Turning Left");
            // Add a default duration for non-timed commands
            Robot::ROTATE_DURATION = 1000; // 1 second default duration
            robotState.movementStartTime = millis();
            robotState.isTimedMovement = true;
        }
        break;

    case 'R':
        if (cmd.isTimed)
        {
            Robot::ROTATE_DURATION = (uint32_t)(cmd.duration * 1000);
            controlMotorA(Robot::TURN_SPEED, true);
            controlMotorB(Robot::TURN_SPEED, false);
            robotState.movementStartTime = millis();
            robotState.isTimedMovement = true;
            playBuzzerPattern(3);
            char response[50];
            snprintf(response, sizeof(response), "Rotating Right for %.1f seconds", cmd.duration);
            Serial.println(response);
        }
        else
        {
            controlMotorA(Robot::MOTOR_SPEED, true);
            controlMotorB(0, true);
            playBuzzerPattern(3);
            Serial.println("Turning Right");
            // Add a default duration for non-timed commands
            Robot::ROTATE_DURATION = 1000; // 1 second default duration
            robotState.movementStartTime = millis();
            robotState.isTimedMovement = true;
        }
        break;

    case 'S':
        stopAllMotors();
        robotState.isTimedMovement = false;
        playBuzzerPattern(4);
        Serial.println("Stopped");
        break;

    case 'E':
        robotState.emergencyStop = true;
        stopAllMotors();
        playBuzzerPattern(4);
        Serial.println("Emergency Stop Activated");
        break;
    }
}

void processCommandQueue()
{
    static bool needDelay = false;  // เพิ่มตัวแปรเพื่อจัดการ delay

    if (!commandQueue.empty()) {
        // ถ้าต้องการ delay ก่อนคำสั่งถัดไป
        if (needDelay) {
            stopAllMotors();
            updateDisplay("Waiting", "1 second");
            Serial.println("Waiting 1 second before next command");
            delay(1000);
        }

        Command currentCmd = commandQueue.front();
        commandQueue.pop();
        executeCommand(currentCmd);

        // ตั้งค่าให้ delay ก่อนคำสั่งถัดไป
        needDelay = true;

        if (commandQueue.empty()) {
            Serial.println("All commands completed");
            needDelay = false;  // รีเซ็ตสถานะเมื่อทำงานเสร็จ
        }
    } else {
        isProcessingQueue = false;
        robotState.isCommandExecuting = false;
        needDelay = false;  // รีเซ็ตสถานะ
    }
}

void checkMotorTimeout()
{
    if (robotState.isTimedMovement &&
        (millis() - robotState.lastMotorCommandTime > Robot::MOTOR_TIMEOUT))
    {
        stopAllMotors();
        robotState.isTimedMovement = false;
        updateDisplay("Timeout", "Timeout");
        playBuzzerPattern(4);
        Serial.println("Motor timeout - Stopped");
    }
}

// ===== Main Functions =====
void setup()
{
    Serial.begin(115200);
    Serial.println("Starting setup...");

    // Initialize pins
    pinMode(Pins::BUZZER_PIN, INPUT);
    digitalWrite(Pins::BUZZER_PIN, LOW);
    delay(20);

    pinMode(Display::STATUS_LED_PIN, OUTPUT);
    digitalWrite(Display::STATUS_LED_PIN, LOW);

    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);

    setupMotors();
    display.begin(SSD1306_SWITCHCAPVCC, Display::SCREEN_ADDRESS);
    display.clearDisplay();
    drawEyes(0); // เริ่มต้นลืมตา
    setupBLE();
    controlBuzzer(false);
    Serial.println("Setup complete");
}

void loop()
{
    if (robotState.emergencyStop)
    {
        stopAllMotors();
        return;
    }

    checkMotorTimeout();

    // Blink LED when not connected
    if (!deviceConnected)
    {
        static uint32_t lastBlinkTime = 0;
        if (millis() - lastBlinkTime >= 500)
        { // Blink every 500ms
            digitalWrite(Display::STATUS_LED_PIN, !digitalRead(Display::STATUS_LED_PIN));
            lastBlinkTime = millis();
        }
    }

    if (robotState.isTimedMovement)
    {
        uint32_t currentTime = millis();
        uint32_t elapsedTime = currentTime - robotState.movementStartTime;
        uint32_t targetDuration = 0;

        if (robotState.lastCommand == 'L' || robotState.lastCommand == 'R')
        {
            targetDuration = Robot::ROTATE_DURATION;
        }
        else
        {
            targetDuration = Robot::MOVEMENT_DURATION;
        }

        if (elapsedTime < targetDuration)
        {
            static uint32_t lastBuzzerTime = 0;
            if (currentTime - lastBuzzerTime >= 500)
            {
                controlBuzzer(!buzzerState);
                lastBuzzerTime = currentTime;
            }
        }

        if (elapsedTime >= targetDuration)
        {
            stopAllMotors();
            robotState.isTimedMovement = false;
            controlBuzzer(false);
            
            // เพิ่ม delay ก่อนเรียก processCommandQueue
            delay(100);  // delay เล็กน้อยเพื่อให้มอเตอร์หยุดสนิท
            
            processCommandQueue();
        }
    }

    // กระพริบตาแยกจากมอเตอร์
    uint32_t now = millis();
    static bool blinking = false;
    static uint32_t blinkStart = 0;
    if (!blinking && now - lastEyeBlinkTime > 2000) { // กระพริบทุก 2 วินาที
        blinking = true;
        blinkStart = now;
        eyeBlinkStep = 0;
    }
    if (blinking) {
        if (now - blinkStart > eyeBlinkInterval * eyeBlinkStep && eyeBlinkStep < 3) {
            drawEyes(eyeBlinkStep);
            eyeBlinkStep++;
            if (eyeBlinkStep == 3) {
                blinking = false;
                lastEyeBlinkTime = now;
                drawEyes(0); // กลับมาลืมตา
            }
        }
    }

    delay(10);
}
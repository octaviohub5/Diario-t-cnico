PF Robotics Point 1. Mobility management
The mobility management of our autonomous vehicle is based on the adaptation and engineering of systems on a pre-existing commercial platform, specifically the chassis of the 1:24 scale Soluna RC model. The choice of this 18 x 8.5 cm chassis is justified by its high-performance mechanical properties: it features a shock-resistant PVC structure ideal for withstanding potential impacts against the track's random walls and a compact design that facilitates navigation in confined spaces. For propulsion, the DC motors integrated into the original vehicle have been retained and implemented; although these motors are designed to reach speeds of up to 20 km/h in their factory configuration (Drift type), our technical implementation prioritizes control over top speed. By using an H-bridge controller and Pulse Width Modulation (PWM) signal management from the source code, we limit the power delivered to an approximate 39% duty cycle (value 100/255). This electronic downshift is a critical engineering decision that transforms excess engine revolutions into manageable torque, preventing uncontrolled skidding on the track and ensuring that the vehicle's speed is compatible with the ultrasonic sensors' refresh rate.

The motion control logic operates under a reactive navigation scheme: upon detecting frontal obstacles within 20 cm, the system leverages the chassis' agility to perform a braking and reversing maneuver for one second, followed by a change of direction toward the area with greater clearance detected by the side sensors. Regarding component integration, a hybrid assembly has been implemented on the PVC body; while the battery and drivetrain remain in their original position to maintain a low center of gravity and mechanical stability, custom structures designed in CAD and manufactured using 3D printing have been added (see attached files). These printed parts function as mounting interfaces to hold the microcontroller and, fundamentally, to position the three ultrasonic sensors on the top of the chassis with precise and rigid angular alignment, ensuring reliable readings without altering the aerodynamics or weight distribution of the vehicle.

-Point 2. Power and sense management
The vehicle's power management and sensor instrumentation have been designed using an architecture that ensures data integrity and autonomy during competition. A 2-cell (2S) Lithium Polymer (LiPo) battery with a capacity of 2800 mAh and a nominal voltage of 7.4V was selected as the primary power source. The choice of this specific battery, with a 45C discharge rate, is critical from an engineering perspective: this high discharge capacity ensures that the motors can handle current spikes during start-up and sharp turns without causing a voltage sag that could reset the microcontroller. To manage this power efficiently, the control logic is not powered directly from the battery; instead, a Buck DC-DC converter (model LM2596) is implemented to reduce and regulate the 7.4V voltage to a stable 5V level for the ESP32 microcontroller and the sensors. The use of this switched-mode regulator, visible in the central chassis mount, is superior to traditional linear regulators because it minimizes energy dissipation as heat, maximizing the vehicle's run time on the track. Furthermore, the motors' energy consumption is optimized by software using a 39% PWM signal (value 100), preventing unnecessary battery drain by limiting the average current delivered to the actuators without sacrificing the torque required for mobility.
Regarding sensor management, the vehicle implements a perception system using three ultrasonic sensors arranged in a fan configuration (left, center, right). These sensors were selected for their reliability in detecting the rigid walls of the track at the required operating distance of 20 cm. However, the engineering challenge of using multiple ultrasonic sensors simultaneously is interference or "crosstalk." To address this, our control algorithm implements sequential triggering: the microcontroller activates the front sensor's trigger, waits for its echo, calculates the distance, and only then proceeds to activate the side sensors. This communication protocol, which includes micro-stabilization delays (delayMicroseconds), ensures that the echo received by a sensor corresponds exclusively to its own emitted pulse and not to a bounce from an adjacent sensor, guaranteeing a clean and accurate reading of the environment. This orchestration between efficient power management through switched regulation and sequential sensor reading allows the vehicle to maintain a robust autonomous state, processing complex navigation decisions without false positives and with a stable power supply.

-Point 3. Obstacle management

● Discussion on obstacle management strategy

The strategy for navigating the obstacle course was designed by analyzing
the physical interaction between our control algorithm and the regulatory specifications
of the 3000 x 3000 mm playing mat. A fundamental engineering decision
was the selection of ultrasonic sensors instead of
infrared sensors; given that the rules stipulate that the interior and exterior walls
are black, optical sensors could have failed due to light absorption,
while our HC-SR04 sensors guarantee reliable detection by operating
using sound waves that bounce effectively off the rigid 100 mm high surface of the walls.
The The
source code implements a virtual safety buffer defined in the condition if (distance1 < 20), which stipulates that the
vehicle must never come within 20 cm of any physical structure
. When operating on a track with randomly placed walls, the vehicle
cannot pre-calculate an optimal route, so the software transforms the
car into a fully reactive agent: when it encounters a blockage in its
path, the backward instruction (delay(1000)) allows the chassis to
move 100 mm away from the wall to avoid mechanical collisions during the
turn, while the logical comparison of the side sensors ensures
that the vehicle always redirects itself to the widest sector of the track,
taking advantage of the dimensions of the mat to maintain a continuous flow of
movement during the three laps.

-Pseudocode
    START PROGRAM
    SET motor pins as OUTPUT
    SET sensor pins (Trigger as OUTPUT, Echo as INPUT)
    START Serial communication
    REPEAT CONSTANTLY (LOOP):

        // 1. Data acquisition
        FIRE Front ultrasonic pulse
        CALCULATE front_distance (cm)

        FIRE Left ultrasonic pulse
        CALCULATE left_distance (cm)

        FIRE Right ultrasonic pulse
        CALCULATE right_distance (cm)

        // 2. Power Management
        SET motor speed to 39% (PWM 100)

        // 3. Reactive Navigation Logic
        IF (front_distance < 20 cm) THEN:
            // There is an obstacle in front, decide to evade
            
            IF (left_distance < right_distance) THEN:
                // The right side has more space
                EXECUTE “Back” movement
                WAIT 1000 milliseconds
                EXECUTE “Turn Right” movement
            
            ELSE (Left side has more space):
                EXECUTE “Backward” movement
                WAIT 1000 milliseconds
                EXECUTE “Turn Left” movement
            END IF

        ELSE:
            // No obstacles nearby
            EXECUTE “Forward” movement
        END IF
        END REPEAT

-Source Code with Detailed Comments

// --- DEFINITION OF PINS AND CONSTANTS ---

// Ultrasonic Sensors (Echo = Input, Trig = Output)
const int echoe = 35; // Echo Front
const int trige = 32; // Trigger Front
const int echoi = 26; // Echo Left
const int trigi = 27; // Trigger Left
const int echod = 21; // Echo Right
const int trigd = 23; // Trigger Right

// H-Bridge Control Pins (Direction of rotation)
const int pin2 = 2;   // IN1
const int pin4 = 4;   // IN2
const int pin18 = 18; // IN3
const int pin19 = 19; // IN4

// Enable Pins (PWM Speed Control)
const int pinEN1 = 12; // Enable Motor A
const int pinEN2 = 13; // Enable Motor B
void setup() {

  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure H-bridge pins as output
  pinMode(pin2, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(pin18, OUTPUT);
  pinMode(pin19, OUTPUT);
  
  // PWM pin configuration
  pinMode(pinEN1, OUTPUT);
  pinMode(pinEN2, OUTPUT);

  // Sensor pin configuration
  pinMode(echoe, INPUT);
  pinMode(trige, OUTPUT);
  pinMode(echoi, INPUT);
  pinMode(trigi, OUTPUT);
  pinMode(echod, INPUT);
  pinMode(trigd, OUTPUT);
 // Ensure that triggers start in low state
  digitalWrite(trige, LOW); 
  digitalWrite(trigi, LOW); 
  digitalWrite(trigd, LOW); 
}
void loop() {

  // --- PHASE 1: SEQUENTIAL READING OF SENSORS ---
  // Read one by one to avoid echo interference (crosstalk)

  // 1.1 Front Sensor
  digitalWrite(trige, LOW); 
  delayMicroseconds(2);
  digitalWrite(trige, HIGH); // Send 10us pulse
  delayMicroseconds(10);
  digitalWrite(trige, LOW);
  
  long time1 = pulseIn(echoe, HIGH); // Measurement of flight time
  // Physical formula: Distance = (Time * Speed of Sound 0.0343 cm/us) / 2
  float distance1 = (time1 * 0.0343) / 2;

  // 1.2 Left Sensor
  digitalWrite(trigi, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigi, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigi, LOW);
  
  long time2 = pulseIn(echoi, HIGH);
  float distance2 = (time2 * 0.0343) / 2;

  // 1.3 Right Sensor
  digitalWrite(trigd, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigd, HIGH); 

PF Robotics Point 7. Engineering Factor
The chassis used to build the autonomous vehicle was purchased and modified from a remote-controlled car sold on Mercado Libre
  delayMicroseconds(10);
  digitalWrite(trigd, LOW);
  
  long time3 = pulseIn(echod, HIGH);
  float distance3 = (time3 * 0.0343) / 2;



// --- PHASE 2: POWER MANAGEMENT ---
  // Electronic speed limitation using PWM (100/255 ≈ 39%)
  analogWrite(pinEN1, 100);
  analogWrite(pinEN2, 100);

 // --- PHASE 3: STATE MACHINE (NAVIGATION LOGIC) ---

  // CASE A: Front Obstacle Detected (< 20cm)
  // Sub-case: The RIGHT side has more free space (Left < Right)
  if (distance1 < 20 && distance2 < distance3){
    // 1. Reverse Maneuver (Gain turning radius)
    digitalWrite(pin2, LOW);  digitalWrite(pin18, LOW);
    digitalWrite(pin4, HIGH); digitalWrite(pin19, HIGH);
    Serial.println(“Backward - Evasion”);
    delay(1000); // 1-second pause in reverse
    
    // 2. Set up right turn
    digitalWrite(pin2, HIGH); digitalWrite(pin18, HIGH);
    digitalWrite(pin4, LOW);  digitalWrite(pin19, LOW);
    Serial.println(“Right Turn”);
}

     // CASE B: Front Obstacle Detected (< 20cm)
  // Sub-case: The LEFT side has more free space (Left > Right)
  else if (distance1 < 20 && distance2 > distance3){
    // 1. Reverse Maneuver
    digitalWrite(pin2, HIGH); digitalWrite(pin18, LOW);
    digitalWrite(pin4, LOW);  digitalWrite(pin19, HIGH);
    Serial.println(“Backward - Evasion”);
    delay(1000); // 1-second pause in reverse
    
    // 2. Set Left Turn
    digitalWrite(pin2, LOW);  digitalWrite(pin18, HIGH);
    digitalWrite(pin4, HIGH); digitalWrite(pin19, LOW);
    Serial.println(“Left Turn”);
  }

  // CASE C: Clear Path (Default)
  else {
    // Constant forward movement
    digitalWrite(pin2, HIGH); digitalWrite(pin18, HIGH);
    digitalWrite(pin4, HIGH); digitalWrite(pin19, LOW);
    Serial.println(“Forward Movement”);
  } }

PF Robotics Point 7. Engineering Factor
The chassis used to build the autonomous vehicle was purchased and modified from a
remote-controlled car sold on Mercado Libre.

Car Modifications
1. Internal Changes: The chassis of the remote-controlled car was opened and the circuit board was removed, disconnecting all the connected wires, which were:
1.1 - Front and rear lights
1.2 - Front motor for Ackerman operation and rear motor for forward/reverse movement
1.3 - On/Off switch
2. Removal of 4x4 Traction
The front and rear mechanisms that provided four-wheel drive via a bar transmitting movement were also opened. This bar was removed to eliminate the front-wheel drive function.
3. Internal Expansion
A box was opened that originally housed the batteries that came with the car. Since the batteries were no longer needed, the space was cut to create more room inside.

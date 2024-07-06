#include <avr/io.h>
#include <avr/interrupt.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define F_CPU 16000000UL // Define CPU frequency

// Define the sensor pins
#define NUM_SENSORS 5
const uint8_t sensorPins[NUM_SENSORS] = {PC0, PC1, PC2, PC3, PC4};

// Define the weights for each sensor
const int weights[NUM_SENSORS] = {-2, -1, 0, 1, 2};

// PID constants
float Kp = 11.0;
float Ki = 1.0;
float Kd = 0.05;

//Distance limit to start stoping
uint8_t ditance_stop = 20;

// PID variables
float error = 0, lastError = 0;
float integral = 0, derivative = 0;
float correction = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Motor control pins
#define leftMotorPin DDB1
#define rightMotorPin DDB2

// Encoder pins
#define LEFT_ENCODER_PIN_A DDD2
#define RIGHT_ENCODER_PIN_A DDD3
#define RIGHT_ENCODER_PIN_B DDB6
#define LEFT_ENCODER_PIN_B  DDB7

#define WHEEL_RADIUS 0.03 // 3 cm wheel radius
#define COUNTS_PER_REVOLUTION 690 // Example value, adjust based on your encoder
#define TIMER_INTERVAL_MS 100 // Timer interval in milliseconds

#define MIN_TIME_INTERVAL 1000 // Minimum time interval between table count updates

// Ultrasonic sensor pins
#define trigPin1 DDD5
#define echoPin1 DDD6

#define BUTTON_PIN_INCREMENT    DDD0
#define BUTTON_PIN_DECREMENT    DDD1
#define BUTTON_PIN_START        DDD7
#define TOUCH_PIN_SERVED        DDD4

#define MAX_TABLE_COUNT 4

// Segment pin mappings (assuming PD0-PD6 are connected to the segments)
#define SEG_A PD0
#define SEG_B PD1
#define SEG_C PD2
#define SEG_D PD3
#define SEG_E PD4
#define SEG_F PD5
#define SEG_G PD6

// Maximum speeds and accelerations in MPS and MPS^2
#define MAX_SPEED 0.000186411
#define MAX_ACCELERATION 3.36
#define MAX_DECELERATION 5.59

#define accelerationStep 2
#define decelerationStep 2

#define M_PI 3.142

#define currentSpeed 0.00009
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


volatile uint8_t table_number = 0;
volatile uint8_t table_count = 0;
volatile uint8_t start_state = 0;

// Encoder variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
const long countsPer90DegreeTurn = 200; // Encoder count for turning 90 degrees

// Speed profile parameters
float leftSpeedMPS = 0.0;
float rightSpeedMPS = 0.0;
float leftAccelerationMPS2 = 0.0;
float rightAccelerationMPS2 = 0.0;

volatile float leftSpeedPrev = 0.0;
volatile float rightSpeedPrev = 0.0;

volatile float leftEncoderCountPrev = 0.0;
volatile float rightEncoderCountPrev = 0.0;

// Stop parameters
const int stopCountThreshold = 10;  // The count after which the robot should stop
int stopCount = 0;

uint8_t mode = 2; //0: normal operation , 1: decelerating, 2: accelerating
uint32_t previous_table_count_update_time = 0; // Time of the last table count update

volatile uint32_t milliseconds = 0; // Milliseconds counter

uint8_t serving_state = 1; // 1: not served, 2: served, 3: return to starting point

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t millis();
long readUltrasonicDistance(uint8_t trigPin, uint8_t echoPin);
long pulseIn(uint8_t pin, uint8_t state);

void _delay_ms(uint16_t ms) {
	uint32_t start = millis();
	while(millis() - start < ms){

	}
}

void _delay_us(uint16_t us) {
	uint32_t start = millis()*1000;
	while(millis()*1000 - start < us){

	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setMotorSpeed(int leftSpeed, int rightSpeed) {
	// Function to set motor speeds
	// Example using PWM for motor driver
	OCR1A = leftSpeed;
	OCR1B = rightSpeed;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turnBend(uint8_t angle) {

	//Calculate the bend angle
	uint8_t bendAngle = angle * countsPer90DegreeTurn / 90;

	// Reset encoder counts
	leftEncoderCount = 0;
	rightEncoderCount = 0;

	// Accelerate to target speed
	int turnSpeed = 0;
	while (turnSpeed < MAX_SPEED) {
		turnSpeed += accelerationStep;
		if (turnSpeed > MAX_SPEED) {
			turnSpeed = MAX_SPEED;
		}
		setMotorSpeed(turnSpeed, -turnSpeed);
		_delay_ms(50);
	}

	// Maintain target speed until the robot has turned 90 degrees
	while (leftEncoderCount < bendAngle && rightEncoderCount < bendAngle);

	// Decelerate to stop
	while (turnSpeed > 0) {
		turnSpeed -= decelerationStep;
		if (turnSpeed < 0) {
			turnSpeed = 0;
		}
		setMotorSpeed(turnSpeed, -turnSpeed);
		_delay_ms(50);
	}

	// Stop the motors
	setMotorSpeed(0, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveToStartingPoint() {
	// Reset encoder counts
	leftEncoderCount = 0;
	rightEncoderCount = 0;

	// Set motors to move the robot forward
	setMotorSpeed(MAX_SPEED, MAX_SPEED);

	// Move the robot back to the starting point
	// Adjust the distance based on your setup
	while (leftEncoderCount < countsPer90DegreeTurn * 4 && rightEncoderCount < countsPer90DegreeTurn * 4);

	// Decelerate to stop
	int moveSpeed = MAX_SPEED;
	while (moveSpeed > 0) {
		moveSpeed -= decelerationStep;
		if (moveSpeed < 0) {
			moveSpeed = 0;
		}
		setMotorSpeed(moveSpeed, moveSpeed);
		_delay_ms(50);
	}

	// Stop the motors
	setMotorSpeed(0, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t isObstacleDetected() {
	return (readUltrasonicDistance(trigPin1, echoPin1) < ditance_stop);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

long readUltrasonicDistance(uint8_t trigPin, uint8_t echoPin) {
	long duration, distance;
	PORTD &= ~(1 << trigPin); // Clear the trigger pin
	_delay_us(2);
	PORTD |= (1 << trigPin); // Set the trigger pin
	_delay_us(10);
	PORTD &= ~(1 << trigPin); // Clear the trigger pin

	duration = pulseIn(echoPin, 1);
	distance = (duration / 2) / 29.1; // Convert duration to distance

	return distance;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(INT0_vect) {
	// Check the state of the second pin of the left encoder
	if (PIND & (1 << LEFT_ENCODER_PIN_B)) {
		leftEncoderCount--;
		} else {
		leftEncoderCount++;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(INT1_vect) {
	// Check the state of the second pin of the right encoder
	if (PIND & (1 << RIGHT_ENCODER_PIN_B)) {
		rightEncoderCount--;
		} else {
		rightEncoderCount++;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER1_COMPA_vect) {
	// Timer1 interrupt service routine, called every TIMER_INTERVAL_MS milliseconds
	int leftEncoderDelta = leftEncoderCount - leftEncoderCountPrev;
	int rightEncoderDelta = rightEncoderCount - rightEncoderCountPrev;
	leftEncoderCountPrev = leftEncoderCount;
	rightEncoderCountPrev = rightEncoderCount;

	// Calculate the speed in counts per second
	float leftSpeedCPS = (float)leftEncoderDelta / (TIMER_INTERVAL_MS / 1000.0);
	float rightSpeedCPS = (float)rightEncoderDelta / (TIMER_INTERVAL_MS / 1000.0);

	// Calculate the speed in meters per second
	leftSpeedMPS = leftSpeedCPS / COUNTS_PER_REVOLUTION * 2 * M_PI * WHEEL_RADIUS;
	rightSpeedMPS = rightSpeedCPS / COUNTS_PER_REVOLUTION * 2 * M_PI * WHEEL_RADIUS;

	// Calculate the acceleration in meters per second squared
	leftAccelerationMPS2 = (leftSpeedMPS - leftSpeedPrev) / (TIMER_INTERVAL_MS / 1000.0);
	rightAccelerationMPS2 = (rightSpeedMPS - rightSpeedPrev) / (TIMER_INTERVAL_MS / 1000.0);

	// Update previous speeds
	leftSpeedPrev = leftSpeedMPS;
	rightSpeedPrev = rightSpeedMPS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER0_COMPA_vect) {
	// Increment milliseconds every 1 ms
	milliseconds++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void init_timer() {
	// Set up Timer1 for CTC mode
	TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode
	TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
	OCR1A = (F_CPU / 1000) * TIMER_INTERVAL_MS / 64 - 1; // Set CTC compare value for 100ms interval with 64 prescaler
	TCCR1B |= (1 << CS11) | (1 << CS10); // Start timer at Fcpu/64

	// Configure Timer0 for CTC mode to generate a 1 ms interrupt
	TCCR0A |= (1 << WGM01); // CTC mode
	OCR0A = F_CPU / 1000 / 64 - 1; // Set CTC compare value for 1 ms interval with 64 prescaler
	TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
	TCCR0B |= (1 << CS01) | (1 << CS00); // Start timer at Fcpu/64
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t millis() {
	uint32_t ms;
	// Disable interrupts to read milliseconds safely
	cli();
	ms = milliseconds;
	sei();
	return ms;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t debounce_button(uint8_t pin) {
	if (!(PINB & (1 << pin))) {
		_delay_ms(50);
		if (!(PINB & (1 << pin))) {
			return 1;
		}
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

int readAnalog(uint8_t pin) {
	// Configure ADC
	ADMUX = (1 << REFS0) | (pin & 0x0F);

	///set prescaler to 128 and enable ADC
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));

	// Return ADC value
	return ADC;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

int constrain(int value, int min, int max) {
	if (value < min) return min;
	if (value > max) return max;
	return value;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

long pulseIn(uint8_t pin, uint8_t state) {
	long width = 0;
	unsigned long numloops = 0;

	// wait for any previous pulse to end
	while (!(PIND & (1 << pin))) {
		if (numloops++ == 1000000) {
			return 0;
		}
	}

	// wait for the pulse to start
	while (PIND & (1 << pin)) {
		if (numloops++ == 1000000) {
			return 0;
		}
	}

	// wait for the pulse to stop
	while (!(PIND & (1 << pin))) {
		if (numloops++ == 1000000) {
			return 0;
		}
	}

	// measure the width of the pulse
	while (PIND & (1 << pin)) {
		width++;
		if (width == 1000000) {
			return 0;
		}
	}

	return width;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void display_digit(uint8_t digit) {
	// Segment patterns for digits 0-9
	const uint8_t segment_map[10] = {
		// GFEDCBA
		0b00111111, // 0
		0b00000110, // 1
		0b01011011, // 2
		0b01001111, // 3
		0b01100110, // 4
		0b01101101, // 5
		0b01111101, // 6
		0b00000111, // 7
		0b01111111, // 8
		0b01101111  // 9
	};

	// Set segments
	uint8_t segments = segment_map[digit];

	if (segments & (1 << 0)) PORTD &= ~(1 << SEG_A); else PORTD |= (1 << SEG_A);
	if (segments & (1 << 1)) PORTD &= ~(1 << SEG_B); else PORTD |= (1 << SEG_B);
	if (segments & (1 << 2)) PORTD &= ~(1 << SEG_C); else PORTD |= (1 << SEG_C);
	if (segments & (1 << 3)) PORTD &= ~(1 << SEG_D); else PORTD |= (1 << SEG_D);
	if (segments & (1 << 4)) PORTD &= ~(1 << SEG_E); else PORTD |= (1 << SEG_E);
	if (segments & (1 << 5)) PORTD &= ~(1 << SEG_F); else PORTD |= (1 << SEG_F);
	if (segments & (1 << 6)) PORTD &= ~(1 << SEG_G); else PORTD |= (1 << SEG_G);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

float calculate_distance_traveled(int encoderCount) {
	float revolutions = (float)encoderCount / COUNTS_PER_REVOLUTION;
	float circumference = 2 * M_PI * WHEEL_RADIUS;
	return revolutions * circumference;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PID_travel(uint8_t weightedSum) {
	// Calculate the position of the line
	float position = weightedSum;

	// Calculate the error
	error = position;

	// Calculate the integral
	integral += error;

	// Calculate the derivative
	derivative = error - lastError;

	// Calculate the correction
	correction = Kp * error + Ki * integral + Kd * derivative;

	// Update the last error
	lastError = error;

	// Use the correction value to adjust the motor speeds
	int leftMotorSpeed = currentSpeed - correction;
	int rightMotorSpeed = currentSpeed + correction;

	// Constrain motor speeds to allowable range
	leftMotorSpeed = constrain(leftMotorSpeed, 0, MAX_SPEED);
	rightMotorSpeed = constrain(rightMotorSpeed, 0, MAX_SPEED);

	// Set motor speeds
	setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

	_delay_ms(10); // Small delay for stability
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////MAIN/////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void) {

	// Initialize sensor pins as input
	DDRC = 0x00;

	// Initialize motor pins as output
	DDRB |= (1 << leftMotorPin) | (1 << rightMotorPin);

	// Set up PWM for motors
	TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM
	TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler 8

	// Initialize encoder pins as input
	// Set encoder pins as inputs
	DDRD &= ~((1 << LEFT_ENCODER_PIN_A) | (1 << LEFT_ENCODER_PIN_B) |
	(1 << RIGHT_ENCODER_PIN_A) | (1 << RIGHT_ENCODER_PIN_B));

	// Enable pull-up resistors on encoder pins
	PORTD |= (1 << LEFT_ENCODER_PIN_A) | (1 << LEFT_ENCODER_PIN_B) |
	(1 << RIGHT_ENCODER_PIN_A) | (1 << RIGHT_ENCODER_PIN_B);
	EICRA = (1 << ISC00) | (1 << ISC10); // Trigger on any logical change and falling edge
	EIMSK = (1 << INT0) | (1 << INT1);   // Enable external interrupts and interrupt 0 and 1

	// Initialize ultrasonic sensor pins as output/input
	DDRD |= (1 << trigPin1);
	DDRD &= ~(1 << echoPin1);

	// Set buttons as inputs
	DDRB &= ~((1 << BUTTON_PIN_INCREMENT) | (1 << BUTTON_PIN_DECREMENT) | (1 << BUTTON_PIN_START)) | (1 << TOUCH_PIN_SERVED);
	// Enable pull-up resistors
	PORTB |= (1 << BUTTON_PIN_INCREMENT) | (1 << BUTTON_PIN_DECREMENT) | (1 << BUTTON_PIN_START) | (1 << TOUCH_PIN_SERVED);

	// Set PD0-PD6 as output
	DDRD |= (1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C) | (1 << SEG_D) |
	(1 << SEG_E) | (1 << SEG_F) | (1 << SEG_G);

	// Enable global interrupts
	sei();

	// Initialize timers for counts
	init_timer();

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////LOOP/////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	while (1) {
		uint8_t sensorValues[NUM_SENSORS];
		uint8_t weightedSum = 0;
		uint8_t sum = 0;
		uint8_t allSensorsHigh = 1;

		if (debounce_button(BUTTON_PIN_INCREMENT)) {
			if (table_number < MAX_TABLE_COUNT) {
				table_number++;
				// Display the number on the 7-segment display
				display_digit(table_number);
			}
			// Wait until button is released
			while (debounce_button(BUTTON_PIN_INCREMENT));
		}

		if (debounce_button(BUTTON_PIN_DECREMENT)) {
			if (table_number > 0) {
				table_number--;
				// Display the number on the 7-segment display
				display_digit(table_number);
			}
			// Wait until button is released
			while (debounce_button(BUTTON_PIN_DECREMENT));
		}

		if (debounce_button(BUTTON_PIN_START)) {
			start_state = 1; // Start the robot
			mode = 2; // Set mode to accelerating
			// Wait until button is released
			while (debounce_button(BUTTON_PIN_START));
		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////START OPERATIONS/////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		if(start_state == 1){

			if (serving_state == 2){

				while(1){
					if (debounce_button(TOUCH_PIN_SERVED)) {
						serving_state = 3; // Start the robot
						mode = 2; // Set mode to accelerating
						// Wait until button is released
						while (debounce_button(TOUCH_PIN_SERVED));
					}
				}
				// Move the robot back to the starting point
				turnBend(0);    // Turn 90 degrees back

				} if (serving_state == 3) {

				// Read the sensor values and calculate the weighted sum and the sum
				for (uint8_t i = 0; i < NUM_SENSORS; i++) {
					sensorValues[i] = readAnalog(sensorPins[i]);
					uint8_t temp_sensor_digital_value = 0;
					if (sensorValues[i] > 500) {
						temp_sensor_digital_value = 1;
					}
					sum += temp_sensor_digital_value;
					weightedSum += temp_sensor_digital_value * weights[i];
					if (sensorValues[i] < 500) {  // Assuming a threshold of 500 for high signal
						allSensorsHigh = 0;
					}
				}

				// Increment stop count if all sensors are high
				if (allSensorsHigh == 1) {
					if (millis() - previous_table_count_update_time > MIN_TIME_INTERVAL) {
						table_count--;
						previous_table_count_update_time = millis();
					}
					if (stopCount >= stopCountThreshold) {
						mode = 0; // Set mode to decelerating
					}
				}

				if (table_count == 0) {
					turnBend(180);    // Turn 90 degrees back
					serving_state = 1; // Set the serving state to none
					start_state = 0; // Set the start to false
					table_number = 0; // Reset the table number
				}


				PID_travel(weightedSum);
				
				} else {

				// Start the robot with the acceleration from the speed profile
				if (mode == 2){
					// Control motor speed based on acceleration and speed
					uint8_t left_motor_speed = 0;
					uint8_t right_motor_speed = 0;

					// Accelerate the motors until the maximum speed is reached
					uint8_t speedReached = 0;

					if (leftSpeedMPS < MAX_SPEED) {
						left_motor_speed += (leftAccelerationMPS2 < MAX_ACCELERATION) ? 1 : -1;
						} else {
						left_motor_speed = 0;
						speedReached = 1;
					}

					if (rightSpeedMPS < MAX_SPEED) {
						right_motor_speed += (rightAccelerationMPS2 < MAX_ACCELERATION) ? 1 : -1;
						} else {
						right_motor_speed = 0;
						speedReached = 1;
					}

					// Set motor speeds
					setMotorSpeed(left_motor_speed, right_motor_speed);
					if (speedReached == 1) {
						mode = 1; // Set mode to normal operation
						break;
					}
					speedReached = 0;
				}

				// Start the robot with the deceleration from the speed profile
				if (mode == 0) {
					// Control motor speed based on deceleration and speed
					uint8_t left_motor_speed = 0;
					uint8_t right_motor_speed = 0;

					// Decelerate the motors until they come to a stop
					uint8_t speedReached = 0;

					if (leftSpeedMPS > 0) {
						left_motor_speed -= (leftAccelerationMPS2 > -MAX_DECELERATION) ? 1 : 0;
						} else {
						left_motor_speed = 0;
						speedReached = 1;
					}

					if (rightSpeedMPS > 0) {
						right_motor_speed -= (rightAccelerationMPS2 > -MAX_DECELERATION) ? 1 : 0;
						} else {
						right_motor_speed = 0;
						speedReached = 1;
					}

					// Set motor speeds
					setMotorSpeed(left_motor_speed, right_motor_speed);
					if (speedReached == 1) {
						mode = 1; // Set mode to normal operation
						break;
					}
					speedReached = 0;
				}

				// Read the sensor values and calculate the weighted sum and the sum
				for (uint8_t i = 0; i < NUM_SENSORS; i++) {
					sensorValues[i] = readAnalog(sensorPins[i]);
					uint8_t temp_sensor_digital_value = 0;
					if (sensorValues[i] > 500) {
						temp_sensor_digital_value = 1;
					}
					sum += temp_sensor_digital_value;
					weightedSum += temp_sensor_digital_value * weights[i];
					if (sensorValues[i] < 500) {  // Assuming a threshold of 500 for high signal
						allSensorsHigh = 0;
					}
				}

				// Check for obstacles
				if (isObstacleDetected() == 1) {
					// Start the robot with the deceleration from the speed profile
					while (1) {
						// Control motor speed based on deceleration and speed
						uint8_t left_motor_speed = 0;
						uint8_t right_motor_speed = 0;

						// Decelerate the motors until they come to a stop
						uint8_t speedReached = 0;

						if (leftSpeedMPS > 0) {
							left_motor_speed -= (leftAccelerationMPS2 > -MAX_DECELERATION) ? 1 : 0;
							} else {
							left_motor_speed = 0;
							speedReached = 1;
						}

						if (rightSpeedMPS > 0) {
							right_motor_speed -= (rightAccelerationMPS2 > -MAX_DECELERATION) ? 1 : 0;
							} else {
							right_motor_speed = 0;
							speedReached = 1;
						}

						// Set motor speeds
						setMotorSpeed(left_motor_speed, right_motor_speed);
						if (speedReached == 1) {
							mode = 1; // Set mode to normal operation
							break;
						}
						speedReached = 0;
					}
					while (isObstacleDetected() == 1) {
						_delay_ms(100); // Wait until obstacles pass
					}
					mode = 2; // Set mode to accelerating
				}

				// Increment stop count if all sensors are high
				if (allSensorsHigh == 1) {
					if (millis() - previous_table_count_update_time > MIN_TIME_INTERVAL) {
						table_count++;
						previous_table_count_update_time = millis();
					}
					if (stopCount >= stopCountThreshold) {
						mode = 0; // Set mode to decelerating
					}
				}

				if (table_number == table_count){
					while (1) {
						// Control motor speed based on deceleration and speed
						uint8_t left_motor_speed = 0;
						uint8_t right_motor_speed = 0;

						// Decelerate the motors until they come to a stop
						uint8_t speedReached = 0;

						if (leftSpeedMPS > 0) {
							left_motor_speed -= (leftAccelerationMPS2 > -MAX_DECELERATION) ? 1 : 0;
							} else {
							left_motor_speed = 0;
							speedReached = 1;
						}

						if (rightSpeedMPS > 0) {
							right_motor_speed -= (rightAccelerationMPS2 > -MAX_DECELERATION) ? 1 : 0;
							} else {
							right_motor_speed = 0;
							speedReached = 1;
						}

						// Set motor speeds
						setMotorSpeed(left_motor_speed, right_motor_speed);
						if (speedReached == 1) {
							mode = 1; // Set mode to normal operation
							break;
						}
						speedReached = 0; //break the deceleration loop when speed has gotten to zero
					}
					turnBend(90);    // Turn 90 degrees
					serving_state = 1; // Set the serving state to true
				}

				PID_travel(weightedSum);
			}
			} else {
			//set 7 segment display to 0
			display_digit(0);
		}
	}

	return 0;
}
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Part ranges (from calibration)
 */
#define MIN_AL	(0)
#define MAX_AL	(200)

#define MIN_ST	(201)
#define MAX_ST	(799)

#define MIN_WT	(800)
#define MAX_WT	(969)

#define MIN_BK	(970)

/**
 * Conveyor movement
 */
#define CONVEYOR_BRAKE 		(0x00)
#define CONVEYOR_FORWARD	(1 << 3)

/**
 * Tray movement
 */
#define STEP_0 	0b00000011 //((1 << 0) | (1 << 1))
#define STEP_1	0b00011000 //((1 << 3) | (1 << 4))
#define STEP_2	0b00000101 //((1 << 0) | (1 << 2))
#define STEP_3	0b00101000 //((1 << 3) | (1 << 5))

/**
 * Tray acceleration
 */
#define LEN_ACCEL_PROFILE	25
const int accel_min_delay = 6;
const int accel_profile[LEN_ACCEL_PROFILE] = {20, 19, 18, 17, 15, 15, 13, 13, 11, 11, 9, 9, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6};
volatile int ind_profile = 0;

/**
 * part_t defines a new type used to identify parts.
 */
typedef enum
{
	BLACK		= 1,	// 0001 on the lower 4 bits of the LEDs
	ALUMINUM,			// 0010
	WHITE,				// 0011
	STEEL,				// 0100
	UNKNOWN		= 10,	// 1010
	NO_PART		= 0xF	// 1111
} part_t;

/**
 * To make part_t circular, implement custom get next and previous parts.
 * This allows STEEL to be next to BLACK, wrapping around.
 */
part_t get_next_CW_part(part_t part);
part_t get_next_CCW_part(part_t part);

/**
 * parts_queue stores each part that is sorted by the system.
 * The indices point to the 'head' and 'tail' of the list, to remember which part is the oldest,
 * and which is the newest.
 *
 * Example queue:
 * [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 ]
 *   ^ ind_oldest
 *   ^ ind_newest
 * 
 * Adding a part (enqueue):
 * [ X 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ... 0 ]
 *   ^ ind_oldest
 *     ^ ind_newest
 *
 * if ind_oldest == ind_newest, the list is empty.
 */
#define LEN_PARTS_QUEUE 1000
part_t parts_queue[LEN_PARTS_QUEUE];
int ind_oldest = 0;
int ind_newest = 0;

/**
 * Look at the state of the list. Empty or length
 */
bool queue_is_empty(void);
int queue_length(void);

/**
 * Prints a 10 bit number onto the LEDs (Port C and the 2 onboard LEDs, read from top to bottom)
 */
void display(uint16_t num);

/**
 * delay uses timer/counter 0 to delay the given number of milliseconds.
 */
void delay(int msec_count);

/**
 * Sets the speed of the conveyor. accepts values from 0 to 0xFF.
 */
void conveyor_speed_set(uint8_t speed);

struct
{
	bool direction;		//!< True is clockwise, false is CCW
	part_t current;		//!< This, along with the offset, shows exactly where the tray is at any given step.
	int current_offset;	//!< Maxes out at 50. Shows the number of steps away from the center of the current part.
	part_t target;		//!< Stores the target for the stepper. When current == target and offset == 0, tray is at the target
} tray_info;

/**
 * tray initialization sequence
 */
void tray_home(void);

/**
 * Sets the target, moves the tray to satisfy that target
 */
void tray_go_to_target();

/**
 * Takes one step. Remembers which step the tray is currently on.
 */
void tray_step1step(bool clockwise);

/**
 * Initialization
 */
void system_init();
void adc_init();
void PWM_start();

/**
 * For testing the stepper motor acceleration profile.
 */
void test_stepper(void);

/**
 * These routines service button presses
 */
void pause_routine(void);
void ramp_down_routine(void);

/**
 * Struct to keep track of what is on the tray
 */
volatile struct
{
	int black;
	int aluminum;
	int white;
	int steel;
} num_parts_sorted;

/**
 * Global flags
 */
volatile bool show_calibration_flag = false;
volatile bool pause_flag = false;
volatile bool ramp_down_flag = false;
volatile bool kill_system = false;

/**
 * Always runs calibration in the background.
 */
volatile uint16_t calib_max = 0x0000;
volatile uint16_t calib_min = 0xFFFF;

int main(void)
{
	system_init();

//	test_stepper();
	
	conveyor_speed_set(0x44);
	PORTB = CONVEYOR_FORWARD;

	while (true)
	{
		while (pause_flag)
		{
			pause_routine();
		}

		if (ramp_down_flag)
		{
			ramp_down_routine();
		}
	}
	return 0;
}

void test_stepper(void)
{
	int i = 0;
	int turns = 0;
	while (true)
	{
		delay(20);
		if (++turns > 3)
			turns = 0;
		while (ind_profile < LEN_ACCEL_PROFILE)
		{
			tray_step1step(tray_info.direction);
			delay( accel_profile[ind_profile++] );
		}
		for (i = 0; i < (50 * turns); i++)
		{
			tray_step1step(tray_info.direction);
			delay( accel_min_delay );	
		}
		while (ind_profile > 0)
		{
			tray_step1step(tray_info.direction);
			delay( accel_profile[--ind_profile] );
		}
		tray_info.direction = !tray_info.direction;
	}
}

void pause_routine(void)
{
	uint8_t port_state = PORTB;
	PORTB = CONVEYOR_BRAKE;

	// display contents of tray
	display((1 << 4) | num_parts_sorted.black);
	delay(4000);

	display((1 << 5) | num_parts_sorted.steel);
	delay(4000);

	display((1 << 6) | num_parts_sorted.white);
	delay(4000);

	display((1 << 7) | num_parts_sorted.aluminum);
	delay(4000);

	// show what's on the belt
	display(0xF0 | queue_length());
	delay(4000);

	display(0x0000);
	PORTB = port_state;
}

void ramp_down_routine(void)
{
	delay(4500);
	while(!queue_is_empty());
	
	// Delay 100ms to ensure the last piece falls.
	delay(100);
	
	cli();
	PORTB = CONVEYOR_BRAKE;
	
	while (true)
	{
		pause_routine();
	}
}

void system_init()
{
	// Interrupts:
	// 	INT0: kill switch
	// 	INT1: pause btn
	// 	INT2: end-of-belt
	// 	INT3: reflectivity optical sensor
	
	// PORTS:
	// 	A: Tray stepper
	// 	B: DC Motor (and PWM)
	// 	C: LEDs
	// 	D: Interrupts
	// 	F: ADC input

	cli();	// Disable interrupts

	// Initialize an empty parts queue
	int i;
	for (i = 0; i < LEN_PARTS_QUEUE; i++)
	{
		parts_queue[i] = NO_PART;
	}

	DDRA = 0xFF; 	// Port A is the tray stepper
	DDRB = 0xFF;	// Port B is the conveyor
	DDRC = 0xFF;	// Port C is output for LEDs
	DDRD = 0xF0;	// Port D is interrupts and 2 diodes
	DDRE = 0x00;	// Port E is the HE sensor

	PORTB = CONVEYOR_BRAKE;

	PWM_start();

	adc_init();

	// INT0 is "ramp down"
	EIMSK |= _BV(INT0); // Pause button
	EICRA |= _BV(ISC00) | _BV(ISC01); // rising edge

	// INT1 is the pause button
	EIMSK |= _BV(INT1); // Pause button
	EICRA |= _BV(ISC10) | _BV(ISC11); // rising edge

	// INT2 is the calibration button
	EIMSK |= _BV(INT2);
	EICRA |= _BV(ISC20) | _BV(ISC21); // rising edge

	// Initialize EX (exit sensor)
	EIMSK |= _BV(INT4);	// INT2 is the exit sensor
	EICRB |= _BV(ISC41);// falling edge interrupt

	// Initialize OR (optical sensor next to reflective sensor
	EIMSK |= _BV(INT3); // INT3 is the reflectivity optical sensor
	EICRA |= _BV(ISC30); // Any edge interrupts

	PORTC = 0;

	// Home the tray on black
	tray_info.direction = true;
	tray_home();
	
	sei();	// Enable interrupts
}

/**
 * Returns distance to the current target
 */
int tray_distance_to_target()
{
	int dist_to_target = 0;
	part_t current = tray_info.current;
	while (current != tray_info.target)
	{
		dist_to_target += 50;
		if (tray_info.direction)
		{
			current = get_next_CW_part(current);
		}
		else
		{
			current = get_next_CCW_part(current);
		}
	}

	if (dist_to_target >= 150)
	{
		// Change direction if we have to go 270.
		tray_info.direction = !tray_info.direction;
		dist_to_target = 50;
	}
	return dist_to_target;
}

part_t get_next_CW_part(part_t part)
{
	part++;
	if (part > STEEL)
	{
		part = BLACK;
	}

	return part;
}

part_t get_next_CCW_part(part_t part)
{
	part--;
	if (part < BLACK)
	{
		part = STEEL;
	}

	return part;
}

void tray_go_to_target()
{
	if (tray_info.current == tray_info.target)
	{
		// We are already here
		return;
	}
	else if ((tray_info.target == NO_PART) || (tray_info.target == UNKNOWN))
	{
		// Nothing to be done for unknown or empty part lists.
		return;
	}

	int dist_to_target = tray_distance_to_target();
	
	int i = 0;
	int count = 0;
	int top_speed_steps = dist_to_target - (LEN_ACCEL_PROFILE * 2);
	for(i = 0; i < dist_to_target; i++)
	{
		tray_step1step(tray_info.direction);

		// Delay: walk up the profile, then hold, then back down the other side.
		if (count < LEN_ACCEL_PROFILE)
		{
			delay(accel_profile[ind_profile++]);
			count++;
		}
		else if (top_speed_steps)
		{
			delay(accel_min_delay);
			top_speed_steps--;
		}
		else
		{
			delay(accel_profile[ind_profile--]);
			if ((PORTB == CONVEYOR_BRAKE) && (ind_profile == 11))
			{
				PORTB = CONVEYOR_FORWARD;
			}
		}
	}
}

/** Only run at the very beginning of a sorting run 
 * Moves the tray until the HE sensor is low, therefore signaling a home 
 */
void tray_home(void)
{
	int i;
	for (i = 75; i != 0; i--)
	{
		tray_step1step(true);
		delay(20);
	}

	while(PINE & 0b01000000)
	{
		tray_step1step(true);
		delay(20);
	}
	tray_info.current = BLACK;
}

void tray_step1step(bool clockwise)
{
	static int step = 0;
	
	if(clockwise)
	{ 
		// CW, positively increments step
		step++;
		
		tray_info.current_offset++;
		if (tray_info.current_offset >= 50)
		{
			tray_info.current_offset = 0;
			tray_info.current = get_next_CW_part(tray_info.current);
		}
	}
	else
	{ 
		// CCW, negatively increments step
		step--;
		
		tray_info.current_offset--;
		if (tray_info.current_offset < 0)
		{
			tray_info.current_offset = 49;
			tray_info.current = get_next_CCW_part(tray_info.current);
		}
	}

	// reset if either direction reaches step boundaries
	if (step > 3)
	{ 
		step = 0;
	}
	else if (step < 0)
	{
		step = 3;
	}

	switch (step)
	{
		case 0:
		PORTA = STEP_0 | STEP_1;
		break;

		case 1:
		PORTA = STEP_1 | STEP_2;
		break;
			
		case 2:
		PORTA = STEP_2 | STEP_3;
		break;
			
		case 3:
		PORTA = STEP_3 | STEP_0;
		break;
	}
}

// Returns the length of the queue
int queue_length(void)
{
	return (ind_newest - ind_oldest);
}

bool queue_is_empty(void)
{
	return (ind_newest == ind_oldest);
}

void display(uint16_t num)
{
	PORTC = (uint8_t)num;
	PORTD &= 0x0F;
	if (num & 0x100)
	{
		PORTD = (1 << 4);
	}
	if (num & 0x200)
	{
		PORTD |= (1 << 7);
	}
}

void delay(int msec_count)
{
	TCCR1B |= _BV(CS10); // Select clock source: no prescaling

	TCCR1B |= _BV(WGM12); // Set WGM12, putting the WGM into CTC mode
						  // Count up until TCNT1 matches OCR1A

	OCR1A = 0x3e8; // Delay 1000 cycles (1000 us = 1ms)
	TCNT1 = 0x0000; // Start the timer at 0
	TIFR1 |= _BV(OCF1A); // Clear OCF1A (interrupt flag)

	int i = 0;
	while (i < msec_count)
	{
		if ((TIFR1 & 0x02) == 0x02) // Wait until timer expires
		{
			TIFR1 |= _BV(OCF1A); // Clear interrupt flag
			i++;
		}
	}
}

void adc_init()
{
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(REFS0) | _BV(MUX0); // Reference voltage Vcc, using channel 1 (port F1)
}

/**
 * Sets the conveyor speed by altering PWM duty cycle
 */
void conveyor_speed_set(uint8_t speed)
{
	OCR0A = speed;
}

/**
 * Initializes PWM output on pin B 7
 */
void PWM_start()
{
	TCCR0A |= _BV(WGM01);
	TCCR0A |= _BV(WGM00); // Fast PWM, TOP is MAX

	TCCR0A |= _BV(COM0A1); // Clear on compare match, set at 0xFF

	TCCR0B |= _BV(CS01); // CS = 0b100, prescaling is 8 (488Hz)

	OCR0A = 0x80; // 50% duty cycle
}

/**
 * When adc_convert is set (by the OR sensor), adc_conversions take place continuously.
 */
volatile bool adc_convert = false;

/**
 * Stores the running minimum from ADC on one part.
 */
volatile uint16_t adc_result;

ISR(ADC_vect)
{
	if (ADC < adc_result)
	{
		adc_result = ADC;
	}

	if (adc_convert)
	{
		ADCSRA |= _BV(ADSC);	// Start conversion
	}
	else
	{
		// Update calibration values
		if (adc_result < calib_min)
		{
			calib_min = adc_result;
		}
		if (adc_result > calib_max)
		{
			calib_max = adc_result;
		}

		// Classify part
		if (adc_result >= MIN_BK)
		{
			parts_queue[ind_newest++] = BLACK;
		}
		else if ((adc_result >= MIN_WT) && (adc_result <= MAX_WT))
		{
			parts_queue[ind_newest++] = WHITE;
		}
		else if ((adc_result >= MIN_ST) && (adc_result <= MAX_ST))
		{
			parts_queue[ind_newest++] = STEEL;
		}
		else if (adc_result <= MAX_AL)
		{
			parts_queue[ind_newest++] = ALUMINUM;
		}
		else
		{
			display(adc_result);
			PORTB = CONVEYOR_BRAKE;
			while (true);
		}
	}
}

/**
 * INT4 is the exit sensor
 */
ISR(INT4_vect)
{
	PORTB = CONVEYOR_BRAKE;
	tray_info.target = parts_queue[ind_oldest];

	if (tray_info.current != tray_info.target)
	{
		tray_go_to_target();
	}
	
	// Make sure queue isn't empty
	if (ind_newest != ind_oldest)
	{
		// Keep track of number of parts sorted
		switch (parts_queue[ind_oldest])
		{
		case BLACK:
			num_parts_sorted.black++;
			break;
		case ALUMINUM:
			num_parts_sorted.aluminum++;
			break;
		case WHITE:
			num_parts_sorted.white++;
			break;
		case STEEL:
			num_parts_sorted.steel++;
			break;
		default:
			break;
		}

		// 'Pop' the oldest part off the queue.
		ind_oldest++;
	}

	PORTB = CONVEYOR_FORWARD;
}

/**
 * INT3 is the OR sensor
 */
ISR(INT3_vect)
{
	adc_convert = !adc_convert;
	if (adc_convert)
	{
		// reset min
		adc_result = 0xFFFF;
		display(0x3FF);
		ADCSRA |= _BV(ADSC); // Start ADC conversion
	}
	else
	{
		display(0);
	}	
}

/**
 * Pause button
 */
ISR(INT1_vect)
{
	delay(25);
	if (PIND & 0x2)
	{
		pause_flag = !pause_flag;
	}
}

/**
 * Calibration button - when pressed, shut down the system and display calibration values.
 */
ISR(INT2_vect)
{
	cli();
	PORTB = CONVEYOR_BRAKE;
	int i;
	PORTC = 0xFF;
	for (i = 0; i < 10; i++)
	{
		delay(200);
		PORTC = ~PORTC;
	}
	while (true)
	{
		display(calib_min);
		delay(7000);
		display(calib_max);
		delay(7000);
	}
}


/**
 * Ramp down button
 */
ISR(INT0_vect)
{
	ramp_down_flag = true;
}

ISR(BADISR_vect)
{
	while (true)
	{
		display(0x3FF);
		delay(250);
		
		display(0x000);
		delay(250);
	}
}
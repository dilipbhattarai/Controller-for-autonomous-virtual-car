#include <avr/io.h>
#include <avr/interrupt.h>

#define BIT(a) (1 << (a))

void Timer_setup();
void task1();

// Speed reference value (rad/s)
float speed_ref = 550; // change this value according to requirements

float read_ADC_voltage(uint8_t channel, int n);
void setup_ADC(uint8_t channel);

//controller functions
float speedController(float wb);
float traction_controller(float wb, float wf);
float braking_controller(float wb, float wf);
float lowPassFilter(float input, float alpha);

volatile uint16_t sensorValue = 0;
volatile long sum = 0;
volatile int count = 0;
int samples = 0;
volatile float t0, pw1, pw2, k;

void setup() {
	
	char ch;
	
	Serial.begin(2000000);
 	
  delay(3000);
  
  cli(); // disable interrupts

  DDRB |= BIT(0); // Configure digital pin 8 (PB0) as output
  DDRD |= BIT(7); // Configure digital pin 7 (PD7) as output

  Timer_setup();
  
  k = 64.0/16e6; // calculate here to save time that is wasted by division in loop
  
  sei(); // enable interrupts
  
  t0 = micros()*1.0e-6; // initial time (s)
  
  while (1)  task1();
 	
	delay(1000);
	exit(0); // not using loop()	
 
}

void task1()
{
	int n;
	float y1 = -1, y2 = -1, y3 = -1;
	float wb, wf;
	float u1, u2;
	//int pw1, pw2;
	float w1, w2;	
	float t;

	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
	const float wmax = 810.0; // maximum back wheel speed (rad/s)	
	const float V_bat = 12.0; // lipo battery voltage
	const float V_bat_inv = 1/V_bat;	

	// read clock time (s)
	t = micros()*1.0e-6 - t0;

	// read car outputs ///////////////

	// note that y1 (drive motor), y2 (right front wheel), and 
	// y3 (left front wheel) are the outputs for the car system

	// *** I recommend using pins A1, A3, and A5 for analog inputs ***
	
	n = 200; // number of ADC samples in averaging filter
	
    y1 = read_ADC_voltage(1, n);
    y2 = read_ADC_voltage(3, n);
    y3 = read_ADC_voltage(5, n);
	
	// use the following scale for the output y1
	// w = 0 	-> y1 = 2.5 V
	// w = wmax -> y1 = 5.0 V
	// y1 = 2.5 + w * wmax_inv * 2.5 ->
	
	// back wheel angular velocity (rad/s)
	wb = (y1 - 2.5) * 0.4 * wmax;

	// front wheel angular velocity (rad/s)
	wf = (y2 - 2.5) * 0.4 * wmax;

	// The test for various controllers are carried out based on time
	
	if (t <= 10) // acceleration/ traction controller test
  {
    u1 = t* 2 ;
    u1 = u1 + traction_controller(wb, wf);
  }
  else if (t > 10 && t <= 40) // constant cruise control/ speed controller test
  {
    u1 = 12.0;
    u1 = u1 + speedController(wb);
  }
  else if (t > 40) // braking controller test
  {
    u1 = 12.0 - t* 0.13; // Applying a retardation to test braking controller 
    u1 = u1 + braking_controller(wb, wf);
  }
	
	// note the maximum input u1 is V_bat (12V)
	// anything more than 12V or less than -12V
	// will have no effect on the simulator
	// use the following if statement to prevent
	// asking for inputs out of range
	if( u1 > V_bat )  u1 = V_bat;
	if( u1 < -V_bat ) u1 = -V_bat;	
	
	u2 = 0.0;

	// convert inputs to actuator values pw1, pw2

	// convert motor voltage to pulse width
	// u1 = (w1 - PW_0) * PW_R * V_bat ->
	w1 = u1 * V_bat_inv * (PW_MAX - PW_0) + PW_0; 
	
	// saturate input if out of range
	if(w1 > PW_MAX) w1 = PW_MAX;
	if(w1 < PW_MIN) w1 = PW_MIN;	
	
	pw1 = w1;
  
//	if(w2 > PW_MAX) w2 = PW_MAX;
//	if(w2 < PW_MIN) w2 = PW_MIN;

	// set pw2 for testing purposes
	
	  pw2 = 1750;
 // remove this in the final compilation
	
	// print out for testing purposes
	
	Serial.print(t);
	Serial.print(",");
	
	Serial.print(y1);
	Serial.print(",");
	
	Serial.print(y2);
	Serial.print(",");

	Serial.print(y3);
	Serial.print(",");	
	
	Serial.print(u1);
	Serial.print(",");	
	
	Serial.print(wb);
	Serial.print(",");		
	
	Serial.print(wf);
	Serial.print("\n");	
	
	delay(30);
}

void loop() {
  // do nothing
}

void Timer_setup() {
  
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1B |= BIT(WGM12);
  TCCR1B |= BIT(CS11) | BIT(CS10); // Set prescaler to 64 and turn on timer

  OCR1A = 5000; // Set timer compare register
  TCNT1 = 0; 

  TIFR1 |= BIT(OCF1A); 
  TIMSK1 = BIT(OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  
	//Serial.print("\nInterrupt got called ");
	
	float t_next1 = 0.0, t_next2 = 0.0, pulse_time1 , pulse_time2  ;
	pulse_time1 = pw1;	
	pulse_time2 = pw2 ;
	static float t_current = 0.0, t_start = 0.0;
	
	//Serial.print("\nReceived pw1 is ");
	//Serial.print(pulse_time1);
	//Serial.print("\nReceived pw2 is ");
	//Serial.print(pulse_time2);
	
	t_current = TCNT1 * k  * 1.0e6;
	t_next1 = t_current + pulse_time1 ;
	
  
	PORTD |= BIT(7); // turn pin 7 on  
		
	while (t_current < t_next1)
	{
	t_current = TCNT1 * k *1.0e6; // update time		
	}
	
	PORTD &= ~BIT(7); //  turn pin 7 off 
	
	
	float buffer_time = t_current + 300.0;
	
	while (t_current < buffer_time)	 // Allow buffer time so that two inturrupt might not inturrupt each other
	{
	t_current = TCNT1 * k *1.0e6; // update time
	}
	
	t_start = TCNT1 * k *1.0e6;
	t_next2 = t_start + pulse_time2 ;
	
	PORTB |= BIT(0); // turn pin 8 on 
	
	while (t_start < t_next2)
	{
	t_start = TCNT1 * k * 1.0e6; // update time	
	}
	
	PORTB &= ~BIT(0); // turn pin 8 off 
	
}

float read_ADC_voltage(int channel, int n)
{
	int i;
	float input, voltage;
	unsigned long int sum;
	const float ADC_to_V = 1.0/1023.0*5;
	
	sum = 0;
	for(i=0;i<n;i++) {
		sum += analogRead(channel);
	}
	
	input = (float)sum / n; // average analog input
	
	// note that the simple division of float below takes around 
	// 40 us compared to around 4 us for equivalent multiplication
	// -> avoid using float division if possible
	// -> use multiplication instead of division when possible
	// -> avoid putting float division in loops
//	voltage = input / 1023.0 * 5;
	voltage = input * ADC_to_V; // much faster than expression above
	
	return voltage;
}
void setup_ADC(uint8_t channel)
{
    // Set ADMUX to use internal 5V reference and set MUX bits
    ADMUX = BIT(REFS0) | (channel & 0x07);

    // Set ADCSRA to enable ADC, set pre-scaler to 128, and enable ADC interrupt
    ADCSRA = BIT(ADEN) | BIT(ADPS2) | BIT(ADPS1) | BIT(ADPS0) | BIT(ADIE);
}
float read_ADC_voltage(uint8_t channel, int n)
{
    sum = 0;
    count = 0;
    samples = n;

    setup_ADC(channel);

    // Enable global interrupts
    sei();

    // Start the first conversion
    ADCSRA |= BIT(ADSC);

    while (count < n)
    {
        // Do nothing, wait for the interrupt routine to finish collecting samples
    }

    // Disable global interrupts
    cli();

    return (float)sum / n * (5.0 / 1023.0);
}

ISR(ADC_vect)
{
    // Read the ADC value
    sensorValue = ADC;

    sum += sensorValue;
    count++;

    if (count < samples)
    {
        // Start the next conversion
        ADCSRA |= BIT(ADSC);
    }
}

float speedController(float wb) {
  static float error_previous = 0.0;
  static float error_integral = 0.0;
  static unsigned long time_previous = 0;

  // Speed control gain
  const float kp_speed = 0.55; // Proportional gain for speed control
  const float ki_speed = 0.00;
  const float kd_speed = 0.0014;

  // Calculate the speed error
  float speed_error = speed_ref - wb;

  unsigned long time_current = micros();
  float dt = (time_current - time_previous) * 1.0e-6;

  float error_derivative = (speed_error - error_previous) / dt;

  float integral_factor = (speed_error * dt);

  error_integral += integral_factor;

  error_previous = speed_error;

  // Calculate the speed control signal
  float control_signal = kp_speed * speed_error + ki_speed * error_integral + kd_speed * error_derivative;

  // Applying the low-pass filter to the control signal
  float alpha = 0.02; // Adjusting the value of alpha to find the best trade-off between smoothing and balancing the system dynamics
  //on trying various values the value of range 0.02 was found out to be the best suited for this sytem
   
  float filtered_control_signal = lowPassFilter(control_signal, alpha);

  time_previous = time_current;
  
  return filtered_control_signal;
 
  return control_signal;
}

float traction_controller(float wb, float wf) {
  static float previous_error = 0.0;
  static float integral = 0.0;
  static unsigned long previous_time = 0;

  const float Kp = 0.05; // Proportional gain
  const float Kd = 0.000; // Derivative gain
  const float Ki = 0.00; // Integral gain

  float slip_ratio = (wf - wb) /fabs(wf) ;
  float error = 0.2 - slip_ratio; // where 0.2 is the desired slip ratio

  unsigned long current_time = micros();
  float dt = (current_time - previous_time) * 1.0e-6;

  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  float control_signal = Kp * error + Kd * derivative + Ki * integral;

  previous_error = error;
  previous_time = current_time;

  return control_signal;
}

float braking_controller(float wb, float wf) {
  static float previous_error_brake = 0.0;
  static float integral_brake = 0.0;
  static unsigned long previous_time_brake = 0;

  const float Kp_brake = 0.05; // Proportional gain
  const float Kd_brake = 0.000; // Derivative gain
  const float Ki_brake = 0.000; // Integral gain

  float slip_ratio = (wb - wf) / fabs(wf);
  float error_brake = -0.2 - slip_ratio; // where -0.2 is the desired slip ratio

  unsigned long current_time_brake = micros();
  float dt = (current_time_brake - previous_time_brake) * 1.0e-6;

  integral_brake += error_brake * dt;
  float derivative_brake = (error_brake - previous_time_brake) / dt;
  float control_signal_brake = Kp_brake * error_brake + Kd_brake * derivative_brake + Ki_brake * integral_brake;

  previous_error_brake = error_brake;
  previous_time_brake  = current_time_brake ;

  return control_signal_brake;
}
float lowPassFilter(float input, float alpha) {
  static float output = 0.0;
  output = alpha * input + (1 - alpha) * output;
  return output;
}

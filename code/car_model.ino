#include <Servo.h>
#include <AFMotor.h>
#include <SD.h>
#include <SPI.h>
#include <TMRpcm.h>

enum arduino_pins {
  inner_rgb_light = 2, motor_front_speed, motor_out_1,
  motor_out_2, motor_out_3, motor_out_4, 
  motor_back_speed, speaker_pin, sd_reader_cs, 
  sd_reader_mosi, sd_reader_miso, sd_reader_sck,
  servo_motor_pin, front_light,
  obstacle_sensor_echo = 17, obstacle_sensor_trig, back_light, 
  servo_motor_center = 90, minimum_obstacle_detect = 39
  };

static inline void car_stop ();
static inline void car_move_backward ();
static inline void car_move_forward ();
static inline void car_move_right ();
static inline void car_move_left ();
static inline void turn_front_light_on ();
static inline void turn_back_light_on ();
static inline void turn_front_light_off ();
static inline void turn_back_light_off ();
static inline void turn_horn_on ();
static inline void turn_horn_off ();
static inline void turn_rgb_light_on ();
static inline void turn_rgb_light_off ();
static void atomatic_movement ();
static void manual_movement ();
double obstacle_sensor_see_right();
double obstacle_sensor_see_left();
static inline void obstacle_detect_forward ();
static inline void obstacle_detected_right ();
static inline void obstacle_detected_left ();
double ultrasonic();
void print_directory (File dir, int num_tabs);

char bluetooth_signal = '\0'; // D means stop all.
unsigned int motor_speed = 250;
unsigned int speaker_volume = 5;

Servo servo_motor;
TMRpcm speaker;

bool is_manual = true;
bool sd_found = false;

void setup () 
{
    pinMode (front_light, OUTPUT);
    pinMode (back_light, OUTPUT);
    pinMode (inner_rgb_light, OUTPUT);

    pinMode (sd_reader_cs, OUTPUT);
    pinMode (sd_reader_mosi, INPUT);
    pinMode (sd_reader_miso, INPUT);
    pinMode (sd_reader_sck, INPUT);

    pinMode (obstacle_sensor_trig, OUTPUT);
    pinMode (obstacle_sensor_echo, INPUT);

    pinMode (motor_out_1, OUTPUT);
    pinMode (motor_out_2, OUTPUT);
    pinMode (motor_out_3, OUTPUT);
    pinMode (motor_out_4, OUTPUT);

    pinMode (motor_front_speed, OUTPUT);
    pinMode (motor_back_speed, OUTPUT);

    servo_motor.attach (servo_motor_pin);

    speaker.speakerPin=speaker_pin;
    Serial.begin (9600); // Default communication rate of the Bluetooth module
    speaker.setVolume(speaker_volume);

    sd_found = SD.begin(sd_reader_cs);
    if(sd_found == true)
    {
      Serial.println ("SD Card found");
    } else Serial.println ("Failed to find SD Card");

}

static inline void car_stop ()
{
    analogWrite (motor_front_speed, 0);
    analogWrite (motor_back_speed, 0);
}

static inline void car_move_backward ()
{
    digitalWrite (motor_out_1, HIGH);
    delay (1);
    digitalWrite (motor_out_2, LOW);
    delay (1);
    digitalWrite (motor_out_3, HIGH);
    delay (1);
    digitalWrite (motor_out_4, LOW);

    analogWrite (motor_front_speed, motor_speed);
    analogWrite (motor_back_speed, motor_speed);
}

static inline void car_move_forward ()
{
    digitalWrite (motor_out_1, LOW);
    delay (1);
    digitalWrite (motor_out_2, HIGH);
    delay (1);
    digitalWrite (motor_out_3, LOW);
    delay (1);
    digitalWrite (motor_out_4, HIGH);

    analogWrite (motor_front_speed, motor_speed);
    analogWrite (motor_back_speed, motor_speed);
}

static inline void car_move_right ()
{
    digitalWrite (motor_out_1, LOW);
    delay (1);
    digitalWrite (motor_out_2, HIGH);
    delay (1);
    digitalWrite (motor_out_3, HIGH);
    delay (1);
    digitalWrite (motor_out_4, LOW);

    analogWrite (motor_front_speed, motor_speed);
    analogWrite (motor_back_speed, motor_speed);
}

static inline void car_move_left ()
{
    digitalWrite (motor_out_1, HIGH);
    delay (1);
    digitalWrite (motor_out_2, LOW);
    delay (1);
    digitalWrite (motor_out_3, LOW);
    delay (1);
    digitalWrite (motor_out_4, HIGH);

    analogWrite (motor_front_speed, motor_speed);
    analogWrite (motor_back_speed, motor_speed);
}

static inline void turn_front_light_on ()
{
	digitalWrite (front_light, HIGH);
}

static inline void turn_back_light_on ()
{
	digitalWrite (back_light, HIGH);
}

static inline void turn_front_light_off ()
{
	digitalWrite (front_light, LOW);
}

static inline void turn_back_light_off ()
{
	digitalWrite (back_light, LOW);
}

static inline void turn_rgb_light_on ()
{
	digitalWrite (inner_rgb_light, HIGH);
}

static inline void turn_rgb_light_off ()
{
  digitalWrite (inner_rgb_light, LOW);
}

static void atomatic_movement ()
{
  double obstacle_distance = ultrasonic();

  if (obstacle_distance <= minimum_obstacle_detect) 
  {
    obstacle_detect_forward ();
    return;
  }
  
  car_move_forward();
  delay (400);
  car_stop ();
}

double obstacle_sensor_see_right()
{
  servo_motor.write(5);
  delay(800);
  return ultrasonic();
}

double ultrasonic() 
{
  delay (50);
  digitalWrite (obstacle_sensor_trig, LOW);
  delayMicroseconds (4);
  digitalWrite (obstacle_sensor_trig, HIGH);
  delayMicroseconds (10);
  digitalWrite (obstacle_sensor_trig, LOW);
  double time_taken = pulseIn (obstacle_sensor_echo, HIGH);
  return (time_taken / 29.0d) / 2.0d;
}

double obstacle_sensor_see_left()
{
  servo_motor.write(180);
  delay(800);
  return ultrasonic();
}

static inline void obstacle_detect_forward ()
{
  car_stop();
  car_move_backward();
  delay(100);
  car_stop();
  
  double left_obstacle_distance = obstacle_sensor_see_left();
  servo_motor.write(servo_motor_center);
  delay(800);
  double right_obstacle_distance = obstacle_sensor_see_right();
  servo_motor.write(servo_motor_center);
  
  if (left_obstacle_distance < right_obstacle_distance) {
    obstacle_detected_left ();
    return;
  } 

  if (left_obstacle_distance > right_obstacle_distance) {
    obstacle_detected_right ();
    return;
  }

  return;
}

static inline void obstacle_detected_right ()
{
  car_move_left ();
  delay(500);
  car_stop();
  delay(200);
  return;
}

static inline void obstacle_detected_left ()
{
  car_move_right ();
  delay(500);
  car_stop();
  delay(200);
  return;
}

void print_directory(File dir, int num_tabs) 
{
  while (true) 
  {
    File entry =  dir.openNextFile();
    if (! entry) 
    {
      break;
    }

    for (uint8_t i = 0; i < num_tabs; i++) 
    {
      Serial.print('\t');
    }
    Serial.print(entry.name());

    if (entry.isDirectory()) 
    {
      Serial.println("/");
      print_directory(entry, num_tabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }

    entry.close();
  }
}

static void manual_movement ()
{
  switch (bluetooth_signal)
  {
    case 'W':
    {
	    turn_front_light_on ();
      break;
    }

    case 'w':
    {
	    turn_front_light_off ();
      break;
    }

    case 'U':
    {
	    turn_back_light_on ();
      break;
    }

    case 'u':
    {
	    turn_back_light_off ();
      break;
    }

    case 'V':
    {
      if (SD.exists ("car_horn.wav"))
      {
        if (!speaker.isPlaying ())
        {
          speaker.play ("CAR_HO~1.WAV");
          speaker.loop (1);
        }
        break;
      }

      Serial.println ("CAR_H~1.WAV not found.");
      break;
    }

    case 'v':
    {
      if (SD.exists ("CAR_HO~1.WAV"))
      {
        if (!speaker.isPlaying())
        {
	        speaker.disable ();
        }
      }
      break;
    }

    case 'X':
    {
	    turn_rgb_light_on ();
      break;
    }

    case 'x':
    {
	    turn_rgb_light_off ();
      break;
    }

    case 'F':
    {
	    car_move_forward ();
      break;
    }

    case 'B':
    {
	    car_move_backward ();
      break;
    }

    case 'L':
    {
	    car_move_left ();
      break;
    }

    case 'R':
    {
	    car_move_right ();
      break;
    }

    case '0':
    {
      motor_speed = 0;
      break;
    }

    case '1':
    {
      motor_speed = 250/10;
      break;
    }

    case '2':
    {
      motor_speed = (250/10)*2;
      break;
    }

    case '3':
    {
      motor_speed = (250/10)*3;
      break;
    }

    case '4':
    {
      motor_speed = (250/10)*4;
      break;
    }

    case '5':
    {
      motor_speed = (250/10)*5;
      break;
    }

    case '6':
    {
      motor_speed = (250/10)*6;
      break;
    }

    case '7':
    {
      motor_speed = (250/10)*7;
      break;
    }

    case '8':
    {
      motor_speed = (250/10)*8;
      break;
    }

    case '9':
    {
      motor_speed = (250/10)*9;
      break;
    }

    case 'q':
    {
      motor_speed = 254;
      break;
    }

  }
}
void loop () 
{
  if(Serial.available () > 0) // Checks whether data is comming from the serial port
  {
  	bluetooth_signal = Serial.read (); // Reads the data from the serial port  
    car_stop ();
  }

  switch (bluetooth_signal)
  {
    case 'a':
    {
      is_manual = true;
      break;
    }
    case 'A':
    {
      is_manual = false;
      break;
    }
    case 'P':
    {
      if (sd_found == true)
      {
        print_directory (SD.open("/"), 0);
        break;
      }

      Serial.println("SD Card not found!");
    }
    case 'M':
    {
      if (speaker.isPlaying() == false)
      {
        speaker.play ("GARIWALA.wav");
        break;
      }

      Serial.println("Already playing!");
      break;
    }
    case 'm':
    {
      if (speaker.isPlaying() == true)
      {
        speaker.disable();
        break;
      }
      
      Serial.println("Speaker not playing!");
      break;
    }
    case '+':
    {
      if (speaker_volume == 5)
      {
        Serial.println("Volume already maxed");
        break;
      }

      speaker_volume = speaker_volume + 1;
      speaker.setVolume(speaker_volume);
      Serial.print("Volume: ");Serial.println(speaker_volume);
      break;
    }
    case '-':
    {
      if (speaker_volume == 0)
      {
        Serial.println("Volume already minimum");
        break;
      }

      speaker_volume = speaker_volume -1;
      speaker.setVolume(speaker_volume);
      Serial.print("Volume: ");Serial.println(speaker_volume);
      break;
    }
  }

  if (is_manual == true)
  {
    manual_movement ();
    return;
  }

  atomatic_movement ();

}

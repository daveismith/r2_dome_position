#include <PID_v1.h>
#include <SPI.h>

#define CHIP_SELECT 10

SPISettings rotaryEncoderSettings(1000000, MSBFIRST, SPI_MODE1);

double setpoint, in, out;

PID domePID(&in, &out, &setpoint, 2.0, 5.0, 1.0, P_ON_M, DIRECT);
//PID domePID(&in, &out, &setpoint, 2.0, 5.0, 1.0, DIRECT);


typedef struct {
  unsigned int data : 12;
  bool ocf : 1;
  bool cof : 1;
  bool lin : 1;
  bool mag_inc : 1;
  bool mag_dec : 1;
  bool even_par : 1;
} encoder_output_t;

encoder_output_t read_encoder() {
  encoder_output_t decoded;
  uint8_t low, mid, high;

  // Read 16 bits of data from the encoder, selecting the encoder before the attempt and 
  // deselecting at the end.
  digitalWrite(CHIP_SELECT, LOW);
  SPI.beginTransaction(rotaryEncoderSettings);
  high = SPI.transfer(0);
  low = SPI.transfer(0);
  digitalWrite(CHIP_SELECT, HIGH);
  SPI.endTransaction();

  // Fake Some Values
  high = 0x80;
  mid = 0x00;
  low = 0x40;

  // Decode The Data From The Encoder
  decoded.data = (high << 4) | (mid >> 4);
  decoded.ocf = (mid >> 3) & 0x01;
  decoded.cof = (mid >> 2) & 0x01;
  decoded.lin = (mid >> 1) & 0x01;
  decoded.mag_inc = mid & 0x01;
  decoded.mag_dec = (low >> 7) & 0x01;
  decoded.even_par = (low >> 6) & 0x01;

  // Return The Structure With The Decoded Response
  return decoded;
}

int find_distance(unsigned int current, unsigned int target, unsigned int total_steps) {
  /* Figures out the shortest distance between two values in a circle which is divided into total_steps sections.
   *
   * The current parameter represents the current position on the circle. The target parameter represents the 
   * desired value. The total_steps indicates the total number of values that make up the circle (eg, 360)
   */

  int current_angle_normalized = current % total_steps;
  int target_angle_normalized = target % total_steps;

#ifdef FIND_DISTANCE_DEBUG
  Serial.print("current_angle: ");
  Serial.println(current_angle_normalized);
  Serial.print("target_angle: ");
  Serial.println(target_angle_normalized);
  Serial.print("total_steps: ");
  Serial.println(total_steps);
  
  int angle_a_not_normal = current_angle_normalized - target_angle_normalized;
  Serial.print("angle_a_not_normal: ");
  Serial.println(angle_a_not_normal);
#endif

  //int angle_a = angle_a_not_normal % total_steps;
  int angle_a = (current_angle_normalized - target_angle_normalized) % (int)total_steps;
  int angle_b = (target_angle_normalized - current_angle_normalized) % (int)total_steps;

  while (angle_a < 0) { angle_a += 360; }
  while (angle_b < 0) { angle_b += 360; }

#ifdef FIND_DISTANCE_DEBUG
  Serial.print("angle_a: ");
  Serial.println(angle_a);
  Serial.print("angle_b: ");
  Serial.println(angle_b);
#endif

  if (angle_a >= angle_b) {
    return angle_b;
  }

  return -angle_a;
}

void setup() {
  // Configure the GPIO for the Chip Select
  pinMode(CHIP_SELECT, OUTPUT);

  // Initialize the variables
  in = 0;
  setpoint = 10;
  out = 0;


  domePID.SetMode(AUTOMATIC);
  domePID.SetSampleTime(100);
  domePID.SetOutputLimits(-2000, 2000);

  // Start SPI Peripheral
  SPI.begin();

  Serial.begin(115200);
}

#define BUFFER_SIZE 512
size_t input_position = 0, position;
char input_buffer[BUFFER_SIZE + 1];

// unsigned int int_setpoint = 0;

void loop() {
  encoder_output_t pos = read_encoder();
  size_t available = Serial.available();
  
  if (available > 0) {
    if (available > (BUFFER_SIZE - input_position)) {
      available = BUFFER_SIZE - input_position;
    }
    
    input_position += Serial.readBytes(&input_buffer[input_position], available);

    position = input_position - 1;

    bool should_print = false, only_numbers = true;
    while (position > 0) {
      if (input_buffer[position] == '\r' || input_buffer[position] == '\n') {
        input_buffer[position] = '\0';
        should_print = true;
      } else if (input_buffer[position] < '0' || input_buffer[position] > '9') {
        only_numbers = false; 
      }
      position--;
    }

    if (should_print || input_position >= BUFFER_SIZE) {
      // Serial.print(">>");
      // Serial.print(input_buffer);
      // Serial.print("<<");
      // Serial.println();
      if (only_numbers) {
        long val = 0;
        for (size_t i = 0; i < (input_position - 1); i++) {
          uint8_t digit = input_buffer[i] - '0';
          if (digit > 9) { break; }
          val *= 10;
          val += digit;
        }

        // Serial.print("# = ");
        // Serial.print(val);
        // Serial.println();

        // int_setpoint = pos.data;
        // int distance = find_distance(int_setpoint, val, 360);
        // Serial.print(setpoint);
        // Serial.print("==>");
        // Serial.print(val);
        // Serial.print(" = ");
        // Serial.print(distance);
        // Serial.println();
        setpoint = (double)val;
      }
      // Serial.flush();
      input_position = 0;
      memset(input_buffer, 0, BUFFER_SIZE + 1);
    }
  }

  // in = pos.data;
  bool resp = domePID.Compute();
  
  Serial.print(in);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(out);
  Serial.println();
  Serial.flush();

  in += out / 100;

  delay(100);
}

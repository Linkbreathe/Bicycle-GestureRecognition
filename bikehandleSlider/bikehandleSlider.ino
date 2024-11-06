#include <CapacitiveSensor.h>

double analog_slider_right1;
double analog_slider_right2;
double analog_slider_right3;

double analog_slider_left1;
double analog_slider_left2;
double analog_slider_left3;

int sender = 25;

int slider_pin_left1 = 13;
int slider_pin_left2 = 12;
int slider_pin_left3 = 14;

int slider_pin_right1 = 27;
int slider_pin_right2 = 33;
int slider_pin_right3 = 32;

int samples_touch = 10;

// Thresholds and maximum values for each sensor
int sliderLeft1MinThresh = 60;
int sliderLeft2MinThresh = 17;
int sliderLeft3MinThresh = 35;

int sliderRight1MinThresh = 30;
int sliderRight2MinThresh = 25;
int sliderRight3MinThresh = 21;

int sliderLeft1MaxValue = 147;
int sliderLeft2MaxValue = 130;
int sliderLeft3MaxValue = 154;

int sliderRight1MaxValue = 147;
int sliderRight2MaxValue = 130;
int sliderRight3MaxValue = 154;

// Capacitive sensor objects
CapacitiveSensor slideSensorLeft1 = CapacitiveSensor(sender, slider_pin_left1);
CapacitiveSensor slideSensorLeft2 = CapacitiveSensor(sender, slider_pin_left2);
CapacitiveSensor slideSensorLeft3 = CapacitiveSensor(sender, slider_pin_left3);

CapacitiveSensor slideSensorRight1 = CapacitiveSensor(sender, slider_pin_right1);
CapacitiveSensor slideSensorRight2 = CapacitiveSensor(sender, slider_pin_right2);
CapacitiveSensor slideSensorRight3 = CapacitiveSensor(sender, slider_pin_right3);

const int mean_window_size_left = 5;        // Number of readings in the rolling mean
double position_history_left[mean_window_size_left] = {0};
int history_index_left = 0;
double previous_mean_position_left = -1.0;
bool is_sliding_left = false;
bool last_slide_left = false;
unsigned long last_left_slide_time = 0;

const int mean_window_size_right = 5;        // Number of readings in the rolling mean
double position_history_right[mean_window_size_right] = {0};
int history_index_right = 0;
double previous_mean_position_right = -1.0;
bool is_sliding_right = false;
bool last_slide_right = false;
unsigned long last_right_slide_time = 0;


const double slide_threshold = 0.09;    // Smaller threshold to detect gradual sliding
const unsigned long slide_end_timeout = 500;  // Timeout for detecting slide end after a left slide (in milliseconds)

void setup() {
   Serial.begin(9600);
}

void loop() {
  // Get raw sensor values
  analog_slider_left1 = slideSensorLeft1.capacitiveSensor(samples_touch);
  analog_slider_left2 = slideSensorLeft2.capacitiveSensor(samples_touch);
  analog_slider_left3 = slideSensorLeft3.capacitiveSensor(samples_touch);

  analog_slider_right1 = slideSensorRight1.capacitiveSensor(samples_touch);
  analog_slider_right2 = slideSensorRight2.capacitiveSensor(samples_touch);
  analog_slider_right3 = slideSensorRight3.capacitiveSensor(samples_touch);

  // Check touch thresholds
  int is_touchedLeft1 = (analog_slider_left1 > sliderLeft1MinThresh) ? 1 : 0;
  int is_touchedLeft2 = (analog_slider_left2 > sliderLeft2MinThresh) ? 1 : 0;
  int is_touchedLeft3 = (analog_slider_left3 > sliderLeft3MinThresh) ? 1 : 0;

  int is_touchedRight1 = (analog_slider_right1 > sliderRight1MinThresh) ? 1 : 0;
  int is_touchedRight2 = (analog_slider_right2 > sliderRight2MinThresh) ? 1 : 0;
  int is_touchedRight3 = (analog_slider_right3 > sliderRight3MinThresh) ? 1 : 0;

  int touch_count_left = is_touchedLeft1 + is_touchedLeft2 + is_touchedLeft3;
  int touch_count_right = is_touchedRight1 + is_touchedRight2 + is_touchedRight3;

  double overall_position_left = -1.0;
  double overall_position_right = -1.0;

  // Calculate position based on touch count
  if (touch_count_left > 0) {
    double total_weight_left = 0.0;
    double weighted_position_left = 0.0;

    // Calculate weighted position based on each sensor's contribution
    if (is_touchedLeft1) {
      double weight = analog_slider_left1 / sliderLeft1MaxValue;
      weighted_position_left += 0.0 * weight;
      total_weight_left += weight;
    }
    if (is_touchedLeft2) {
      double weight = analog_slider_left2 / sliderLeft2MaxValue;
      weighted_position_left += 0.5 * weight;
      total_weight_left += weight;
    }
    if (is_touchedLeft3) {
      double weight = analog_slider_left3 / sliderLeft3MaxValue;
      weighted_position_left += 1.0 * weight;
      total_weight_left += weight;
    }

    // Compute final position
    if (total_weight_left > 0) {
      overall_position_left = weighted_position_left / total_weight_left;
    }
  }

  // Add the current position to the history buffer and update the rolling mean
  position_history_left[history_index_left] = overall_position_left;
  history_index_left = (history_index_left + 1) % mean_window_size_left;

  // Calculate the rolling mean of the last `mean_window_size` positions
  double mean_position_left = 0.0;
  int valid_readings_left = 0;
  for (int i = 0; i < mean_window_size_left; i++) {
    if (position_history_left[i] >= 0) {  // Ignore uninitialized entries
      mean_position_left += position_history_left[i];
      valid_readings_left++;
    }
  }
  mean_position_left = (valid_readings_left > 0) ? mean_position_left / valid_readings_left : -1.0;

  // Slide detection logic using rolling mean
  if (mean_position_left >= 0.0) {
    if (previous_mean_position_left == -1.0) {
      // First valid mean position, no slide detected yet
      previous_mean_position_left = mean_position_left;
    } else {
      if (mean_position_left < previous_mean_position_left - slide_threshold) {
        Serial.println("Slide Left Detected");
        is_sliding_left = true;
        last_slide_left = true;
        last_left_slide_time = millis();  // Record the time of the left slide
      }
      // Update the previous mean position for the next loop
      previous_mean_position_left = mean_position_left;
    }
  } else {
    // No touch detected, reset sliding state
    if (is_sliding_left) {
      unsigned long current_time = millis();
      
      if (last_slide_left && (current_time - last_left_slide_time < slide_end_timeout)) {
        Serial.println("LIGHT LEFT");  // Special output for left slide followed by end
      } else {
        Serial.println("Slide Ended");
      }
      
      is_sliding_left = false;
      last_slide_left = false;
    }
    previous_mean_position_left = -1.0;
  }

  // Output the calculated mean position if it’s valid
  if (mean_position_left >= 0.0) {
    Serial.print("LEFT,");
    Serial.print(mean_position_left, 2);
    Serial.println();
  }

  // Calculate position based on touch count
  if (touch_count_right > 0) {
    double total_weight_right = 0.0;
    double weighted_position_right = 0.0;

    // Calculate weighted position based on each sensor's contribution
    if (is_touchedRight1) {
      double weight = analog_slider_right1 / sliderRight1MaxValue;
      weighted_position_right += 0.0 * weight;
      total_weight_right += weight;
    }
    if (is_touchedRight2) {
      double weight = analog_slider_right2 / sliderRight2MaxValue;
      weighted_position_right += 0.5 * weight;
      total_weight_right += weight;
    }
    if (is_touchedRight3) {
      double weight = analog_slider_right3 / sliderRight3MaxValue;
      weighted_position_right += 1.0 * weight;
      total_weight_right += weight;
    }

    // Compute final position
    if (total_weight_right > 0) {
      overall_position_right = weighted_position_right / total_weight_right;
    }
  }

  // Add the current position to the history buffer and update the rolling mean
  position_history_right[history_index_right] = overall_position_right;
  history_index_right = (history_index_right + 1) % mean_window_size_right;

  // Calculate the rolling mean of the last `mean_window_size` positions
  double mean_position_right = 0.0;
  int valid_readings_right = 0;
  for (int i = 0; i < mean_window_size_right; i++) {
    if (position_history_right[i] >= 0) {  // Ignore uninitialized entries
      mean_position_right += position_history_right[i];
      valid_readings_right++;
    }
  }
  mean_position_right = (valid_readings_right > 0) ? mean_position_right / valid_readings_right : -1.0;

  // Slide detection logic using rolling mean
  if (mean_position_right >= 0.0) {
    if (previous_mean_position_right == -1.0) {
      // First valid mean position, no slide detected yet
      previous_mean_position_right = mean_position_right;
    } else {
      if (mean_position_right < previous_mean_position_right - slide_threshold) {
        Serial.println("Slide Right Detected");
        is_sliding_right = true;
        last_slide_right = true;
        last_right_slide_time = millis();  // Record the time of the left slide
      }
      // Update the previous mean position for the next loop
      previous_mean_position_right = mean_position_right;
    }
  } else {
    // No touch detected, reset sliding state
    if (is_sliding_right) {
      unsigned long current_time = millis();
      
      if (last_slide_right && (current_time - last_right_slide_time < slide_end_timeout)) {
        Serial.println("LIGHT RIGHT");  // Special output for left slide followed by end
      } else {
        Serial.println("Slide Ended");
      }
      
      is_sliding_right = false;
      last_slide_right = false;
    }
    previous_mean_position_right = -1.0;
  }

  // Output the calculated mean position if it’s valid
  if (mean_position_right >= 0.0) {
    Serial.print("RIGHT,");
    Serial.print(mean_position_right, 2);
    Serial.println();
  }

  delay(100);
}

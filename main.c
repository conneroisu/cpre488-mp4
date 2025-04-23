#include <limits.h>
#include <stdint.h>
#include <stdio.h>

// Incorrect implementation (casting before comparison)
static inline int16_t incorrect_saturateSignedInt16(float in) {
  // casting first "wraps" or UB's large floats,
  // so the > / < comparisons never work the way you expect
  if ((int16_t)in > INT16_MAX)
    return INT16_MAX;
  if ((int16_t)in < -INT16_MAX)
    return -INT16_MAX;
  return (int16_t)in;
}

// Correct implementation (comparing before casting)
static inline int16_t correct_saturateSignedInt16(float in) {
  if (in > (float)INT16_MAX)
    return INT16_MAX;
  if (in < -(float)INT16_MAX)
    return -INT16_MAX;
  return (int16_t)in;
}

int main() {
  // Test values: within range, slightly above range, way above range
  float test_values[] = {
      0.0f,      // Zero
      100.0f,    // Small positive value
      -100.0f,   // Small negative value
      32767.0f,  // INT16_MAX
      -32768.0f, // INT16_MIN
      32768.0f,  // Just above INT16_MAX
      -32769.0f, // Just below INT16_MIN
      50000.0f,  // Well above INT16_MAX
      -50000.0f, // Well below INT16_MIN
      100000.0f, // Very large positive
      -100000.0f // Very large negative
  };

  int num_tests = sizeof(test_values) / sizeof(test_values[0]);

  printf("Testing saturation functions with various inputs:\n");
  printf("%-15s %-15s %-15s %-15s\n", "Input (float)", "Expected", "Incorrect",
         "Correct");
  printf("------------------------------------------------------------\n");

  for (int i = 0; i < num_tests; i++) {
    float input = test_values[i];

    // Calculate expected value
    int16_t expected;
    if (input > INT16_MAX) {
      expected = INT16_MAX;
    } else if (input < INT16_MIN) {
      expected = INT16_MIN;
    } else {
      expected = (int16_t)input;
    }

    // Get results from both implementations
    int16_t incorrect_result = incorrect_saturateSignedInt16(input);
    int16_t correct_result = correct_saturateSignedInt16(input);

    // Print results
    printf("%-15.2f %-15d %-15d %-15d", input, expected, incorrect_result,
           correct_result);

    // Highlight differences
    if (incorrect_result != expected) {
      printf(" <- FAIL");
    }
    printf("\n");
  }

  return 0;
}

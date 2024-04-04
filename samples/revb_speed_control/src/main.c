#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>

static struct pwm_dt_spec servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
const struct device *const servo_dev = DEVICE_DT_GET(DT_NODELABEL(servo));

const struct pwm_dt_spec servo_spec = {
  .dev = servo_dev,
	.channel = 0,
	.period = PWM_NSEC(10000),
	.flags = PWM_POLARITY_NORMAL,
};

int main(void)
{
  pwm_set_pulse_dt(&servo_spec, 0);

  int initial_width = servo.period;

  while(1) {
    servo.period = initial_width;

    for (int j = 0; j < 15; ++j) {
      int pulse_widths[7] = {0.05 * servo.period, 0.15 * servo.period, 0.30 * servo.period,
        0.45 * servo.period, 0.60 * servo.period, 0.75 * servo.period, 0.90 * servo.period};

      for (int i = 0; i < 7; ++i) {
        pwm_set_pulse_dt(&servo, pulse_widths[i]);

        printk("count = %d,\t total width (ns) = %d,\t pulse width (ns) = %d,\t frequency (khz) = %d\n",
          j, servo.period, pulse_widths[i], 1000000000 / servo.period);

        k_sleep(K_SECONDS(10000));
      }

      servo.period = servo.period * 100 / 120;
      // servo.period = servo.period / (1 + 200 * (float) servo.period / 1000000000);
    }
  }

  return 0;
}

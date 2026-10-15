/* Where the follower's tip is, live, from C alone: what a board with a display runs, the display here a
 * terminal line. The calibration comes from the servos' EEPROM and the offsets from SO101_OFFSETS (as the
 * Python side reads them), so nothing but the arm is needed. The torque goes off first, so the arm turns by
 * hand: hold it if it stands raised with its torque on.
 *
 *     make example && ./build/where /dev/tty.usbmodem58760431541      (Linux: /dev/ttyACM0); Ctrl-C ends it */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "so101ik.h"

/* "shoulder_lift=2.3,elbow_flex=-5.9": degrees by joint, 0 for a joint not named; -1 if it is not that. */
static int read_offsets(const char *s, so101_real off[SO101_NA]) {
    static const char *names[SO101_NA] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};
    while (s && *s) {
        while (*s == ' ' || *s == ',') s++;
        if (!*s) break;
        const char *eq = strchr(s, '=');
        if (!eq) return -1;
        int j = 0;
        while (j < SO101_NA && !(strlen(names[j]) == (size_t)(eq - s) && !strncmp(s, names[j], (size_t)(eq - s)))) j++;
        if (j == SO101_NA) return -1;
        char *end;
        off[j] = (so101_real)strtod(eq + 1, &end);
        if (end == eq + 1 || (*end && *end != ',' && *end != ' ')) return -1;
        s = end;
    }
    return 0;
}

int main(int argc, char **argv) {
    static so101_arm a;
    so101_hal hal;
    so101_servo_cal cal[SO101_NJ];
    so101_real off[SO101_NA] = {0}, tip[3], pitch;
    if (argc != 2) {
        fprintf(stderr, "usage: %s PORT   (the follower's serial port)\n", argv[0]);
        return 2;
    }
    if (read_offsets(getenv("SO101_OFFSETS"), off)) {
        fprintf(stderr, "SO101_OFFSETS: joint=degrees, comma-separated, e.g. shoulder_lift=2.3,elbow_flex=-5.9\n");
        return 2;
    }
    if (so101_hal_posix_open(&hal, argv[1])) {
        fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }
    if (so101_cal_read(&hal, cal)) {
        fprintf(stderr, "servos 1-6 on %s do not answer: is the arm powered?\n", argv[1]);
        return 1;
    }
    so101_arm_init(&a, &hal, cal);
    memcpy(a.model.offsets, off, sizeof off);
    if (so101_arm_torque_off(&a)) {
        fprintf(stderr, "%s\n", a.err);
        return 1;
    }
    printf("the tip in mm (x forward, y to the arm's left, z up) and the gripper's pitch in degrees (-90 down)\n");
    for (;;) {
        if (so101_arm_where(&a, NULL, tip, &pitch)) {
            fprintf(stderr, "\n%s\n", a.err);
            return 1;
        }
        printf("\rx %6.1f   y %6.1f   z %6.1f   pitch %6.1f ", 1000 * (double)tip[0], 1000 * (double)tip[1],
               1000 * (double)tip[2], (double)pitch);
        fflush(stdout);
        hal.sleep_us(hal.ctx, 50000);
    }
}

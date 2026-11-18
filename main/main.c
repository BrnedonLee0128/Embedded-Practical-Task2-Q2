#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"

/* ---------- Config ---------- */
#define PRIO_A   1      // lowest
#define PRIO_B   2
#define PRIO_C   3
#define PRIO_D   4
#define PRIO_E   5      // highest
#define STACK    4096
#define WORK_MS  20     // simulated compute time per iteration (busy-wait)
#define R_TANK   1.5    // tank radius (m)

#define GATE_BIT    (1 << 0)
#define DONE_A      (1 << 0)
#define DONE_B      (1 << 1)
#define DONE_C      (1 << 2)
#define DONE_D      (1 << 3)
#define DONE_E      (1 << 4)
#define DONE_ALL    (DONE_A | DONE_B | DONE_C | DONE_D | DONE_E)

static EventGroupHandle_t gate;       // start gate: all tasks wait here
static EventGroupHandle_t done_grp;   // each task sets its bit when finished

static double result[5];              // 0=A ... 4=E
static char   order[6];
static int    order_n = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static inline long long now_ms(void) { return esp_timer_get_time() / 1000; }

// Busy-wait: task stays Ready, only a higher-priority task can take the CPU.
static void work(void) {
    int64_t end = esp_timer_get_time() + (int64_t)WORK_MS * 1000;
    while (esp_timer_get_time() < end) { }
}

static void wait_gate(void) {
    xEventGroupWaitBits(gate, GATE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

static void finish(char name, int idx, double value, EventBits_t bit) {
    portENTER_CRITICAL(&mux);
    result[idx] = value;
    order[order_n++] = name;
    portEXIT_CRITICAL(&mux);
    printf("[%5lld ms] Task %c FINAL SUM = %.4f  <-- finished #%d\n",
           now_ms(), name, value, order_n);
    xEventGroupSetBits(done_grp, bit);
}

/* ---------- Task E (prio 5): E = pi r^2 + 2 pi r^2 ---------- */
static void task_E(void *pv) {
    wait_gate();
    double sum = 0, area = M_PI * R_TANK * R_TANK;
    for (int i = 1; i <= 3; i++) {                 // main tank + 2 reserve tanks
        work();
        sum += area;
        printf("[%5lld ms] Task E (prio %d) tank %d: partial = %.4f\n", now_ms(), PRIO_E, i, sum);
    }
    finish('E', 4, sum, DONE_E);
    vTaskDelete(NULL);
}

/* ---------- Task D (prio 4): D = 1 + 4/1! + 4^2/2! ---------- */
static void task_D(void *pv) {
    wait_gate();
    double sum = 0, term = 1;
    for (int k = 0; k <= 2; k++) {
        work();
        if (k > 0) term *= 4.0 / k;                // 4^k / k!
        sum += term;
        printf("[%5lld ms] Task D (prio %d) k=%d: partial = %.4f\n", now_ms(), PRIO_D, k, sum);
    }
    finish('D', 3, sum, DONE_D);
    vTaskDelete(NULL);
}

/* ---------- Task C (prio 3): C = sum (n+3)/n, n = 1..5 ---------- */
static void task_C(void *pv) {
    wait_gate();
    double sum = 0;
    for (int n = 1; n <= 5; n++) {
        work();
        sum += (n + 3.0) / n;
        printf("[%5lld ms] Task C (prio %d) n=%d: partial = %.4f\n", now_ms(), PRIO_C, n, sum);
    }
    finish('C', 2, sum, DONE_C);
    vTaskDelete(NULL);
}

/* ---------- Task B (prio 2): B = sum 2^m / m!, m = 1..5 ---------- */
static void task_B(void *pv) {
    wait_gate();
    double sum = 0, term = 1;
    for (int m = 1; m <= 5; m++) {
        work();
        term *= 2.0 / m;                           // 2^m / m!
        sum += term;
        printf("[%5lld ms] Task B (prio %d) m=%d: partial = %.4f\n", now_ms(), PRIO_B, m, sum);
    }
    finish('B', 1, sum, DONE_B);
    vTaskDelete(NULL);
}

/* ---------- Task A (prio 1): A = sum [x + (x+5)], x = 0..20 ---------- */
static void task_A(void *pv) {
    wait_gate();
    int sum = 0;
    for (int x = 0; x <= 20; x++) {
        work();
        sum += x + (x + 5);
        printf("[%5lld ms] Task A (prio %d) x=%d: partial = %d\n", now_ms(), PRIO_A, x, sum);
    }
    finish('A', 0, (double)sum, DONE_A);
    vTaskDelete(NULL);
}

/* ---------- Summary ---------- */
static void summary(void) {
    static const char  *name[5]     = {"A", "B", "C", "D", "E"};
    static const double expect[5]   = {525.0, 6.2667, 11.85, 13.0, 21.206};

    printf("\n======== SUMMARY ========\n");
    printf("Task | Priority | Expected | Printed  | Result\n");
    for (int i = 0; i < 5; i++) {
        int ok = fabs(result[i] - expect[i]) < 0.001;
        printf("  %s  |    %d     | %8.4f | %8.4f | %s\n",
               name[i], i + 1, expect[i], result[i], ok ? "PASS" : "FAIL");
    }
    order[order_n] = '\0';
    printf("Completion order: %c, %c, %c, %c, %c  (expected E, D, C, B, A) -> %s\n",
           order[0], order[1], order[2], order[3], order[4],
           (order[0]=='E' && order[1]=='D' && order[2]=='C' && order[3]=='B' && order[4]=='A')
               ? "PASS" : "FAIL");
    printf("=========================\n");
}

void app_main(void) {
    gate     = xEventGroupCreate();
    done_grp = xEventGroupCreate();

    printf("Creating tasks in order A, B, C, D, E\n");
    xTaskCreate(task_A, "Task A", STACK, NULL, PRIO_A, NULL);  printf("  created A (prio %d)\n", PRIO_A);
    xTaskCreate(task_B, "Task B", STACK, NULL, PRIO_B, NULL);  printf("  created B (prio %d)\n", PRIO_B);
    xTaskCreate(task_C, "Task C", STACK, NULL, PRIO_C, NULL);  printf("  created C (prio %d)\n", PRIO_C);
    xTaskCreate(task_D, "Task D", STACK, NULL, PRIO_D, NULL);  printf("  created D (prio %d)\n", PRIO_D);
    xTaskCreate(task_E, "Task E", STACK, NULL, PRIO_E, NULL);  printf("  created E (prio %d)\n", PRIO_E);

    printf("All five Ready. Releasing start gate.\n\n");
    xEventGroupSetBits(gate, GATE_BIT);

    xEventGroupWaitBits(done_grp, DONE_ALL, pdFALSE, pdTRUE, portMAX_DELAY);
    summary();
}
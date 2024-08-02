#include <stdio.h>
#include <math.h>

#define DT 0.01       // Time step
#define PI 3.14159265358979323846

typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
} State;

typedef struct {
    float data[6][6];
} Matrix6x6;

typedef struct {
    float data[6];
} Vector6;

void matrix6x6_identity(Matrix6x6 *m) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            m->data[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void matrix6x6_multiply(Matrix6x6 *result, Matrix6x6 *a, Matrix6x6 *b) {
    Matrix6x6 temp = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                temp.data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }
    *result = temp;
}

void matrix6x6_add(Matrix6x6 *result, Matrix6x6 *a, Matrix6x6 *b) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            result->data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }
}

void vector6_add(Vector6 *result, Vector6 *a, Vector6 *b) {
    for (int i = 0; i < 6; i++) {
        result->data[i] = a->data[i] + b->data[i];
    }
}

void vector6_subtract(Vector6 *result, Vector6 *a, Vector6 *b) {
    for (int i = 0; i < 6; i++) {
        result->data[i] = a->data[i] - b->data[i];
    }
}

void matrix6x6_subtract(Matrix6x6 *result, Matrix6x6 *a, Matrix6x6 *b) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            result->data[i][j] = a->data[i][j] - b->data[i][j];
        }
    }
}

void ekf_predict(State *state, Matrix6x6 *P, Matrix6x6 *Q, float gx, float gy, float gz) {
    // State transition matrix F (simplified for this example)
    Matrix6x6 F = {0};
    matrix6x6_identity(&F);

    // Update state with gyro data
    state->roll += gx * DT;
    state->pitch += gy * DT;
    state->yaw += gz * DT;

    // Predict state covariance P = F * P * F^T + Q
    Matrix6x6 Ft = F;
    Matrix6x6 FP;
    matrix6x6_multiply(&FP, &F, P);
    matrix6x6_multiply(P, &FP, &Ft);
    matrix6x6_add(P, P, Q);
}

void ekf_update(State *state, Matrix6x6 *P, Matrix6x6 *R, float ax, float ay, float az) {
    // Measurement matrix H (simplified for this example)
    Matrix6x6 H = {0};
    matrix6x6_identity(&H);

    // Measurement residual y = z - H * x
    Vector6 z = {ax, ay, az, 0.0f, 0.0f, 0.0f};
    Vector6 hx = {state->x, state->y, state->z, state->roll, state->pitch, state->yaw};
    Vector6 y;
    vector6_subtract(&y, &z, &hx);

    // Kalman gain K = P * H^T * (H * P * H^T + R)^-1
    Matrix6x6 PHt, HP, HPHtR, HPHtR_inv, K;
    matrix6x6_multiply(&PHt, P, &H);
    matrix6x6_multiply(&HP, &H, P);
    matrix6x6_add(&HPHtR, &HP, R);
    // Assuming HPHtR is invertible and directly inverting (simplified, not numerically stable)
    HPHtR_inv = HPHtR; // Replace with a proper matrix inversion function
    matrix6x6_multiply(&K, &PHt, &HPHtR_inv);

    // Update state x = x + K * y
    Vector6 Ky;
    for (int i = 0; i < 6; i++) {
        Ky.data[i] = 0.0f;
        for (int j = 0; j < 6; j++) {
            Ky.data[i] += K.data[i][j] * y.data[j];
        }
    }
    vector6_add(&hx, &hx, &Ky);
    state->x = hx.data[0];
    state->y = hx.data[1];
    state->z = hx.data[2];
    state->roll = hx.data[3];
    state->pitch = hx.data[4];
    state->yaw = hx.data[5];

    // Update state covariance P = (I - K * H) * P
    Matrix6x6 I_KH, I = {0};
    matrix6x6_identity(&I);
    Matrix6x6 KH;
    matrix6x6_multiply(&KH, &K, &H);
    matrix6x6_subtract(&I_KH, &I, &KH);
    Matrix6x6 temp;
    matrix6x6_multiply(&temp, &I_KH, P);
    *P = temp;
}

int main() {
    State state = {0};
    Matrix6x6 P, Q, R;
    matrix6x6_identity(&P);
    matrix6x6_identity(&Q);
    matrix6x6_identity(&R);

    // Replace these with actual sensor readings
    float gx, gy, gz;  // Gyroscope data (angular velocities)
    float ax, ay, az;  // Accelerometer data (linear accelerations)

    // Example loop (replace with actual loop in your application)
    for (int i = 0; i < 100; i++) {
        // Get sensor data (replace with actual data acquisition)
        gx = 0.01; gy = 0.02; gz = 0.03;
        ax = 0.1; ay = 0.2; az = 9.8;

        // EKF prediction and update
        ekf_predict(&state, &P, &Q, gx, gy, gz);
        ekf_update(&state, &P, &R, ax, ay, az);

        // Print the estimated pose
        printf("Orientation: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", state.roll, state.pitch, state.yaw);
        printf("Position: X=%.2f, Y=%.2f, Z=%.2f\n", state.x, state.y, state.z);
    }

    return 0;
}
#define NUM_STATES 3 // 状態数。実際の問題に合わせて変更
#define NUM_OBSERVATIONS 1 // 観測数。これは実際の問題に合わせて変更

class Observer {
private:
   float A[NUM_STATES][NUM_STATES];
   float B[NUM_STATES];
   float C[NUM_OBSERVATIONS][NUM_STATES];
   float L[NUM_STATES][NUM_OBSERVATIONS];
   float t;

public:
   float x_est[NUM_STATES];

   Observer(float A[NUM_STATES][NUM_STATES], float B[NUM_STATES], float C[NUM_OBSERVATIONS][NUM_STATES], float L[NUM_STATES][NUM_OBSERVATIONS]) {
      for (int i = 0; i < NUM_STATES; i++) {
         for (int j = 0; j < NUM_STATES; j++) {
            this->A[i][j] = A[i][j];
         }
         this->B[i] = B[i];
         for (int j = 0; j < NUM_OBSERVATIONS; j++) {
            this->L[i][j] = L[i][j]; // Observer gain
            this->C[j][i] = C[j][i];
         }
      }
   }

   void initialize(float x_est_init[]) {
      for (int i = 0; i < NUM_STATES; i++) {
         x_est[i] = x_est_init[i];
      }
      t = 0;
   }

   void dynamics(float x[], float u, float y[NUM_OBSERVATIONS], float x_dot[]) {
      float y_est[NUM_OBSERVATIONS];

      for (int j = 0; j < NUM_OBSERVATIONS; j++) {
         y_est[j] = 0;
         for (int i = 0; i < NUM_STATES; i++) {
            y_est[j] += C[j][i] * x[i];
         }
      }

      // x_dot = Ax + Bu + L(y - y_est)の計算
      for (int i = 0; i < NUM_STATES; i++) {
         x_dot[i] = 0;
         for (int j = 0; j < NUM_STATES; j++) {
            x_dot[i] += A[i][j] * x[j];
         }
         x_dot[i] += B[i] * u;
      }
      for (int j = 0; j < NUM_OBSERVATIONS; j++) {
         for (int i = 0; i < NUM_STATES; i++) {
            x_dot[i] += L[i][j] * (y[j] - y_est[j]);
         }
      }
   }

   void step(float y[NUM_OBSERVATIONS], float u, float dt = 0.01) {
      float k1[NUM_STATES], k2[NUM_STATES], k3[NUM_STATES], k4[NUM_STATES];
      float x_tmp[NUM_STATES];

      // Simplified Runge-Kutta implementation
      dynamics(x_est, u, y, k1);
      for (int i = 0; i < NUM_STATES; i++) {
         x_tmp[i] = x_est[i] + 0.5 * dt * k1[i];
      }

      dynamics(x_tmp, u, y, k2);
      for (int i = 0; i < NUM_STATES; i++) {
         x_tmp[i] = x_est[i] + 0.5 * dt * k2[i];
      }

      dynamics(x_tmp, u, y, k3);
      for (int i = 0; i < NUM_STATES; i++) {
         x_tmp[i] = x_est[i] + dt * k3[i];
      }

      dynamics(x_tmp, u, y, k4);
      for (int i = 0; i < NUM_STATES; i++) {
         x_est[i] += dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
      }

      t += dt;
   }

   float* getEstimate() {
      return x_est;
   }
};

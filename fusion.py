import math
import time
import threading


class Fusion:
    def __init__(self):
        self.lock = threading.Lock()

        # Heading (IMU yaw in radians)
        self.yaw = 0.0

        # EKF state [px, py, vx, vy]
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.P = [
            [0.05, 0.0, 0.0, 0.0],
            [0.0, 0.05, 0.0, 0.0],
            [0.0, 0.0, 0.4, 0.0],
            [0.0, 0.0, 0.0, 0.4],
        ]

        # Process and measurement tuning
        self.process_accel_sigma = 0.7
        self.vmax = 1.0
        self.k_vel = 0.20
        self.v_meas_max = 1.0

        self.R_move = [
            [0.05, 0.0],
            [0.0, 0.05],
        ]
        self.R_stationary_mid = [
            [0.25, 0.0],
            [0.0, 0.25],
        ]
        self.R_stationary_small = [
            [0.60, 0.0],
            [0.0, 0.60],
        ]
        self.R_zupt = [
            [0.0001, 0.0],
            [0.0, 0.0001],
        ]

        # UWB filtering/gating
        self.uwb_max_jump = 2.0
        self.jump_buffer = 0.6
        self.ema_alpha = 0.65
        self.uwb_deadband = 0.025
        self.median_window = 5
        self.stationary_small_gate = 0.05
        self.stationary_outlier_gate = 0.18

        # Startup bootstrap
        self.init_required = 8
        self.init_spread_max = 0.4
        self.initialized = False
        self.init_samples = []

        # IMU stationary classifier
        self.accel_noise_floor = 0.04
        self.gyro_noise_floor = 0.08
        self.accel_lp_alpha = 0.30
        self.gyro_lp_alpha = 0.25
        self.acc_stationary_enter = 0.07
        self.acc_stationary_exit = 0.10
        self.gyro_stationary_enter = 0.06
        self.gyro_stationary_exit = 0.14
        self.stationary_samples_required = 20
        self.stationary_counter = 0
        self.imu_stationary = False
        self.accel_lp = 0.0
        self.gyro_lp = 0.0

        # Non-holonomic constraint (cart does not strafe)
        self.nhc_gain = 0.8
        self.nhc_min_speed = 0.03

        # UWB history
        self.last_uwb = None
        self.last_uwb_time = None
        self.uwb_hist_x = []
        self.uwb_hist_y = []
        self.uwb_ema_x = None
        self.uwb_ema_y = None

    @staticmethod
    def _matmul4(A, B):
        out = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for k in range(4):
                aik = A[i][k]
                for j in range(4):
                    out[i][j] += aik * B[k][j]
        return out

    @staticmethod
    def _transpose4(A):
        return [[A[j][i] for j in range(4)] for i in range(4)]

    @staticmethod
    def _add4(A, B):
        return [[A[i][j] + B[i][j] for j in range(4)] for i in range(4)]

    def _predict(self, dt):
        A = [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

        px, py, vx, vy = self.state
        self.state = [px + vx * dt, py + vy * dt, vx, vy]

        # Process noise from white acceleration model
        q = self.process_accel_sigma * self.process_accel_sigma
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = [
            [0.25 * dt4 * q, 0.0, 0.5 * dt3 * q, 0.0],
            [0.0, 0.25 * dt4 * q, 0.0, 0.5 * dt3 * q],
            [0.5 * dt3 * q, 0.0, dt2 * q, 0.0],
            [0.0, 0.5 * dt3 * q, 0.0, dt2 * q],
        ]

        AP = self._matmul4(A, self.P)
        AT = self._transpose4(A)
        self.P = self._add4(self._matmul4(AP, AT), Q)

        # Non-holonomic cart constraint in body frame:
        # lateral velocity should be near zero.
        vx, vy = self.state[2], self.state[3]
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > self.nhc_min_speed and not self.imu_stationary:
            c = math.cos(self.yaw)
            s = math.sin(self.yaw)
            v_forward = c * vx + s * vy
            v_lateral = -s * vx + c * vy
            v_lateral *= (1.0 - self.nhc_gain)
            self.state[2] = c * v_forward - s * v_lateral
            self.state[3] = s * v_forward + c * v_lateral

        # Hard cap final velocity magnitude
        vx, vy = self.state[2], self.state[3]
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > self.vmax and speed > 0.0:
            scale = self.vmax / speed
            self.state[2] *= scale
            self.state[3] *= scale

    def _update_pos(self, z_x, z_y, R):
        # Innovation y = z - Hx where H picks [px, py].
        y0 = z_x - self.state[0]
        y1 = z_y - self.state[1]

        s00 = self.P[0][0] + R[0][0]
        s01 = self.P[0][1] + R[0][1]
        s10 = self.P[1][0] + R[1][0]
        s11 = self.P[1][1] + R[1][1]
        det = s00 * s11 - s01 * s10
        if abs(det) < 1e-9:
            return

        inv00 = s11 / det
        inv01 = -s01 / det
        inv10 = -s10 / det
        inv11 = s00 / det

        # K = P H^T S^-1 (4x2)
        K = [[0.0, 0.0] for _ in range(4)]
        for i in range(4):
            p0 = self.P[i][0]
            p1 = self.P[i][1]
            K[i][0] = p0 * inv00 + p1 * inv10
            K[i][1] = p0 * inv01 + p1 * inv11

        # state update
        for i in range(4):
            self.state[i] += K[i][0] * y0 + K[i][1] * y1

        # covariance update: P=(I-KH)P
        KH = [
            [K[0][0], K[0][1], 0.0, 0.0],
            [K[1][0], K[1][1], 0.0, 0.0],
            [K[2][0], K[2][1], 0.0, 0.0],
            [K[3][0], K[3][1], 0.0, 0.0],
        ]
        I_KH = [
            [1.0 - KH[0][0], -KH[0][1], 0.0, 0.0],
            [-KH[1][0], 1.0 - KH[1][1], 0.0, 0.0],
            [-KH[2][0], -KH[2][1], 1.0, 0.0],
            [-KH[3][0], -KH[3][1], 0.0, 1.0],
        ]
        self.P = self._matmul4(I_KH, self.P)

    def _zupt_update(self):
        # Pseudo-measurement: vx = 0, vy = 0
        y0 = -self.state[2]
        y1 = -self.state[3]

        s00 = self.P[2][2] + self.R_zupt[0][0]
        s01 = self.P[2][3] + self.R_zupt[0][1]
        s10 = self.P[3][2] + self.R_zupt[1][0]
        s11 = self.P[3][3] + self.R_zupt[1][1]
        det = s00 * s11 - s01 * s10
        if abs(det) < 1e-9:
            return

        inv00 = s11 / det
        inv01 = -s01 / det
        inv10 = -s10 / det
        inv11 = s00 / det

        # K = P H^T S^-1 where H picks [vx, vy]
        K = [[0.0, 0.0] for _ in range(4)]
        for i in range(4):
            p2 = self.P[i][2]
            p3 = self.P[i][3]
            K[i][0] = p2 * inv00 + p3 * inv10
            K[i][1] = p2 * inv01 + p3 * inv11

        for i in range(4):
            self.state[i] += K[i][0] * y0 + K[i][1] * y1

        KH = [
            [0.0, 0.0, K[0][0], K[0][1]],
            [0.0, 0.0, K[1][0], K[1][1]],
            [0.0, 0.0, K[2][0], K[2][1]],
            [0.0, 0.0, K[3][0], K[3][1]],
        ]
        I_KH = [
            [1.0, 0.0, -KH[0][2], -KH[0][3]],
            [0.0, 1.0, -KH[1][2], -KH[1][3]],
            [0.0, 0.0, 1.0 - KH[2][2], -KH[2][3]],
            [0.0, 0.0, -KH[3][2], 1.0 - KH[3][3]],
        ]
        self.P = self._matmul4(I_KH, self.P)

        self.state[2] = 0.0
        self.state[3] = 0.0

    def imu_predict(self, yaw, ax_body, ay_body, gyro_z, dt):
        with self.lock:
            self.yaw = yaw

            if not self.initialized:
                return

            # Stationary classifier
            ax = 0.0 if abs(ax_body) < self.accel_noise_floor else ax_body
            ay = 0.0 if abs(ay_body) < self.accel_noise_floor else ay_body
            gz = 0.0 if abs(gyro_z) < self.gyro_noise_floor else gyro_z

            accel_mag = math.sqrt(ax * ax + ay * ay)
            self.accel_lp = self.accel_lp_alpha * accel_mag + (1 - self.accel_lp_alpha) * self.accel_lp
            self.gyro_lp = self.gyro_lp_alpha * abs(gz) + (1 - self.gyro_lp_alpha) * self.gyro_lp

            stationary_sample = (
                self.accel_lp < self.acc_stationary_enter
                and self.gyro_lp < self.gyro_stationary_enter
            )
            moving_sample = (
                self.accel_lp > self.acc_stationary_exit
                or self.gyro_lp > self.gyro_stationary_exit
            )

            if stationary_sample:
                self.stationary_counter = min(
                    self.stationary_samples_required,
                    self.stationary_counter + 1,
                )
            elif moving_sample:
                self.stationary_counter = 0

            self.imu_stationary = self.stationary_counter >= self.stationary_samples_required

            if dt > 0.0:
                self._predict(min(dt, 0.05))

            # ZUPT: enforce zero velocity when stationary
            if self.imu_stationary:
                self._zupt_update()

    def uwb_update(self, x_uwb, y_uwb):
        with self.lock:
            if x_uwb is None or y_uwb is None:
                return

            # Startup bootstrap
            if not self.initialized:
                self.init_samples.append((x_uwb, y_uwb))
                if len(self.init_samples) < self.init_required:
                    return

                xs = [p[0] for p in self.init_samples]
                ys = [p[1] for p in self.init_samples]
                if (max(xs) - min(xs)) > self.init_spread_max or (max(ys) - min(ys)) > self.init_spread_max:
                    self.init_samples.clear()
                    return

                x0 = sorted(xs)[len(xs) // 2]
                y0 = sorted(ys)[len(ys) // 2]
                now = time.time()

                self.state = [x0, y0, 0.0, 0.0]
                self.P = [
                    [0.05, 0.0, 0.0, 0.0],
                    [0.0, 0.05, 0.0, 0.0],
                    [0.0, 0.0, 0.4, 0.0],
                    [0.0, 0.0, 0.0, 0.4],
                ]

                self.uwb_hist_x = [x0]
                self.uwb_hist_y = [y0]
                self.uwb_ema_x = x0
                self.uwb_ema_y = y0
                self.last_uwb = (x0, y0)
                self.last_uwb_time = now
                self.stationary_counter = self.stationary_samples_required
                self.imu_stationary = True
                self.initialized = True
                self.init_samples.clear()
                return

            # UWB prefilter (median + EMA)
            self.uwb_hist_x.append(x_uwb)
            self.uwb_hist_y.append(y_uwb)
            if len(self.uwb_hist_x) > self.median_window:
                self.uwb_hist_x.pop(0)
            if len(self.uwb_hist_y) > self.median_window:
                self.uwb_hist_y.pop(0)

            med_x = sorted(self.uwb_hist_x)[len(self.uwb_hist_x) // 2]
            med_y = sorted(self.uwb_hist_y)[len(self.uwb_hist_y) // 2]
            self.uwb_ema_x = self.ema_alpha * med_x + (1 - self.ema_alpha) * self.uwb_ema_x
            self.uwb_ema_y = self.ema_alpha * med_y + (1 - self.ema_alpha) * self.uwb_ema_y
            x_meas = self.uwb_ema_x
            y_meas = self.uwb_ema_y

            now = time.time()
            dt = now - self.last_uwb_time
            if dt <= 0.0:
                return
            dt = min(dt, 0.25)
            dt_vel = max(dt, 0.08)

            px, py = self.state[0], self.state[1]
            dx = x_meas - px
            dy = y_meas - py
            err = math.sqrt(dx * dx + dy * dy)

            max_jump = min(self.uwb_max_jump, self.vmax * dt + self.jump_buffer)
            if err > max_jump:
                return

            # Stationary three-zone behavior
            if self.imu_stationary:
                self.state[2] = 0.0
                self.state[3] = 0.0

                if err >= self.stationary_outlier_gate:
                    self.last_uwb = (x_meas, y_meas)
                    self.last_uwb_time = now
                    return

                if err < self.stationary_small_gate:
                    self._update_pos(x_meas, y_meas, self.R_stationary_small)
                else:
                    self._update_pos(x_meas, y_meas, self.R_stationary_mid)

                self.last_uwb = (x_meas, y_meas)
                self.last_uwb_time = now
                return

            # Generic deadband for moving mode
            if err < self.uwb_deadband:
                self.state[2] = 0.0 if abs(self.state[2]) < 0.02 else self.state[2] * 0.8
                self.state[3] = 0.0 if abs(self.state[3]) < 0.02 else self.state[3] * 0.8
                self.last_uwb = (x_meas, y_meas)
                self.last_uwb_time = now
                return

            # Moving update
            self._update_pos(x_meas, y_meas, self.R_move)

            prev_x, prev_y = self.last_uwb
            vx_meas = (x_meas - prev_x) / dt_vel
            vy_meas = (y_meas - prev_y) / dt_vel
            vx_meas = max(-self.v_meas_max, min(self.v_meas_max, vx_meas))
            vy_meas = max(-self.v_meas_max, min(self.v_meas_max, vy_meas))

            self.state[2] += self.k_vel * (vx_meas - self.state[2])
            self.state[3] += self.k_vel * (vy_meas - self.state[3])

            speed = math.sqrt(self.state[2] * self.state[2] + self.state[3] * self.state[3])
            if speed > self.vmax and speed > 0.0:
                scale = self.vmax / speed
                self.state[2] *= scale
                self.state[3] *= scale

            self.last_uwb = (x_meas, y_meas)
            self.last_uwb_time = now

    def get_position(self):
        with self.lock:
            if not self.initialized:
                return {"x": None, "y": None, "heading": math.degrees(self.yaw) % 360}

            return {
                "x": round(self.state[0], 3),
                "y": round(self.state[1], 3),
                "heading": round(math.degrees(self.yaw) % 360, 4),
            }

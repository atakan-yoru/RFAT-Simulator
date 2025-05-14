import numpy as np

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0,
                 frame_size=1024, sampling_frequency=1000.0,
                 output_limits=(-np.inf, np.inf)):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.frame_size = frame_size
        self.sampling_frequency = sampling_frequency
        self.dt = 1.0 / sampling_frequency

        self.previous_error = np.zeros(frame_size, dtype=np.float32)
        self.integral = np.zeros(frame_size, dtype=np.float32)

        # Output saturation limits
        self.output_min, self.output_max = output_limits

    def compute_control(self, error_vector):
        """
        Compute PID control signals for a batch (vector) of errors,
        correctly scaled according to sampling time (dt), and clipped to output limits.
        
        Parameters
        ----------
        error_vector : ndarray (shape: frame_size,)
            Current batch of errors.

        Returns
        -------
        control_signal_vector : ndarray (shape: frame_size,)
        """
        error_vector = np.asarray(error_vector, dtype=np.float32)

        self.integral += error_vector * self.dt
        derivative = (error_vector - self.previous_error) / self.dt
        self.previous_error = error_vector.copy()

        control_signal = (self.kp * error_vector +
                          self.ki * self.integral +
                          self.kd * derivative)

        control_signal[np.abs(control_signal) < 1e-12] = 0.0

        # Apply output saturation
        control_signal = np.clip(control_signal, self.output_min, self.output_max)

        return control_signal


class EKF:
    """
    Vectorised Extended Kalman Filter for *angle + angular-rate* tracking.

    Two modes:
      - mode='static_mount'      : antenna boresight is fixed (most common test)
      - mode='moving_mount'      : placeholder for future implementation
    """

    def __init__(self, frame_size=1024, dt=0.1,
                process_var=1e-3, meas_var=1e-1,
                mode="static_mount",
                meas_threshold=1e-1):

        # ---- State vectors -------------------------------------------------
        # x = [theta, omega]ᵀ  for each sample in the vector
        self.x = np.zeros((frame_size, 2, 1), dtype=np.float32)
        # One 2×2 covariance matrix per sample
        self.P = np.repeat(np.eye(2, dtype=np.float32)[None, :, :],
                           frame_size, axis=0)

        # Constant matrices (same for all samples, so scalar-ised)
        self.F = np.array([[1, dt],
                           [0, 1]], dtype=np.float32)

        self.Q = process_var * np.array([[dt**4/4, dt**3/2],
                                         [dt**3/2, dt**2]], dtype=np.float32)

        self.H = np.array([[1, 0]], dtype=np.float32)  # direct angle measurement
        self.R = np.array([[meas_var]], dtype=np.float32)
        self.meas_threshold = meas_threshold
        self.mode = mode
        self.dt = dt
        self.frame_size = frame_size
        self.process_var = process_var
        self.meas_var = meas_var


    # ---------------------------------------------------------------------

    def _predict(self):
        """ Time‐update: x = F x ;  P = F P Fᵀ + Q, for each of the frame_size states """
        # xₖ₋₁ is (N,2,1), F is (2,2) → result (N,2,1)
        self.x = np.einsum('ij,njk->nik', self.F, self.x)

        # Pₖ₋₁ is (N,2,2), F is (2,2), want (N,2,2)
        self.P = np.einsum('ij,njk,kl->nil', self.F, self.P, self.F.T) + self.Q

    @staticmethod
    def wrap_pi(a):
        """Wrap angles in `a` to [-π, +π]."""
        return (a + np.pi) % (2*np.pi) - np.pi


    def _update_static(self,
                       z_left, z_right,
                       theta_left=-np.deg2rad(25),
                       theta_right=+np.deg2rad(25),
                       n=-np.log(2)/np.log(np.cos(np.deg2rad(60/2)))):
        """
        EKF update step using normalized measurement:
          z_norm = (P_R - P_L)/(P_R + P_L),
        and skipping any sample where (P_R+P_L) < meas_threshold.
        """
        # flatten
        zL = np.asarray(z_left,  dtype=np.float32).flatten()
        zR = np.asarray(z_right, dtype=np.float32).flatten()
        N = self.frame_size

        # 1) build normalized measurement vector
        denom_meas = zL + zR
        print(f"denom_meas: {np.mean(denom_meas)}")
        z_norm = np.zeros_like(denom_meas, dtype=np.float32)
        valid_meas = denom_meas > self.meas_threshold
        z_norm[ valid_meas ] = (zR[valid_meas] - zL[valid_meas]) / denom_meas[valid_meas]

        # 2) predicted angles
        theta_pred = self.x[:,0,0]

        # 3) predicted left/right “beam” terms
        cos_r = np.cos(theta_pred - theta_right)
        cos_l = np.cos(theta_pred - theta_left)
        cos_r[cos_r < 0] = 0
        cos_l[cos_l < 0] = 0

        a = cos_r**n
        b = cos_l**n
        denom_pred = a + b

        # 4) predicted normalized measurement h_pred
        h_pred = np.zeros_like(denom_pred, dtype=np.float32)
        valid_pred = denom_pred > self.meas_threshold
        h_pred[ valid_pred ] = (a[valid_pred] - b[valid_pred]) / denom_pred[valid_pred]

        # 5) innovation y = z_norm - h_pred
        y = (z_norm - h_pred).reshape(N,1,1)

        # 6) Jacobian dh/dtheta of the normalized ratio:
        #    h = (a - b)/(a + b)  with  a = cos_r**n,  b = cos_l**n
        #    da = d/dθ [cos_r**n] = -n cos_r^(n-1) sin(theta-theta_r)
        #    db = d/dθ [cos_l**n] = -n cos_l^(n-1) sin(theta-theta_l)
        da = -n * cos_r**(n-1) * np.sin(theta_pred - theta_right)
        db = -n * cos_l**(n-1) * np.sin(theta_pred - theta_left)

        dh_dtheta = np.zeros_like(theta_pred, dtype=np.float32)
        # apply ratio derivative only where denom_pred is safe
        num = a - b
        sum_ = denom_pred
        # (da-db)*sum - num*(da+db)  all over sum^2
        dh = ((da - db)*sum_ - num*(da + db))
        dh_dtheta[ valid_pred ] = dh[valid_pred] / (sum_[valid_pred]**2)

        # 7) Build H (N×1×2) and do the usual EKF gain + update
        H = np.zeros((N,1,2), dtype=np.float32)
        H[:,0,0] = dh_dtheta
        # H[:,0,1] stays zero (we don’t observe ω directly)

        S = np.einsum('nij,njk,nkl->nil', H, self.P, H.transpose(0,2,1)) + self.R
        K = np.einsum('nij,njk->nik', self.P, H.transpose(0,2,1)) / S

        # state & covariance update
        self.x += K * y
        I = np.eye(2, dtype=np.float32)[None,:,:]
        KH = np.einsum('nij,njk->nik', K, H)
        self.P = np.einsum('nij,njk->nik', I - KH, self.P)




    def step(self, measurement_vector):
        """
        One EKF iteration (predict+update) on the whole 1024-sample block.
        Returns the filtered angle vector (same shape as input).
        """
        if self.mode == "moving_mount":
            # Placeholder for future: include mount angle in the state
            return measurement_vector

        # ==== static-mount EKF ====
        self._predict()
        self._update_static(np.asarray(measurement_vector, dtype=np.float32))
        return self.x[:, 0, 0]   # filtered theta for each sample


class UKF:
    """
    Vectorised Unscented Kalman Filter for *angle + angular-rate* tracking,
    with the same interface as your EKF (static_mount only).
    """
    def __init__(self, frame_size=1024, dt=0.1,
                 process_var=1e-3, meas_var=1e-1,
                 mode="static_mount", meas_threshold=1e-1,
                 alpha=1e-3, beta=2.0, kappa=0.0):
        self.frame_size = frame_size
        self.dt = dt
        self.mode = mode
        self.meas_threshold = meas_threshold

        # State: x = [theta, omega] per sample
        self.x = np.zeros((frame_size, 2, 1), dtype=np.float32)
        # Covariance per sample
        self.P = np.repeat(np.eye(2, dtype=np.float32)[None,:,:],
                           frame_size, axis=0)

        # Linear part (for predict)
        self.F = np.array([[1, dt],
                           [0, 1]], dtype=np.float32)
        self.Q = process_var * np.array([[dt**4/4, dt**3/2],
                                         [dt**3/2, dt**2]], dtype=np.float32)

        # Measurement noise
        self.R = np.array([[meas_var]], dtype=np.float32)

        # Unscented‐Transform parameters
        L = 2
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lambda_ = alpha*alpha*(L + kappa) - L
        self.gamma = np.sqrt(L + self.lambda_)

        # Weights for mean & covariance
        Wm0 = self.lambda_/(L + self.lambda_)
        Wc0 = Wm0 + (1 - alpha*alpha + beta)
        Wi = 1.0/(2*(L + self.lambda_))
        # length 2L+1 = 5
        self.Wm = np.array([Wm0] + [Wi]*(2*L), dtype=np.float32)
        self.Wc = np.array([Wc0] + [Wi]*(2*L), dtype=np.float32)

    @staticmethod
    def wrap_pi(a):
        return (a + np.pi) % (2*np.pi) - np.pi

    def _predict(self):
        # Linear predict
        self.x = np.einsum('ij,njk->nik', self.F, self.x)
        # Wrap the angle
        # Covariance predict
        self.P = np.einsum('ij,njk,kl->nil', self.F, self.P, self.F.T) + self.Q

    def _sigma_points(self, x, P):
        """
        Build 2L+1 sigma points for a single state x (shape (2,)) and P (2×2).
        Returns array shape (5, 2).
        """
        L = 2
        # scaled Cholesky
        S = np.linalg.cholesky((L + self.lambda_) * P)
        X = np.zeros((2*L+1, L), dtype=np.float32)
        X[0] = x
        for i in range(L):
            col = S[:, i]
            X[i+1]   = x + col
            X[L+1+i] = x - col
        return X

    def _update_static(self,
                       z_left, z_right,
                       theta_left=-np.deg2rad(25),
                       theta_right=+np.deg2rad(25),
                       n=-np.log(2)/np.log(np.cos(np.deg2rad(60/2)))):
        """
        UKF‐based update step with normalized measurement
            z_norm = (zR - zL)/(zR + zL),
        skipping samples where (zR+zL)<meas_threshold.
        """
        N = self.frame_size
        # flatten inputs
        zL = np.asarray(z_left,  dtype=np.float32).flatten()
        zR = np.asarray(z_right, dtype=np.float32).flatten()

        for i in range(N):
            # extract this sample's state & cov
            x_i = self.x[i,:,0].astype(np.float64)   # (2,)
            P_i = self.P[i].astype(np.float64)      # (2,2)

            # 1) Predict with UT through linear F+Q
            #   (since f is linear, this matches EKF predict)
            x_pred = self.F.dot(x_i)
            P_pred = self.F.dot(P_i).dot(self.F.T) + self.Q

            # 2) Generate sigma‐points around x_pred
            SP = self._sigma_points(x_pred, P_pred)  # (5,2)

            # 3) No process‐nonlinearity, so SP_pred = SP

            # 4) Measurement UT: propagate SP through h(.)
            Z = np.zeros(2*2+1, dtype=np.float64)  # length=5
            for k in range(5):
                θk = SP[k,0]
                # two‐antenna beam powers
                cr = max(np.cos(θk - theta_right), 0.0)
                cl = max(np.cos(θk - theta_left),  0.0)
                a = cr**n; b = cl**n
                d = a + b
                Z[k] = (a - b)/d if d > self.meas_threshold else 0.0

            # 5) Predicted measurement mean
            z_pred = np.dot(self.Wm, Z)

            # 6) Innovation covariance S and cross‐covariance Pxz
            S = self.R.copy().astype(np.float64)
            Pxz = np.zeros((2,), dtype=np.float64)
            for k in range(5):
                dz = Z[k] - z_pred
                dx = SP[k] - x_pred
                S   += self.Wc[k] * dz * dz
                Pxz += self.Wc[k] * dx * dz

            # 7) Kalman gain
            K = Pxz / S  # shape (2,)

            # 8) Actual normalized measurement
            d_meas = zL[i] + zR[i]
            z_norm = (zR[i] - zL[i]) / d_meas if d_meas > self.meas_threshold else 0.0

            # 9) State update
            dx = z_norm - z_pred
            x_upd = x_pred + K * dx


            # 10) Covariance update
            P_upd = P_pred - np.outer(K, K) * S

            # store back
            self.x[i,:,0] = x_upd.astype(np.float32)
            self.P[i]       = P_upd.astype(np.float32)

    def step(self, z_left, z_right, **kwargs):
        """
        Convenience: predict+update in one call.
        """
        self._predict()
        self._update_static(z_left, z_right, **kwargs)
        return self.x[:,0,0]

class KF:
    """
    Vectorised Extended Kalman Filter for *angle + angular-rate* tracking.

    Two modes:
      - mode='static_mount'      : antenna boresight is fixed (most common test)
      - mode='moving_mount'      : placeholder for future implementation
    """

    def __init__(self, frame_size=1024, dt=0.1,
                 process_var=1e-3, meas_var=1e-1,
                 mode="static_mount"):
        self.mode = mode
        self.frame_size = frame_size
        self.dt = dt

        # ---- State vectors -------------------------------------------------
        # x = [theta, omega]ᵀ  for each sample in the vector
        self.x = np.zeros((frame_size, 2, 1), dtype=np.float32)
        # One 2×2 covariance matrix per sample
        self.P = np.repeat(np.eye(2, dtype=np.float32)[None, :, :],
                           frame_size, axis=0)

        # Constant matrices (same for all samples, so scalar-ised)
        self.F = np.array([[1, dt],
                           [0, 1]], dtype=np.float32)

        self.Q = process_var * np.array([[dt**4/4, dt**3/2],
                                         [dt**3/2, dt**2]], dtype=np.float32)

        self.H = np.array([[1, 0]], dtype=np.float32)  # direct angle measurement
        self.R = np.array([[meas_var]], dtype=np.float32)  # scalar

    # ---------------------------------------------------------------------

    def _predict(self):
        """ Time‐update: x = F x ;  P = F P Fᵀ + Q, for each of the frame_size states """
        # xₖ₋₁ is (N,2,1), F is (2,2) → result (N,2,1)
        self.x = np.einsum('ij,njk->nik', self.F, self.x)

        # Pₖ₋₁ is (N,2,2), F is (2,2), want (N,2,2)
        self.P = np.einsum('ij,njk,kl->nil', self.F, self.P, self.F.T) + self.Q

    def _update_static(self, z):
        """
        Measurement-update when the antenna mount is static.
        z: 1D array of raw angle measurements (length = frame_size).
        """
        # ensure z is float32 and shape (N,)
        z = np.asarray(z, dtype=np.float32).flatten()
        N = self.frame_size

        # --- 1) Innovation: y = z - H x_pred  (H = [1 0])  ---
        # Predicted angle is x[:,0,0]
        theta_pred = self.x[:, 0, 0]               # shape (N,)
        y = (z - theta_pred).reshape(N, 1, 1)      # shape (N,1,1)

        # --- 2) Innovation covariance: S = H P Hᵀ + R  ---
        # Since H picks out P[:,0,0]:
        S = self.P[:, 0:1, 0:1] + self.R           # shape (N,1,1)

        # --- 3) Kalman gain: K = P Hᵀ S⁻¹  ---
        # Hᵀ = [[1],[0]] so P Hᵀ is just the first column of P
        K = self.P[:, :, 0:1] / S                  # shape (N,2,1)

        # --- 4) State update: x = x + K y  ---
        self.x += K * y                            # broadcast over (2×1)

        # --- 5) Covariance update: P = (I - K H) P  ---
        I = np.eye(2, dtype=np.float32)[None, :, :]     # shape (1,2,2)
        KH = K @ self.H                               # (N,2,1) @ (1,2) → (N,2,2)
        self.P = (I - KH) @ self.P                    # batch matmul (N,2,2)

    # ---------------------------------------------------------------------

    def step(self, measurement_vector):
        """
        One EKF iteration (predict+update) on the whole 1024-sample block.
        Returns the filtered angle vector (same shape as input).
        """
        if self.mode == "moving_mount":
            # Placeholder for future: include mount angle in the state
            return measurement_vector

        # ==== static-mount EKF ====
        self._predict()
        self._update_static(np.asarray(measurement_vector, dtype=np.float32))
        return self.x[:, 0, 0]   # filtered theta for each sample
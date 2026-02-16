import matplotlib.pyplot as plt
import numpy as np
import time

import simulation_interface as si


class ControllerPID:
    def __init__(self, kp, ki, kd, step_size):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.step_size = step_size
        self.cum_err = 0
        self.last_err = 0

    def step(self, err: float) -> float:
        self.cum_err += err * self.step_size
        d_err = (err - self.last_err) / self.step_size
        self.last_err = err
        return self.kp * err + self.ki * self.cum_err + self.kd * d_err


if __name__ == "__main__":
    # Ensure to replace "your_config_file.json" with an actual path
    # relevant to your SimulationInterface's constructor.
    config_file_path = "ford_expedition_2003"

    # # Set data paths
    # si.set_chrono_data_path(r"C:\Users\15309\Project_Chrono\chrono_simulink_cosim\data")
    # si.set_vehicle_data_path(
    #     r"C:\Users\15309\Project_Chrono\chrono_simulink_cosim\data"
    # )

    # Create an instance of the wrapper
    sim = si.SimulationInterface(config_file_path)

    # Load steering trajectory from CSV
    steering_data = np.loadtxt(
        r"C:\Users\15309\Project_Chrono\chrono_simulink_cosim\data\Ford Expedition 2003\Figures from Paper\Extracted Data\Fig9_Steering_Input_Test_Data.csv",
        delimiter=",",
    )
    time_steer = steering_data[:, 0]
    steer_deg = steering_data[:, 1]
    steer_rad = np.deg2rad(steer_deg)

    # Example si.Input data as a pandas DataFrame
    # Initialize a DataFrame with zeros and correct columns
    duration_s = 30
    n_steps = int(duration_s / sim.get_step_size())
    sim_time = np.arange(n_steps) * sim.get_step_size()

    # steer_ref = 0.8 * np.ones_like(sim_time)
    # dead_time = 3
    # dead_time_mask = sim_time < dead_time
    # steer_ref[dead_time_mask] = 0.0
    # velocity_ref = np.clip(1.5 * (sim_time - dead_time), 0, 100)
    # velocity_ref[dead_time_mask] = 0.0

    input_data_np = np.zeros((n_steps, si.Input.LENGTH))
    # input_data_np[:, si.Input.STEERING] = np.zeros_like(sim_time)  #
    # input_data_np[:, si.Input.STEERING] = 3.0 * np.sin(0.5 * sim_time)
    # square_mask = np.where(np.logical_and(sim_time > 4, sim_time < 7))
    # input_data_np[square_mask, si.Input.STEERING] = 0.1
    # delay_mask = sim_time < 20

    # Set a very small amount of throttle so that the automatic transmission stays in
    # forward gear. Apply delay to allow vehicle to settle on terrain.
    input_data_np[:, si.Input.THROTTLE] = 1.0 * np.ones_like(sim_time)
    # input_data_np[delay_mask, si.Input.THROTTLE] = 0
    # input_data_np[:, si.Input.BRAKE] = 1.0

    # DEBUG: Temporarily commented out plot
    # plt.figure()
    # plt.plot(sim_time, input_data_np[:, si.Input.THROTTLE], label="Throttle")
    # plt.plot(sim_time, input_data_np[:, si.Input.BRAKE], label="Brake")
    # plt.plot(sim_time, input_data_np[:, si.Input.STEERING], label="Steering")
    # plt.legend()
    # plt.grid()

    # Set terrain height (flat terrain at z=0)
    height = 0.0
    input_data_np[:, si.Input.TERRAIN_HEIGHT_FL] = height * np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_HEIGHT_FR] = height * np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_HEIGHT_RL] = height * np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_HEIGHT_RR] = height * np.ones_like(sim_time)

    # Set terrain normal vectors (upward normal for flat terrain: [0, 0, 1])
    input_data_np[:, si.Input.TERRAIN_NORMAL_X_FL] = np.zeros_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Y_FL] = np.zeros_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Z_FL] = np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_X_FR] = np.zeros_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Y_FR] = np.zeros_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Z_FR] = np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_X_RL] = np.zeros_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Y_RL] = np.zeros_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Z_RL] = np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_X_RR] = np.zeros_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Y_RR] = np.zeros_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Z_RR] = np.ones_like(sim_time)

    # Set friction coefficients
    input_data_np[:, si.Input.TERRAIN_MU_FL] = 0.8 * np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_MU_FR] = 0.8 * np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_MU_RL] = 0.8 * np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_MU_RR] = 0.8 * np.ones_like(sim_time)

    output_data_np = np.zeros((n_steps, si.Output.LENGTH))

    # steering_pid = ControllerPID(10.0, 1e2, 0.0, sim.get_step_size())
    # velocity_pid = ControllerPID(1e0, 4e-2, 0.0, sim.get_step_size())

    start_time = time.time()
    try:
        for i in range(input_data_np.shape[0]):
            time_now = output_data_np[max(0, i - 1), si.Output.SIM_TIME]
            if (i % 1000) == 0.0:
                print(f"time_now:{time_now}")
            # Set steering from CSV trajectory
            steer_cmd = np.interp(time_now, time_steer, steer_rad)
            # steer_cmd = 0.0
            input_data_np[i, si.Input.STEERING] = steer_cmd
            output_data_np[i, :] = sim.step(input_data_np[i, :])
    except RuntimeError as e:
        print(f"Got error: {e}")
    runtime = time.time() - start_time
    print(f"Real-time: {runtime} / Sim-time: {time_now} = {runtime/duration_s}")

    plt.figure()
    ax11 = plt.subplot(411)
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     steer_ref,
    #     label="Reference Pinion Angle (rad)",
    # )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.STEERING_PINION_ANGLE],
        label="Pinion Angle (rad)",
    )
    plt.grid()
    plt.legend()
    plt.subplot(412, sharex=ax11)
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        input_data_np[:, si.Input.STEERING],  # / 0.010,
        label="Steering Input (Nm)",
    )
    plt.legend()
    plt.grid()
    plt.subplot(413, sharex=ax11)
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     velocity_ref,
    #     label="Velocity Reference",
    # )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.CHASSIS_VEL_X],
        label="Chassis Velocity X (m/s)",
    )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.WHEEL_ANG_VEL_FL],
    #     label="Wheel Angular Velocity FL (rad/s)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.WHEEL_ANG_VEL_FR],
    #     label="Wheel Angular Velocity FR (rad/s)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.WHEEL_ANG_VEL_RL],
    #     label="Wheel Angular Velocity RL (rad/s)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.WHEEL_ANG_VEL_RR],
    #     label="Wheel Angular Velocity RR (rad/s)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.WHEEL_STEER_ANG_FL],
    #     label="Wheel Steer Angle FL (rad)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.WHEEL_STEER_ANG_FR],
    #     label="Wheel Steer Angle FR (rad)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_MOMENT_X_FL],
    #     label="Tire Moment X FL (Nm)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_MOMENT_Y_FL],
    #     label="Tire Moment Y FL (Nm)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_MOMENT_Z_FL],
    #     label="Tire Moment Z FL (Nm)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_MOMENT_Z_FR],
    #     label="Tire Moment Z FR (Nm)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_LAT_SLIP_FL],
    #     label="Tire Lateral Slip FL (rad)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_LAT_SLIP_FR],
    #     label="Tire Lateral Slip FR (rad)",
    # )
    plt.legend()
    plt.grid()
    plt.subplot(414, sharex=ax11)
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     input_data_np[:, si.Input.THROTTLE],
    #     label="Throttle",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_LAT_SLIP_FL],
    #     label="Tire lat slip FL (rad)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_LAT_SLIP_FR],
    #     label="Tire lat slip FR (rad)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_LONG_SLIP_FL],
    #     label="Tire lon slip FL (%)",
    # )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_FORCE_VERT_FL],
        label="Tire Force Z FL (N)",
    )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_FORCE_VERT_FR],
        label="Tire Force Z FR (N)",
    )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_FORCE_VERT_RL],
        label="Tire Force Z RL (N)",
    )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_FORCE_VERT_RR],
        label="Tire Force Z RR (N)",
    )
    plt.legend()
    plt.grid()
    plt.tight_layout()

    # Plot the four requested quantities
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 2, 1)
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.CHASSIS_VEL_X],
        label="Longitudinal Speed (m/s)",
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Longitudinal Speed (m/s)")
    plt.legend()
    plt.grid()

    plt.subplot(2, 2, 2)
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.CHASSIS_ACC_Y],
        label="Lateral Acceleration (m/s²)",
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Lateral Acceleration (m/s²)")
    plt.legend()
    plt.grid()

    plt.subplot(2, 2, 3)
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.CHASSIS_ANG_VEL_Z],
        label="Yaw Rate (rad/s)",
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw Rate (rad/s)")
    plt.legend()
    plt.grid()

    plt.subplot(2, 2, 4)
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.CHASSIS_ORIENT_X],
        label="Roll Angle (rad)",
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Roll Angle (rad)")
    plt.legend()
    plt.grid()

    plt.tight_layout()

    # plt.figure()
    # after_init_mask = sim_time > 4
    # # plt.plot(
    # #     output_data_np[after_init_mask, si.Output.TIRE_LAT_SLIP_FL],
    # #     output_data_np[after_init_mask, si.Output.TIRE_MOMENT_X_FL],
    # #     label="Moment X",
    # # )
    # # plt.plot(
    # #     output_data_np[after_init_mask, si.Output.TIRE_LAT_SLIP_FL],
    # #     output_data_np[after_init_mask, si.Output.TIRE_MOMENT_Y_FL],
    # #     label="Moment Y",
    # # )
    # plt.plot(
    #     output_data_np[after_init_mask, si.Output.TIRE_LAT_SLIP_FL],
    #     output_data_np[after_init_mask, si.Output.TIRE_MOMENT_Z_FL],
    #     label="Moment Z",
    # )
    # plt.xlabel("Lat Slip")
    # plt.legend()
    # plt.grid()
    # plt.tight_layout()

    # plt.figure()
    # plt.plot(
    #     output_data_np[after_init_mask, si.Output.TIRE_LONG_SLIP_FL],
    #     output_data_np[after_init_mask, si.Output.TIRE_MOMENT_X_FL],
    #     label="Moment X",
    # )
    # plt.plot(
    #     output_data_np[after_init_mask, si.Output.TIRE_LONG_SLIP_FL],
    #     output_data_np[after_init_mask, si.Output.TIRE_MOMENT_Y_FL],
    #     label="Moment Y",
    # )
    # plt.plot(
    #     output_data_np[after_init_mask, si.Output.TIRE_LONG_SLIP_FL],
    #     output_data_np[after_init_mask, si.Output.TIRE_MOMENT_Z_FL],
    #     label="Moment Z",
    # )
    # plt.xlabel("Lon Slip")
    # plt.legend()
    # plt.grid()

    plt.figure()
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_MOMENT_Z_FL],
    #     label="Moment Z FL (Nm)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_MOMENT_Z_FR],
    #     label="Moment Z FR (Nm)",
    # )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_FORCE_LONG_FL],
        label="Tire Force Long. FL (N)",
    )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_FORCE_LONG_FR],
        label="Tire Force Long. FR (N)",
    )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_FORCE_LONG_RL],
        label="Tire Force Long. RL (N)",
    )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_FORCE_LONG_RR],
        label="Tire Force Long. RR (N)",
    )
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.CHASSIS_ACC_Y],
        label="Acc Y (m/s^2)",
    )
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.CHASSIS_ANG_VEL_Z],
        label="Ang Vel Z (rad/s)",
    )
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.CHASSIS_ORIENT_X],
        label="Ang Orient X (rad)",
    )
    plt.legend()
    plt.grid()

    plt.show()

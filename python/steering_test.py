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
    config_file_path = "sedan"  # Example path, replace with your actual config file

    # Create an instance of the wrapper
    sim = si.SimulationInterface(config_file_path)

    # Example si.Input data as a pandas DataFrame
    # Initialize a DataFrame with zeros and correct columns
    duration_s = 7
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
    # input_data_np[:, si.Input.STEERING] = 0.3 * np.sin(0.5 * sim_time)
    # square_mask = np.where(np.logical_and(sim_time > 4, sim_time < 7))
    # input_data_np[square_mask, si.Input.STEERING] = 0.1
    # delay_mask = sim_time < 4

    # Set a very small amount of throttle so that the automatic transmission stays in
    # forward gear.
    input_data_np[:, si.Input.THROTTLE] = 0.01 * np.ones_like(sim_time)
    # input_data_np[delay_mask, si.Input.THROTTLE] = 0
    # input_data_np[:, si.Input.BRAKE] = 1.0
    input_data_np[:, si.Input.TERRAIN_NORMAL_Z_FL] = np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Z_FR] = np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Z_RL] = np.ones_like(sim_time)
    input_data_np[:, si.Input.TERRAIN_NORMAL_Z_RR] = np.ones_like(sim_time)
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
            sim_time = output_data_np[max(0, i - 1), si.Output.SIM_TIME]
            if (i % 1000) == 0.0:
                print(f"sim_time:{sim_time}")
            # if sim_time[i] > dead_time:
            #     input_data_np[i, si.Input.STEERING] = steering_pid.step(
            #         steer_ref[i]
            #         - output_data_np[max(0, i - 1), si.Output.STEERING_PINION_ANGLE]
            #     )
            #     vel_err = (
            #         velocity_ref[i]
            #         - output_data_np[max(0, i - 1), si.Output.CHASSIS_VEL_X]
            #     )
            #     throttle = velocity_pid.step(vel_err)
            #     # print(f"err:{vel_err}; cmd:{throttle}")
            #     input_data_np[i, si.Input.THROTTLE] = np.clip(
            #         throttle, a_min=0, a_max=1
            #     )
            output_data_np[i, :] = sim.step(input_data_np[i, :])
    except RuntimeError as e:
        print(f"Got error: {e}")
    runtime = time.time() - start_time
    print(f"Real-time: {runtime} / Sim-time: {duration_s} = {runtime/duration_s}")

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
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        input_data_np[:, si.Input.THROTTLE],
        label="Throttle",
    )
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
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_FORCE_VERT_FL],
    #     label="Tire Force Z FL (N)",
    # )
    # plt.plot(
    #     output_data_np[:, si.Output.SIM_TIME],
    #     output_data_np[:, si.Output.TIRE_FORCE_VERT_FR],
    #     label="Tire Force Z FR (N)",
    # )
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
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_MOMENT_Z_FL],
        label="Moment Z FL (Nm)",
    )
    plt.plot(
        output_data_np[:, si.Output.SIM_TIME],
        output_data_np[:, si.Output.TIRE_MOMENT_Z_FR],
        label="Moment Z FR (Nm)",
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

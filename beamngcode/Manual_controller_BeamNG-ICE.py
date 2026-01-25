import os 
import time
import pyautogui
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from hCCC_controller_ICE import hCCC
from scipy.interpolate import interp1d
from beamngpy import BeamNGpy, Scenario, Vehicle
import argparse
import re  # For regular expression matching
from beamngpy.sensors import Electrics
import math

# NEW IMPORTS FOR VIDEO FILE MANAGEMENT
import glob
import shutil

# NEW IMPORT FOR WINDOW FOCUS
from pywinauto import Desktop, Application # type: ignore

import pygame  # Add pygame import for controller support

def create_directory(path):
    """Ensure the specified directory exists."""
    if not os.path.exists(path):
        os.makedirs(path)

def activate_vr_mode():
    """Presses the relevant keys to switch BeamNG.tech to VR mode."""
    time.sleep(2)
    pyautogui.press('2')
    print("Switched to driver view")
    time.sleep(2)
    print("Activating VR mode...")
    for _ in range(2):
        pyautogui.keyDown('ctrl')
        pyautogui.press('num0')  # 'num0' represents Numpad 0
        pyautogui.keyUp('ctrl')
        time.sleep(2)  # Small delay between key presses
    pyautogui.keyDown('ctrl')
    pyautogui.press('num5')  # 'num5' represents Numpad 5
    pyautogui.keyUp('ctrl')
    print("VR mode activated and recentered.")
    time.sleep(2)

def get_next_test_number(directory):
    """Determines the next test number based on existing files in the directory."""
    test_numbers = []
    pattern = re.compile(r'Test(\d+)\.vehicle\..*\.csv')
    if os.path.exists(directory):
        for filename in os.listdir(directory):
            match = pattern.match(filename)
            if match:
                test_num = int(match.group(1))
                test_numbers.append(test_num)
    if test_numbers:
        return max(test_numbers) + 1
    else:
        return 1  # Start from 1 if no files are present

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='BeamNG controller')
    parser.add_argument('--control', type=str, required=True,
                        help='Control method (hccc)')
    parser.add_argument('--scn', type=int, required=True,
                        help='Scenario number (1 - 50)')
    parser.add_argument('--vehicle', type=str, default='etk800',
                        help='Ego vehicle model')
    args = parser.parse_args()

    control_method = args.control.lower()
    SCN = args.scn
    ego_vehicle_type = args.vehicle

    dt = 0.1  # Simulation timestep in seconds

    # Print important information
    print(f"Control method: {control_method}")
    print(f"Scenario number: {SCN}")
    print(f"Ego vehicle model: {ego_vehicle_type}")
    print(f"Simulation timestep (dt): {dt}")

    # Add after parsing arguments, before the VR mode prompt
    # Initialize pygame for controller input
    pygame.init()
    pygame.joystick.init()

    # Find vJoy device
    joystick_count = pygame.joystick.get_count()
    print(f"Number of joysticks detected: {joystick_count}")

    vjoy_device = None
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        print(f"Found joystick {i}: {joystick.get_name()}")
        if "vjoy" in joystick.get_name().lower():
            vjoy_device = joystick
            print(f"vJoy device found at index {i}")
            
    if vjoy_device is None:
        print("WARNING: vJoy device not found. Human override will not function.")


    # Ask if user wants to activate VR
    vr_bool = input("Do you want VR on or off (y/n): ")

    while vr_bool not in ['n', 'y']:
        print("Make sure to enter 'y' or 'n': lowercase please.")
        vr_bool = input("Do you want VR on or off (y/n): ")

    activate_vr_bool = (vr_bool == 'y')

    # Load the CSV file for the scenario speeds
    try:
        df = pd.read_csv(f'./scn/Scenarios.csv', header=0)
        print("Scenario CSV file loaded successfully.")

        # Verify that the file has at least three columns
        if df.shape[1] < 51:
            raise ValueError("The input CSV file does not have at least 61 columns. Please check the file format.")

        # Extract the target speeds (SCNnd column)
        # Multiply by 1.7 only if needed for your unit conversion
        #target_speed_list = df.iloc[:, SCN].values * 1.7
        target_speed_list = pd.to_numeric(df.iloc[:, SCN], errors='coerce').dropna().values
        N = len(target_speed_list)
        if N < 2:
            raise ValueError("Need at least 2 speed data points in the CSV to interpolate.")
        max_simulation_time = (N - 1) / 10

    except FileNotFoundError:
        print(f"File ./scn/Scenarios.csv not found. Please check the path.")
        exit(1)
    except ValueError as e:
        print(f"Error loading CSV file: {e}")
        exit(1)

    print(f"Maximum simulation time: {max_simulation_time} seconds")

    # Initialize BeamNGpy instance with Vulkan
    beamng = BeamNGpy('localhost',
                      64256,
                      home=r'D:\BeamNG.tech.v0.37.6.0')
    beamng.open(None, '-gfx', 'vk')  # Launch BeamNG.tech with Vulkan rendering

    # Create a new scenario in the 'west_coast_usa' map
    scenario = Scenario(
        'west_coast_usa',
        'TestScenario',
        description=(
            f"Welcome to the custom BeamNG simulation!\n\n"
            f"### How to Start VR:\n"
            f"1. Ensure your VR headset is properly connected and turned on.\n"
            f"2. Launch BeamNG.tech with your VR configuration enabled.\n"
            f"3. When the scenario loads, follow these steps to activate VR mode:\n"
            f"3.1) Ctrl + Numpad 0 (twice)\n"
            f"3.2) Ctrl + Numpad 5\n"
            f"4. Verify that the VR view is aligned correctly before starting the simulation.\n\n"
            f"### Simulation Overview:\n"
            f"This scenario involves two vehicles:\n"
            f"- **Ego Vehicle (Blue)**: This is the controlled vehicle that follows the specified path.\n"
            f"- **Preceding Vehicle (Red)**: This vehicle maintains a target speed as specified in the scenario configuration.\n\n"
            f"**Important Information:**\n"
            f"Scenario Number: {SCN}\n"
            f"Control Method: {control_method}\n"
            f"Ego Vehicle Model: {ego_vehicle_type}\n"
            f"Max Simulation Time: {max_simulation_time} s\n"
            f"Timestep: {dt} s\n"
            f"Make sure to monitor the outputs in the terminal to track simulation progress and collect data. All results, including speed, acceleration, and headway, will be saved to the designated output files."
        )
    )

    # Add vehicles
    ego_vehicle = Vehicle('ego_vehicle',
                          model=ego_vehicle_type,
                          licence='EGO',
                          colour='Blue')
    preceding_vehicle = Vehicle('preceding_vehicle',
                                model=ego_vehicle_type,
                                licence='PRE',
                                colour='Red')

    # Starting positions and rotations for both vehicles (pos, rot_quat)
    ego_start_pos = (-800.3449955, -464.0569456, 106.624861)
    ego_start_rot = (0.0017, 0.0053, 0.9169, -0.3991)
    pre_start_pos = (-793.2977155, -457.3357649, 106.6151421)
    pre_start_rot = (0.0014, 0.0043, 0.9235, -0.3836)

    scenario.add_vehicle(ego_vehicle, pos=ego_start_pos, rot_quat=ego_start_rot)
    scenario.add_vehicle(preceding_vehicle, pos=pre_start_pos, rot_quat=pre_start_rot)

    # Compile and load the scenario
    scenario.make(beamng)
    beamng.load_scenario(scenario)
    print("Scenario Loaded")

    # If user asked for VR, activate it
    if activate_vr_bool:
        activate_vr_mode()

    beamng.start_scenario()

    #################################
    # START VIDEO RECORDING CHANGES #
    #################################

    # Wait a moment for the window to appear
    time.sleep(2)
    print("Attempting to focus the BeamNG.tech Vulkan window using pywinauto...")

    try:
        # Attempt to find the BeamNG.tech window with "Vulkan" in the title
        beamng_window = Desktop(backend="win32").window(title_re="BeamNG.tech.*Vulkan")
        beamng_window.set_focus()
        print("Successfully focused BeamNG.tech window.")
    except:
        print("Could not find the BeamNG.tech Vulkan window. Check the title or if the game is open.")

    time.sleep(1)

    # Press Win + Alt + R to start recording
    print("Attempting to start Windows Game Bar recording...")
    pyautogui.keyDown('winleft')
    pyautogui.keyDown('alt')
    pyautogui.press('r')
    pyautogui.keyUp('alt')
    pyautogui.keyUp('winleft')
    time.sleep(1)  # Short delay

    #################################
    # END VIDEO RECORDING CHANGES   #
    #################################

    # Configure the preceding vehicle's AI
    preceding_vehicle.ai_set_mode('span')
    preceding_vehicle.ai_drive_in_lane(True)
    preceding_vehicle.ai_set_aggression(0.0)

    # Set ego vehicle to real automatic
    ego_vehicle.set_shift_mode("arcade")

    # Create interpolation array for the preceding vehicle speed
    t_original = np.linspace(0, (N - 1)*0.1, N)
    t_simulation = np.arange(0, max_simulation_time, dt)

    f_interp = interp1d(t_original, target_speed_list, kind='linear', fill_value='extrapolate')
    target_speed_list_interp = f_interp(t_simulation)

    total_steps = len(t_simulation)

    # Prepare data storage
    speed_pre = []
    speed_ego = []
    position_ego = []
    position_pre = []
    time_list = []
    acceleration_ego = []
    headway = []
    deltav_list = []
    throttle_list = []
    brake_list = []
    hCCC_list = []
    vehicle_throttle_list = []
    vehicle_brake_list = [] 
    engine_throttle_list = []
    engine_brake_list = []

    # Instantiate the chosen controller if not manual
    controller = None
    if control_method == "manual":
        controller = hCCC(ego_vehicle, preceding_vehicle, dt=dt)

    # Attach an electrics sensor to retrieve throttle/brake inputs
    electrics_sensor = Electrics()
    ego_vehicle.attach_sensor('electrics', electrics_sensor)

    raw_throttle = 0.0
    raw_brake = 0.0
    raw_steer = 0.0

    def check_controller_inputs():
        pygame.event.pump()
        
        if vjoy_device is not None:
            THROTTLE_AXIS = 1
            BRAKE_AXIS = 2
            STEER_AXIS = 0

            raw_throttle = vjoy_device.get_axis(THROTTLE_AXIS)
            raw_brake = vjoy_device.get_axis(BRAKE_AXIS)
            raw_steer = vjoy_device.get_axis(STEER_AXIS)
            
            raw_throttle = (raw_throttle + 1.0) / 2.0
            raw_brake    = (raw_brake + 1.0) / 2.0

        return raw_throttle, raw_brake, raw_steer

    print("Starting simulation loop...")
    for k in range(total_steps):
        simulation_time = k * dt

        # Advance simulation by one step
        beamng.step(1)

        # Set the speed for the preceding vehicle
        target_speed_pre = target_speed_list_interp[k]
        preceding_vehicle.ai_set_speed(target_speed_pre, mode='set')

        ego_throttle_input, ego_brake_input, ego_steer_input = check_controller_inputs()

        throttle_list.append(ego_throttle_input)
        brake_list.append(ego_brake_input)

        ego_input = ego_throttle_input - ego_brake_input
        # If a non-manual controller is active, let it do its logic
        if controller is not None:
            if isinstance(controller, hCCC):
                hCCC_acc = controller.run_step()
                hCCC_list.append(hCCC_acc)
                vehicle_cmd = ego_input + hCCC_acc
                if vehicle_cmd >= 0.0:
                    throttle_vehicle = vehicle_cmd
                    brake_vehicle = 0.0
                else:
                    throttle_vehicle = 0.0
                    brake_vehicle = -vehicle_cmd
                throttle_vehicle = float(np.clip(throttle_vehicle, 0.0, 1.0))
                brake_vehicle = float(np.clip(brake_vehicle, 0.0, 1.0))
                ego_vehicle.control(throttle=ego_throttle_input, brake=ego_brake_input, steering=ego_steer_input)
                vehicle_throttle_list.append(throttle_vehicle)
                vehicle_brake_list.append(brake_vehicle)

        # Poll sensors
        preceding_vehicle.poll_sensors()
        ego_vehicle.poll_sensors()

        electrics_data = ego_vehicle.sensors['electrics'].data

        # Fetch driver throttle and brake inputs
        engine_throttle = electrics_data.get('throttle_input', 0.0)
        engine_brake = electrics_data.get('brake_input', 0.0)

        engine_throttle_list.append(engine_throttle)
        engine_brake_list.append(engine_brake)
        
        # Collect data
        state_pre = preceding_vehicle.state
        speed_pre.append(math.sqrt(sum(v ** 2 for v in state_pre['vel'])))

        state_ego = ego_vehicle.state
        speed_ego.append(math.sqrt(sum(v ** 2 for v in state_ego['vel'])))

        deltav_list.append(math.sqrt(sum(v ** 2 for v in state_ego['vel'])) - math.sqrt(sum(v ** 2 for v in state_pre['vel'])))

        if k > 0:
            accel_ego_current = (speed_ego[-1] - speed_ego[-2]) / dt
        else:
            accel_ego_current = 0
        acceleration_ego.append(accel_ego_current)

        position_ego_current = state_ego['pos']
        position_pre_current = state_pre['pos']
        position_ego.append(position_ego_current)
        position_pre.append(position_pre_current)

        current_headway = np.linalg.norm(
            np.array(position_pre_current) - np.array(position_ego_current)
        ) - 4.5
        headway.append(current_headway)

        time_list.append(simulation_time)

        if k % int(1 / dt) == 0:
            print(f"Simulation time: {simulation_time:.1f} seconds")

    print("Simulation completed.")

    #################################
    # STOP VIDEO RECORDING CHANGES  #
    #################################
    print("Attempting to stop Windows Game Bar recording...")
    pyautogui.keyDown('winleft')
    pyautogui.keyDown('alt')
    pyautogui.press('r')
    pyautogui.keyUp('alt')
    pyautogui.keyUp('winleft')
    time.sleep(2)  # Give Windows time to finalize the file

    beamng.close()

    # Prepare output directories
    graph_dir = f'graphs/{control_method}'
    data_dir = f'data/{control_method}'

    # NEW VIDEO DIRECTORY
    video_dir = f'video/{control_method}'

    create_directory(graph_dir)
    create_directory(data_dir)
    create_directory(video_dir)  # Create a video folder

    # Determine next test number for file naming
    test_counter = get_next_test_number(data_dir)
    print(f"Test number: {test_counter}")

    # Construct filenames
    csv_filename = (
        f'{data_dir}/Test{test_counter}.vehicle.{ego_vehicle_type}_ICE'
        f'.scn{SCN}.{control_method}.csv'
    )
    graph_filename = (
        f'{graph_dir}/Test{test_counter}.vehicle.{ego_vehicle_type}_ICE'
        f'.scn{SCN}.{control_method}.png'
    )

    # Save data to CSV
    data_dict = {
        'time[s]': time_list,
        'position_ego[m]': position_ego,
        'position_pre[m]': position_pre,
        'speed_ego[m/s]': speed_ego,
        'speed_pre[m/s]': speed_pre,
        'acceleration_ego[m/s2]': acceleration_ego,
        'headway[m]': headway,
        'deltav[m/s]': deltav_list,
        'driver_throttle': throttle_list,
        'driver_brake': brake_list,
        'hCCC': hCCC_list,
        'vehicle_throttle': vehicle_throttle_list,
        'vehicle_brake': vehicle_brake_list,
        'engine_throttle': engine_throttle_list,
        'engine_brake': engine_brake_list        
    }

    df_out = pd.DataFrame(data_dict)
    df_out.to_csv(csv_filename, index=False)
    print(f"Results saved to {csv_filename}")

    # Plot results
    plt.figure(figsize=(12, 6))
    plt.plot(time_list, speed_pre, label='Preceding Vehicle')
    plt.plot(time_list, speed_ego, label='Ego Vehicle')
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.title('Speed vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(graph_filename)
    plt.show()
    print(f"Graph saved to {graph_filename}")

    print("Test completed successfully.")

    #################################
    # MOVE VIDEO FILE (OPTIONAL)
    #################################
    captures_folder = os.path.join(os.path.expanduser('~'), 'Videos', 'Captures')
    list_of_mp4s = glob.glob(os.path.join(captures_folder, '*.mp4'))
    if list_of_mp4s:
        latest_mp4 = max(list_of_mp4s, key=os.path.getctime)
        new_file_path = os.path.join(video_dir, f"Test{test_counter}.mp4")
        print(f"Moving {latest_mp4} to {new_file_path}")
        shutil.move(latest_mp4, new_file_path)
    else:
        print("No new recording found in Captures folder. Video file not moved.")

if __name__ == '__main__':
    main()

    
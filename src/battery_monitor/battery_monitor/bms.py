import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState 
from clearpath_platform_msgs.msg import Power

import numpy as np
import time

from battery_monitor.bms_utils import Params, StateParams, State,  BatteryModelState
from battery_monitor.battery_model import BatterySoCModel, BatteryParamModel
from battery_monitor.ekf import EkfSoC, EkfParams
from battery_monitor.coulomb_counting import CoulombCnt


class BMS(Node):

    def __init__(self):
        super().__init__('Battery_monitoring_system')
        
        self.declare_parameter("battery_general_params")
        self.declare_parameter("battery_state_params")
        
        self.battery_general_params_path = self.get_parameter('battery_general_params').get_parameter_value().string_value
        self.battery_state_params_path   = self.get_parameter('battery_state_params').get_parameter_value().string_value

        # Load battery params form file
        self.battery_params = Params()
        if not self.battery_params.init_from_file(self.battery_general_params_path):
            raise Exception(f"""
                            Exception loading params form file '{self.battery_general_params_path}' 
                            File does not exist of have bad structure.
                            """) 
        
        # Load last battery state form file 
        self.battery_state = StateParams(24 * 3600)  # Threshold = 24h

        # Check if battery state is still valid
        model_state = BatteryModelState()
        if self.battery_state.check_file(self.battery_state_params_path) != StateParams.FileState.EXISTS_VALID:
            self.battery_state.init_from_file(self.battery_state_params_path)
            model_state.from_array(np.array([[self.battery_state.R1], 
                                             [self.battery_state.C1],
                                             [self.battery_state.R], 
                                             [self.battery_state.Q]]))

        # Init EKF for RQ
        self.ekf_param = EkfParams(BatteryParamModel(self.battery_params))
        self.ekf_param.init(model_state)

        # Init EKF for Soc
        self.ekf_soc = EkfSoC(BatterySoCModel(self.battery_params))
        self.ekf_soc.init(State())
        
        
        # Collect sensor data info buffers 
        # (sliding window approach for smoothness)
        self.voltage_buffer = []
        self.current_buffer = []

        self.predicted_voltage_buffer = []
        
        # Subscribe to battery state message
        self.subscription = self.create_subscription(
            Power,
            '/a200_1056/platform/mcu/status/power',
            self.message_handler,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        # Bms state publisher
        self.ekf_publisher = self.create_publisher(BatteryState, 'kalman_bms_state', qos_profile_sensor_data)
        self.cc_publisher = self.create_publisher(BatteryState, 'coulomb_bms_state', qos_profile_sensor_data)
        self.predicted_publisher = self.create_publisher(BatteryState, 'predicted_voltage', qos_profile_sensor_data)

        self.last_message_timestamp = time.time()

        # Init Coulomb counting method after some time
        # It's inited after delay to give EKF some time for correst init step estimation
        self.run_cc = False
        self.cc_timer = self.create_timer(10 * 60, self.timer_cb)

        # Create timer to save battery state data every hour
        self.create_timer(60 * 60, self.save_battery_state)

    def timer_cb(self):
        """
        Timer claback to init Coulomb counting method
        """
        
        self.run_cc = True
        self.cc = CoulombCnt(self.ekf_soc.state.SoC, self.ekf_param.state.Q)

        # After first initialization set timer delay to 1h  
        self.cc_timer.destroy()
        self.cc_timer = self.create_timer(60 * 60, self.timer_cb)


    def step_ekf_param_update(self):
        """
        EKF for battery model parameters update step
        """

        return self.ekf_param.state_update()

    def step_ekf_param_innovate(self, current: float, voltage: float, state: State, dt: float):
        """
        EKF for battery model parameters innovate step
        """

        ret = self.ekf_param.state_innovation(current, voltage, state, dt)

        self.get_logger().info(f"      < V: {voltage:.5f} > < I: {current:.5f} >")
        self.get_logger().info(f"R1 estimate: {self.ekf_param.state.R1:.5f}")
        self.get_logger().info(f"C1 estimate: {self.ekf_param.state.C1:.5f}")
        self.get_logger().info(f"R  estimate: {self.ekf_param.state.R:.5f}")
        self.get_logger().info(f"Q  estimate: {self.ekf_param.state.Q:.5f}")
        self.get_logger().info(f"------------------------------------------------")

        return ret


    def step_ekf_soc_update(self, current: float, params: BatteryModelState, dt: float):
        """
        EKF for SoC estimation update step
        """

        return self.ekf_soc.state_update(current, params, dt)


    def step_ekf_soc_innovate(self, current, voltage, params: BatteryModelState, dt: float):
        """
        EKF for SoC estimation innovate step
        """

        ret = self.ekf_soc.state_innovation(current, voltage, params, dt)

        if len(self.predicted_voltage_buffer) > 100:
            self.predicted_voltage_buffer = self.predicted_voltage_buffer[1:]
        self.predicted_voltage_buffer.append(ret)

        self.get_logger().info(f"      < V: {voltage:.5f} > < I: {current:.5f} >")
        self.get_logger().info(f"SoC estimate: {self.ekf_soc.state.SoC:.5f}")
        self.get_logger().info(f"V1  estimate: {self.ekf_soc.state.V1:.5f}")
        self.get_logger().info(f"------------------------------------------------")

        return ret
    

    def step_cc(self, current, dt):

        self.cc.step(-current, dt)

    def send_cc_message(self):
        
        resp_msg = BatteryState()
        resp_msg.percentage  = self.cc.last_SoC
        self.cc_publisher.publish(resp_msg)


    def message_handler(self, msg: Power) -> None:
        """
        Handle battery state message

        :params msg: battery state mesage
        """

        dt = time.time() - self.last_message_timestamp

        self.collect_data(msg)

        # Wait for buffers to fill
        if len(self.current_buffer) < 100:
            return

        current = sum(self.current_buffer) / len(self.current_buffer)
        voltage = sum(self.voltage_buffer) / len(self.voltage_buffer)

        # Make EKF prediction steps

        new_params = self.step_ekf_param_update()
        new_state  = self.step_ekf_soc_update(current, new_params, dt)
        
        self.step_ekf_param_innovate(current, voltage, new_state, dt)
        self.step_ekf_soc_innovate(current, voltage, new_params, dt)
        
        # Update time and publish BMS state
        self.last_message_timestamp = time.time()
        self.send_message(current, voltage)

        if self.run_cc:
            self.step_cc(current, dt)
            self.send_cc_message()


    def collect_data(self, msg: Power) -> None:
        """
        Collect voltage and current data into buffer

        :params msg: battery state mesage
        """
        
        if len(self.current_buffer) > 100:
            self.current_buffer = self.current_buffer[1:]

        if len(self.voltage_buffer) > 100:
            self.voltage_buffer = self.voltage_buffer[1:]

        current = msg.measured_currents[Power.A200_LEFT_DRIVER_CURRENT]
        current += msg.measured_currents[Power.A200_RIGHT_DRIVER_CURRENT]
        current += msg.measured_currents[Power.A200_MCU_AND_USER_PORT_CURRENT]

        voltage = msg.measured_voltages[Power.A200_BATTERY_VOLTAGE]

        self.current_buffer.append(current)
        self.voltage_buffer.append(voltage)


    def send_message(self, current, voltage) -> None:
        """
        Publish bms state into topic
        """

        resp_msg = BatteryState()
        resp_msg.capacity    = self.ekf_param.state.Q
        resp_msg.charge      = self.ekf_soc.state.SoC * self.ekf_param.state.Q
        resp_msg.percentage  = self.ekf_soc.state.SoC
        resp_msg.voltage  = voltage
        resp_msg.current  = current
        resp_msg.power_supply_health = int((self.ekf_param.state.Q / self.battery_params.Q0) * 100)
        resp_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self.ekf_publisher.publish(resp_msg)

        predictred_voltage = BatteryState()
        predictred_voltage.voltage  = sum(self.predicted_voltage_buffer) / len(self.predicted_voltage_buffer)
        self.predicted_publisher.publish(predictred_voltage)

    
    def save_battery_state(self):
        """
        Writes battery state to file
        """

        self.battery_state.R1 = self.ekf_param.state.R1
        self.battery_state.C1 = self.ekf_param.state.C1
        self.battery_state.R  = self.ekf_param.state.R
        self.battery_state.Q  = self.ekf_param.state.Q

        self.battery_state.write_to_file(self.battery_state_params_path)




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = BMS()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
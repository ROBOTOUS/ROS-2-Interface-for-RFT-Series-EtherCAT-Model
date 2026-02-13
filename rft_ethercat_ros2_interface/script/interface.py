#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import ctypes
import struct
import threading
import time
from collections import namedtuple
import os
import pysoem
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16, Int16MultiArray, Float64
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool
from example_interfaces.srv import AddTwoInts

class InterfaceError(Exception):
    def __init__(self, message):
        super().__init__(message)
        self.message = message

class RFT(Node):
    
    @staticmethod
    def load_config_from_txt():
        """Reads interface name and Hz value from the config file."""
        base_path = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(base_path, "config.txt")
        
        # Default settings
        conf = {"interface": "eth0", "hz": 1000.0}

        if not os.path.exists(config_path):
            try:
                with open(config_path, "w") as f:
                    f.write("Interface: eth0\n")
                    f.write("OutputRate_Hz: 1000\n")
            except Exception as e:
                print(f"File create error: {e}")
            return conf

        try:
            with open(config_path, "r") as f:
                for line in f:
                    if "Interface:" in line:
                        conf["interface"] = line.split(":")[1].strip()
                    elif "OutputRate_Hz:" in line:
                        conf["hz"] = float(line.split(":")[1].strip())
        except Exception as e:
            print(f"Config read error: {e}")
            
        return conf

    def __init__(self, ifname, run_cycle):
        super().__init__('RFT')
        self._ifname = ifname
        
        # EtherCAT communication cycle setup
        self.declare_parameter('run_cycle', run_cycle)
        self.run_cycle = self.get_parameter('run_cycle').get_parameter_value().double_value

        # Logging initialization info
        self.get_logger().info(f"Setting Output Rate: {int(1.0/self.run_cycle)}Hz ({self.run_cycle*1000:.1f}ms)")
        self.get_logger().info(f"Initializing node with interface: {self._ifname}")
        
        self._pd_thread_stop_event = threading.Event()
        self._actual_wkc = 0
        self._master = None

        self._expected_slave_layout = { 0: self.tfsensor_config }
        self._run_initialized = False
        self._ft_stop_flag = True  # FT continuous/stop flag

        # ROS2 publishers
       
        self.FT_data_pub = self.create_publisher(WrenchStamped, "/RFT/FT_data", 1)

        # ROS2 services
        self.get_rate_srv = self.create_service(SetBool, 'GET_OUTPUT_RATE', self.get_output_rate_callback)
        self.get_model_srv = self.create_service(SetBool, 'GET_MODEL_NAME', self.get_model_callback)
        self.get_serial_srv = self.create_service(SetBool, 'GET_SERIAL_NUMBER', self.get_serial_callback)
        self.get_fw_srv = self.create_service(SetBool, 'GET_FIRMWARE_VERSION', self.get_fw_callback)
        self.get_overload_srv = self.create_service(SetBool, 'GET_OVERLOAD', self.get_overload_service_callback)
        self.bias_srv = self.create_service(SetBool, 'SET_BIAS', self.set_bias_callback)
        self.ft_control_srv = self.create_service(SetBool, 'FT_CONTINOUS', self.ft_control_callback)

        # LPF service setup
        self._cut_off_table = [0, 500, 300, 200, 150, 100, 50, 40, 30, 20, 10, 5, 3, 2, 1]
        self._lpf_value = 0
        self.lpf_srv = self.create_service(AddTwoInts, 'SET_FILTER', self.lpf_service_callback)
        self.get_lpf_srv = self.create_service(SetBool, 'GET_FILTER', self.get_lpf_service_callback)
        self._current_filter_hz = 0

        self.bias_on = False

        # Process data thread
        self._comm_thread = threading.Thread(target=self.processdata_loop, daemon=True)

        # Timer setup
        self.timer = self.create_timer(self.run_cycle, self.timer_callback)

    # --------------------- Master & Slave Initialization ---------------------

    def initialize_master(self):
        self.get_logger().info("Opening master on interface: " + self._ifname)
        self._master.open(self._ifname)
        
       
        cnt = self._master.config_init()
        if cnt <= 0:
            self._master.close()
            raise InterfaceError('No slave found. Check cable or power!')

        self.get_logger().info(f"Found {cnt} slaves. Mapping config...")

       
        for i, slave in enumerate(self._master.slaves):
            
            if i == 0:
                slave.config_func = self.tfsensor_config
            slave.is_lost = False

        self.get_logger().info("Slave configuration skipped check and set to default.")

       
        self._master.config_map()
        self._master.config_dc()
        self._master.state = pysoem.OP_STATE
        self._master.write_state()
        self._master.state_check(pysoem.OP_STATE, 50000)
        self.get_logger().info("Master state: OPERATIONAL")
        time.sleep(0.5)
        
    def processdata_loop(self):
        """Cyclic thread for sending/receiving process data (PDO)."""
        while not self._pd_thread_stop_event.is_set():
            if self._master:
                self._master.send_processdata()
                self._actual_wkc = self._master.receive_processdata(1000)
            time.sleep(self.run_cycle)

    def tfsensor_config(self, slave_pos):
        """Initial SDO configuration for the sensor."""
        slave = self._master.slaves[slave_pos]

   
    def initialize_sensor(self):
        """Runs sensor-specific initialization steps via SDO."""
        slave = self._master.slaves[0]
        try:
            # 1. Read bias value
            self.get_logger().info("Reading Bias value...")
            raw_bias = slave.sdo_read(0x2001, 1)
            if raw_bias:
                bias_val = int.from_bytes(raw_bias, byteorder='little', signed=False)
            time.sleep(0.01) 

            # 3. Set default bias state
            self.set_bias_off()
            self.bias_on = False
            self.get_logger().info("Bias set to OFF and Sensor Initialized")
            
        except Exception as e:
            self.get_logger().warn(f"Sensor init warning (SDO access failed): {e}")

    # --------------------- Sensor Data Publishing (PDO Mode) ---------------------

    def publish_sensor_data(self):
        """Reads real-time data from PDO buffer and publishes to ROS2 topics."""
        if self._ft_stop_flag or not self._master:
            return

        slave = self._master.slaves[0]
        buffer = slave.input  # Accessing the mapped input memory directly

        try:
            # Parsing based on MFC memory layout:
            # <ffffff : Force/Torque (6 floats = 24 bytes)
            # H       : Status (1 unsigned short = 2 bytes)
            # xx      : Padding (2 bytes dummy)
            # f       : Temperature (1 float = 4 bytes)
            if len(buffer) >= 32:
                unpacked = struct.unpack('<ffffffH xxf', buffer[:32])
                fx, fy, fz, tx, ty, tz, status, temp = unpacked

                # Publish Force/Torque
                ft_msg = WrenchStamped()
                ft_msg.header.stamp = self.get_clock().now().to_msg()
                ft_msg.header.frame_id = 'RFT'
                ft_msg.wrench.force.x, ft_msg.wrench.force.y, ft_msg.wrench.force.z = fx, fy, fz
                ft_msg.wrench.torque.x, ft_msg.wrench.torque.y, ft_msg.wrench.torque.z = tx, ty, tz
                self.FT_data_pub.publish(ft_msg)

                # Publish Temperature
                self.Temperature_pub.publish(Float64(data=float(temp)))

                # Publish Status (mapping bits to wrench values)
                status_msg = WrenchStamped()
                status_msg.header = ft_msg.header
                status_msg.wrench.force.x = float((status >> 0) & 1)
                self.FTS_status_pub.publish(status_msg)
        except Exception:
            pass

    # --------------------- Timer Callback ---------------------

    def timer_callback(self):
        try:
            if not self._run_initialized:
                self._master = pysoem.Master()
                self.initialize_master()
                if not self._comm_thread.is_alive():
                    self._comm_thread.start()
                self.initialize_sensor()
                self._run_initialized = True
                self.get_logger().info("Initialization completed")
            else:
                self.publish_sensor_data()
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

    # --------------------- Node Destruction ---------------------

    def destroy_node(self):
        self._pd_thread_stop_event.set()
        if self._comm_thread.is_alive():
            self._comm_thread.join()
        if self._master:
            self._master.close()
        super().destroy_node()

    # --------------------- Services ---------------------

    def set_bias_callback(self, request, response):
        try:
            if not self._run_initialized:
                raise RuntimeError("Device not initialized")
            if request.data:
                self.set_bias_on()
            else:
                self.set_bias_off()
            
            # Update local bias state via SDO
            raw = self._master.slaves[0].sdo_read(0x2001, 1)
            self.bias_on = bool(int.from_bytes(raw, byteorder='little'))
            
            response.success = True
            response.message = f"Bias state: {'ON' if self.bias_on else 'OFF'}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def set_bias_on(self):
        self._master.slaves[0].sdo_write(0x2001, 1, struct.pack('<H', 1))
        self.get_logger().info("Bias ON command sent")

    def set_bias_off(self):
        self._master.slaves[0].sdo_write(0x2001, 1, struct.pack('<H', 0))
        self.get_logger().info("Bias OFF command sent")

    def ft_control_callback(self, request, response):
        self._ft_stop_flag = not request.data
        response.success = True
        response.message = "FT output started" if request.data else "FT output stopped"
        return response

    def lpf_service_callback(self, request, response):
        sub_type = request.a
        if sub_type < 0 or sub_type >= len(self._cut_off_table):
            self.get_logger().error(f"Invalid LPF sub_type: {sub_type}")
            response.sum = -1
            return response
        
        value = self._cut_off_table[sub_type]
        self._master.slaves[0].sdo_write(0x2001, 2, struct.pack('<H', value))
        self.get_logger().info(f"LPF set to cut_off[{sub_type}] = {value}Hz")
        self._lpf_value = sub_type
        response.sum = value
        return response
    
    def get_lpf_service_callback(self, request, response):
      
        try:
           
            if not self._run_initialized or self._master is None:
                self.get_logger().error("Master not initialized")
                response.success = False
                response.message = "Master not initialized"
                return response

            slave = self._master.slaves[0]
           
            raw_data = slave.sdo_read(0x2001, 2)
            
            if raw_data and len(raw_data) >= 2:
                rcvd_cut_off = struct.unpack('<H', raw_data)[0]
                self.get_logger().info(f"Filter read success: {rcvd_cut_off}Hz")
                
                response.success = True
                response.message = str(rcvd_cut_off)
            else:
                self.get_logger().error("Filter value read failed (No data)")
                response.success = False
                response.message = "Failed to read SDO data"
        
       
        except Exception as e:
            self.get_logger().error(f"Communication failure: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            
        return response
    
    def get_model_callback(self, request, response):
        try:
            if not self._run_initialized or self._master is None:
                self.get_logger().error("Master not initialized")
                response.success = False
                response.message = "Master not initialized"
                return response

            if request.data: 
                slave = self._master.slaves[0]
               
                raw_data = slave.sdo_read(0x2000, 1)
                
                if raw_data:
                    model_name = raw_data.decode('utf-8').rstrip('\x00')
                    self.get_logger().info(f"Model Name read success: {model_name}")
                    response.success = True
                    response.message = model_name
                else:
                    response.success = False
                    response.message = "Failed to read SDO (0x2000:1)"
            else:
                response.success = False
                response.message = "Request data is False. Set to True to read."

        except Exception as e:
            self.get_logger().error(f"Model Name service failure: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            
        return response
    
    def get_serial_callback(self, request, response):
        try:
            if not self._run_initialized or self._master is None:
                self.get_logger().error("Master not initialized")
                response.success = False
                response.message = "Master not initialized"
                return response

            if request.data:
                slave = self._master.slaves[0]
                raw_data = slave.sdo_read(0x2000, 2)
                
                if raw_data:
                    serial_num = raw_data.decode('utf-8').rstrip('\x00')
                    self.get_logger().info(f"Serial Number read success: {serial_num}")
                    response.success = True
                    response.message = serial_num
                else:
                    response.success = False
                    response.message = "Failed to read SDO (0x2000:2)"
            else:
                response.success = False
                response.message = "Request data is False."

        except Exception as e:
            self.get_logger().error(f"Serial Number service failure: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            
        return response
    
    def get_output_rate_callback(self, request, response):
        try:
            if request.data:  
                
                current_hz = 1.0 / self.run_cycle
                
                response.success = True
              
                response.message = f"Current Output Rate: {int(current_hz)}Hz"
                self.get_logger().info(f"Reported current rate: {int(current_hz)}Hz")
            else:
                response.success = False
                response.message = "Request data is False. Set to True to get rate."
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            
        return response

    def get_fw_callback(self, request, response):
        try:
            if not self._run_initialized or self._master is None:
                self.get_logger().error("Master not initialized")
                response.success = False
                response.message = "Master not initialized"
                return response

            if request.data:
                slave = self._master.slaves[0]
                raw_data = slave.sdo_read(0x2000, 3)
                
                if raw_data:
                    fw_version = raw_data.decode('utf-8').rstrip('\x00')
                    self.get_logger().info(f"Firmware Version read success: {fw_version}")
                    response.success = True
                    response.message = fw_version
                else:
                    response.success = False
                    response.message = "Failed to read SDO (0x2000:3)"
            else:
                response.success = False
                response.message = "Request data is False."

        except Exception as e:
            self.get_logger().error(f"Firmware Version service failure: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            
        return response
    

    def get_overload_service_callback(self, request, response):
        
        try:
            if not self._run_initialized or self._master is None:
                response.success = False
                response.message = "Master not initialized"
                return response

            slave = self._master.slaves[0]
            counts = []
            
           
            for sub_idx in range(1, 7):
                raw_data = slave.sdo_read(0x2002, sub_idx)
                if raw_data:
                    
                    cnt = struct.unpack('<B', raw_data[0:1])[0]
                    counts.append(cnt)
                else:
                    counts.append(0)

            self.get_logger().info(f"Overload counts read: {counts}")
            
            response.success = True
           
            response.message = f"Fx:{counts[0]}, Fy:{counts[1]}, Fz:{counts[2]}, Tx:{counts[3]}, Ty:{counts[4]}, Tz:{counts[5]}"

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            
        return response
        
        

# --------------------- Main ---------------------

def main(args=None):
    rclpy.init(args=args)
    
    # 1. Load config from file
    config_data = RFT.load_config_from_txt()
    
    # 2. Determine target interface
    if len(sys.argv) > 1:
        target_ifname = sys.argv[1]
    else:
        target_ifname = config_data["interface"]
    
    # 3. Determine cycle time
    target_run_cycle = 1.0 / config_data["hz"]

    # 4. Create and spin node
    node = RFT(target_ifname, target_run_cycle)
    print(f"--- Starting RFT Interface on {target_ifname} ---")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n--- Shutting down RFT Interface ---")
    except Exception as e:
        print(f"Runtime Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

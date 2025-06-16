#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json

# Import standard ROS 2 message types
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Point

# --- Configuration ---
MOONRAKER_HOST = "localhost"
MOONRAKER_PORT = 7125

class KlipperBridgeNode(Node):
    """A ROS 2 node to bridge Klipper data from Moonraker to ROS topics."""
    
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('klipper_bridge_node')
        self.get_logger().info("Klipper-ROS 2 Bridge started.")

        # --- ROS 2 Publishers ---
        # Create publishers using the ROS 2 'create_publisher' method
        self.pub_extruder_temp_actual = self.create_publisher(Float32, '/klipper/extruder/temperature/actual', 10)
        self.pub_extruder_temp_target = self.create_publisher(Float32, '/klipper/extruder/temperature/target', 10)
        self.pub_bed_temp_actual = self.create_publisher(Float32, '/klipper/heater_bed/temperature/actual', 10)
        self.pub_bed_temp_target = self.create_publisher(Float32, '/klipper/heater_bed/temperature/target', 10)
        self.pub_toolhead_pos = self.create_publisher(Point, '/klipper/toolhead/position', 10)
        self.pub_print_state = self.create_publisher(String, '/klipper/print_stats/state', 10)
        self.pub_print_progress = self.create_publisher(Float32, '/klipper/print_stats/progress', 10)
    
    async def connect_and_publish(self):
        """The main async loop for connecting to Moonraker and publishing data."""
        uri = f"ws://{MOONRAKER_HOST}:{MOONRAKER_PORT}/websocket"
        
        # The rclpy.ok() check is the ROS 2 equivalent of !rospy.is_shutdown()
        while rclpy.ok():
            try:
                async with websockets.connect(uri) as websocket:
                    self.get_logger().info(f"Successfully connected to Moonraker at {uri}")

                    # Subscribe to the printer objects
                    subscribe_message = {
                        "jsonrpc": "2.0",
                        "method": "printer.objects.subscribe",
                        "params": {
                            "objects": {
                                "extruder": ["temperature", "target"],
                                "heater_bed": ["temperature", "target"],
                                "toolhead": ["position"],
                                "print_stats": ["state", "progress"]
                            }
                        },
                        "id": 1
                    }
                    await websocket.send(json.dumps(subscribe_message))

                    # Listen for messages from Moonraker
                    while rclpy.ok():
                        message_str = await websocket.recv()
                        data = json.loads(message_str)

                        if data.get("method") == "notify_status_update":
                            self.process_status_update(data["params"][0])
                        
            except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError) as e:
                self.get_logger().warn(f"WebSocket connection error: {e}. Retrying in 5 seconds...")
                await asyncio.sleep(5)
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred: {e}")
                break

    def process_status_update(self, status):
        """Processes the dictionary of status updates and publishes to topics."""
        for key, value in status.items():
            if key == "extruder":
                self.pub_extruder_temp_actual.publish(Float32(data=value.get("temperature", 0.0)))
                self.pub_extruder_temp_target.publish(Float32(data=value.get("target", 0.0)))
            
            elif key == "heater_bed":
                self.pub_bed_temp_actual.publish(Float32(data=value.get("temperature", 0.0)))
                self.pub_bed_temp_target.publish(Float32(data=value.get("target", 0.0)))

            elif key == "toolhead":
                pos = value.get("position", [0.000, 0.000, 0.000])
                toolhead_point = Point(x=pos[0], y=pos[1], z=pos[2])
                self.pub_toolhead_pos.publish(toolhead_point)

            elif key == "print_stats":
                self.pub_print_state.publish(String(data=value.get("state", "standby")))
                self.pub_print_progress.publish(Float32(data=value.get("progress", 0.0)))

def main(args=None):
    rclpy.init(args=args)
    node = KlipperBridgeNode()
    
    try:
        asyncio.run(node.connect_and_publish())
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and shutdown the node
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

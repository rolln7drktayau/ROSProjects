import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
import time
import statistics
from datetime import datetime

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.timestamps = []
        self.message_count = 0
        self.total_messages = 1000
        self.start_time = time.time()
        self.get_logger().info("Listener node started - Waiting for messages...")

    def listener_callback(self, msg):
        try:
            current_time = time.time()
            self.timestamps.append(current_time)
            self.message_count += 1
            
            # Affiche un message toutes les 100 réceptions
            if self.message_count % 100 == 0:
                self.get_logger().info(f"Received {self.message_count} messages (Latest: {msg.data})")
            
            if self.message_count >= self.total_messages:
                self.calculate_stats()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")

    def calculate_stats(self):
        try:
            if len(self.timestamps) < 2:
                self.get_logger().error("Not enough messages received for statistics")
                return
                
            durations = [self.timestamps[i] - self.timestamps[i-1] 
                        for i in range(1, len(self.timestamps))]
            
            mean_duration = statistics.mean(durations)
            variance = statistics.variance(durations) if len(durations) > 1 else 0
            
            print(f"\n--- Listener Statistics ---")
            print(f"First message received at: {datetime.fromtimestamp(self.timestamps[0])}")
            print(f"Last message received at: {datetime.fromtimestamp(self.timestamps[-1])}")
            print(f"Total messages received: {self.message_count}")
            print(f"Total time: {self.timestamps[-1] - self.timestamps[0]:.4f} seconds")
            print(f"Mean duration between messages: {mean_duration:.6f} seconds")
            print(f"Variance: {variance:.6f}")
            print(f"Frequency: {1/mean_duration:.2f} Hz")
            print(f"System time elapsed: {time.time() - self.start_time:.2f} seconds")
            
        except Exception as e:
            self.get_logger().error(f"Error calculating stats: {str(e)}")

def main(args=None):
    try:
        rclpy.init(args=args)
        listener = Listener()
        rclpy.spin(listener)
        listener.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\nListener stopped by user")
    except Exception as e:
        print(f"Error in listener: {str(e)}")

if __name__ == '__main__':
    main()
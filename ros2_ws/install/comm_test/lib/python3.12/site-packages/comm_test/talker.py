import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
import time
import statistics
from datetime import datetime

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.1  # Contrôle la vitesse (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.start_time = time.time()
        self.timestamps = []
        self.total_messages = 1000
        self.get_logger().info("Talker node started - Beginning transmission...")

    def timer_callback(self):
        try:
            msg = String()
            msg.data = f"Message {self.i:04d}"  # Format sur 4 chiffres
            publish_time = time.time()
            
            self.publisher_.publish(msg)
            self.timestamps.append(publish_time)
            
            # Affiche un message toutes les 50 publications
            if self.i % 50 == 0:
                self.get_logger().info(f"Published {self.i:04d}/{self.total_messages} - '{msg.data}'")
            
            self.i += 1
            
            if self.i >= self.total_messages:
                self.calculate_stats()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Publishing error: {str(e)}")

    def calculate_stats(self):
        try:
            durations = []
            for i in range(1, len(self.timestamps)):
                durations.append(self.timestamps[i] - self.timestamps[i-1])
            
            mean_duration = statistics.mean(durations)
            variance = statistics.variance(durations) if len(durations) > 1 else 0
            
            print(f"\n--- Talker Final Statistics ---")
            print(f"First message sent at: {datetime.fromtimestamp(self.timestamps[0])}")
            print(f"Last message sent at: {datetime.fromtimestamp(self.timestamps[-1])}")
            print(f"Total messages sent: {self.total_messages}")
            print(f"Total transmission time: {self.timestamps[-1] - self.timestamps[0]:.4f} seconds")
            print(f"Mean duration between sends: {mean_duration:.6f} seconds")
            print(f"Variance: {variance:.6f}")
            print(f"Actual frequency: {1/mean_duration:.2f} Hz")
            print(f"Target frequency: {1/0.1:.2f} Hz")
            print(f"System time elapsed: {time.time() - self.start_time:.2f} seconds")
            
        except Exception as e:
            self.get_logger().error(f"Stats calculation error: {str(e)}")

def main(args=None):
    try:
        rclpy.init(args=args)
        talker = Talker()
        rclpy.spin(talker)
        talker.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\nTalker stopped by user")
    except Exception as e:
        print(f"Error in talker: {str(e)}")

if __name__ == '__main__':
    main()
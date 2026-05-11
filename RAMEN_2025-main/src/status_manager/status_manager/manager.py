import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16
from status_manager import (NAME)


class StatusManager(Node):
    def __init__(self):
        super().__init__('status_manager_node')
        self.name_list = ['AGV1', 'AGV2']
        
        self.name = NAME
        self.another_name = [name for name in self.name_list if name != self.name][0]
        self.status = 0
        self.get_logger().info(f'StatusManager initialized with name: {self.name} and another name: {self.another_name}')
        
        self.subscription = self.create_subscription(
            Int16,
            'force_status_set',
            self.force_status_set_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            f'/{self.another_name}_Button_pushed',
            self.AGV2_Button_pushed_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            '/tag_detection_1',
            self.camera1_callback,
            
            10
        )
        self.subscription = self.create_subscription(
            String,
            '/noodle_done',
            self.noodle_done_callback, 
            10
        )
        self.subscription = self.create_subscription(
            String,
            '/tag_detection_2',
            self.camera2_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            '/negi_done',
            self.negi_done_callback,
            10
        )
        
        self.subscription = self.create_subscription(
            String,
            'serve_arrive',
            self.serve_arrive_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            f'/{self.name}_Button_pushed',
            self.AGV1_Button_pushed_callback,
            10
        )
        
        self.publisher_mode = self.create_publisher(String, 'mode_change', 10)
        self.publisher_restart = self.create_publisher(String, '/start', 10)
        self.publisher_stop = self.create_publisher(String, '/stop', 10)
        self.publisher_target = self.create_publisher(String, 'move_target', 10)
        self.get_logger().info(f'StatusManager initialized with name: {self.name} and status: {self.status}')
       
       
        
    def force_status_set_callback(self, msg):
        self.status = msg.data
        self.get_logger().info(f'Status forced to set to {self.status}')

    def AGV2_Button_pushed_callback(self, msg):
        if self.status == 0:
            self.status = 1
            self.publisher_target.publish(String(data='none')) #目標位置ノードの目標を無しに
            self.publisher_restart.publish(String(data=f'{self.name}'))
            self.get_logger().info('AGV2 Button pushed, status changed to 1')
        
    def camera1_callback(self, msg):
        if self.status == 1:
            self.status = 2
            self.publisher_mode.publish(String(data=f'{self.name}')) #/mode_changeが送られる
            self.get_logger().info('Camera1 detected, status changed to 2')
        
    def noodle_done_callback(self, msg):
        if self.status == 2:
            self.publisher_mode.publish(String(data=f'{self.name}')) #/mode_changeが送られる
            self.publisher_restart.publish(String(data=f'{self.name}'))
            self.get_logger().info('Noodle done, restarting process')
            
        
    def camera2_callback(self, msg):
        if self.status == 2:
            self.status = 3
            self.publisher_mode.publish(String(data=f'{self.name}'))
            self.get_logger().info('Camera2 detected, status changed to 3')

    def negi_done_callback(self, msg):
        if self.status == 3:
            self.publisher_target.publish(String(data='serve')) #目標位置ノードの目標を提供位置に
            self.publisher_mode.publish(String(data=f'{self.name}')) #/mode_changeが送られる
            self.publisher_restart.publish(String(data=f'{self.name}'))
            self.get_logger().info('Negi done, serving noodles and restarting process')

    def serve_arrive_callback(self, msg):
        if self.status == 3:
            self.status = 4
            self.get_logger().info('Serve arrived, status changed to 4')

    def AGV1_Button_pushed_callback(self, msg):
        if self.status == 4:
            self.status = 0
            self.publisher_target.publish(String(data='return')) #目標位置ノードの目標を帰還位置に
            self.publisher_restart.publish(String(data=f'{self.name}'))
            self.get_logger().info('AGV1 Button pushed, status changed to 0')
            
    def destroy_node(self):
        self.get_logger().info('StatusManager node is being destroyed')
        super().destroy_node()
            
def main(args=None):
    rclpy.init(args=args)
    status_manager = StatusManager()
    rclpy.spin(status_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# 이 노드는 배달된 음식을 받고(/delivery_arrived) 상태를 확인합니다.
# 문제가 있으면 /request_remake 액션을 호출하여 재조리를 요청합니다.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ramen_interfaces.msg import CookedRamen
from ramen_interfaces.action import RequestRemake

class CustomerNode(Node):
    def __init__(self):
        super().__init__('customer_node')
        
        # 이 노드가 담당할 테이블 번호 (시뮬레이션을 위해 하드코딩)
        # 실제로는 각 테이블마다 이 노드가 실행되거나, UI에서 테이블 번호를 받아야 함
        # 여기서는 모든 테이블의 주문을 받는다고 가정합니다.
        
        # 배달 도착 알림을 구독
        self.subscription = self.create_subscription(
            CookedRamen,
            'delivery_arrived',
            self.delivery_callback,
            10)
        
        # 재조리 요청 액션 클라이언트
        self.remake_action_client = ActionClient(self, RequestRemake, 'request_remake')
        
        # 재조리 완료된 음식을 다시 서빙 로봇에게 전달하는 퍼블리셔
        self.remake_publisher = self.create_publisher(CookedRamen, 'food_ready', 10)
        
        self.get_logger().info('고객 노드(CustomerNode) 활성화. 배달을 기다립니다...')

    def delivery_callback(self, msg: CookedRamen):
        table = msg.order.table_number
        self.get_logger().info(f'테이블 {table}: 배달 도착! 상태 확인 중...')
        
        if msg.cook_status == "perfect":
            self.get_logger().info(f'[테이블 {table}] 조리 상태 완벽! "맛있게 먹겠습니다!"')
        else:
            self.get_logger().warn(f'[테이블 {table}] 조리 상태 불량 ({msg.cook_status}).')
            self.get_logger().warn(f'[테이블 {table}] "조리가 잘못되었습니다! 재조리 요청합니다."')
            
            # 재조리 요청 액션 호출
            self.send_remake_request(msg)

    def send_remake_request(self, bad_food: CookedRamen):
        if not self.remake_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('재조리(RequestRemake) 액션 서버를 찾을 수 없습니다.')
            return

        goal_msg = RequestRemake.Goal()
        goal_msg.bad_food = bad_food
        
        self.get_logger().info('주방에 재조리 요청(Goal) 전송 중...')
        
        self.remake_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.remake_feedback_callback
        ).add_done_callback(self.remake_goal_response_callback)

    def remake_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('재조리 요청이 주방에서 거부되었습니다.')
            return
        
        self.get_logger().info('주방에서 재조리 요청을 수락했습니다. (Goal Accepted)')
        goal_handle.get_result_async().add_done_callback(self.remake_result_callback)

    def remake_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[피드백] 주방 재조리 진행 상태: {feedback.progress}')

    def remake_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('재조리가 완료되었습니다! (Result Received)')
        
        new_food: CookedRamen = result.new_food
        self.get_logger().info(f'새로 받은 음식: {new_food.order.ramen_type} (상태: {new_food.cook_status})')
        
        # 재조리된 음식을 다시 서빙 로봇에게 전달
        self.get_logger().info('재조리된 음식을 서빙 로봇에게 전달합니다 (/food_ready)...')
        self.remake_publisher.publish(new_food)


def main(args=None):
    rclpy.init(args=args)
    customer_node = CustomerNode()
    rclpy.spin(customer_node)
    customer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
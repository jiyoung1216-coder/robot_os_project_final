# 이 노드는 /new_order를 구독하여 주문을 받고, 조리(랜덤 상태) 후 /food_ready로 보냅니다.
# 또한 /request_remake 액션 서버를 운영하여 재조리 요청을 처리합니다.

import rclpy
import time
import random
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

from ramen_interfaces.msg import RamenOrder, CookedRamen
from ramen_interfaces.action import RequestRemake

class KitchenNode(Node):
    def __init__(self):
        super().__init__('kitchen_node')
        
        # 신규 주문 구독
        self.subscription = self.create_subscription(
            RamenOrder,
            'new_order',
            self.order_callback,
            10)
        
        # 조리 완료 음식 발행
        self.publisher = self.create_publisher(CookedRamen, 'food_ready', 10)
        
        # 재조리 요청 액션 서버
        self.remake_action_server = ActionServer(
            self,
            RequestRemake,
            'request_remake',
            goal_callback=self.remake_goal_callback,
            handle_accepted_callback=self.remake_handle_accepted_callback,
            execute_callback=self.remake_execute_callback,
            cancel_callback=self.remake_cancel_callback
        )
        
        # 조리 상태 랜덤 할당을 위한 설정
        self.cook_choices = ["perfect", "undercooked", "overcooked", "missing_topping", "missing_side"]
        self.cook_weights = [0.8, 0.05, 0.05, 0.05, 0.05]  # 정상 80%, 비정상 20%

        self.get_logger().info('주방(KitchenNode) 오픈. 주문을 기다립니다...')

    def order_callback(self, msg: RamenOrder):
        self.get_logger().info(f'테이블 {msg.table_number}로부터 신규 주문 접수:')
        self.get_logger().info(f' - {msg.ramen_type} (토핑: {msg.toppings}, 사이드: {msg.sides})')
        
        # 조리 시뮬레이션
        self.get_logger().info('조리 시작... (5초 소요)')
        time.sleep(5)
        
        # 랜덤 조리 상태 할당
        cook_status = random.choices(self.cook_choices, self.cook_weights, k=1)[0]
        
        cooked_food = CookedRamen()
        cooked_food.order = msg
        cooked_food.cook_status = cook_status
        
        self.get_logger().info(f'조리 완료! 상태: {cook_status}')
        
        # 조리 완료된 음식을 서빙 로봇에게 전달
        self.publisher.publish(cooked_food)
        self.get_logger().info('서빙 로봇에게 음식 전달 (/food_ready)...')

    # --- 재조리 액션 서버 콜백 ---

    def remake_goal_callback(self, goal_request: RequestRemake.Goal):
        """액션 Goal 요청 수신 시 호출됨. 요청을 수락할지 거부할지 결정."""
        self.get_logger().info(f'재조리 요청 수신: {goal_request.bad_food.order.ramen_type}')
        # 현재는 모든 재조리 요청을 수락
        return GoalResponse.ACCEPT

    def remake_handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        """Goal이 수락된 직후 호출됨 (execute_callback 전에)."""
        self.get_logger().info('재조리 요청을 수락했습니다. 조리사에게 전달 중...')
        # 별도 스레드에서 실행되도록 execute_callback을 시작
        goal_handle.execute()

    def remake_execute_callback(self, goal_handle: ServerGoalHandle):
        """Goal 실행 시 호출됨. 여기가 실제 작업이 일어나는 곳."""
        self.get_logger().info('재조리 시작... (5초 소요)')
        
        feedback_msg = RequestRemake.Feedback()
        feedback_msg.progress = "Re-cooking"
        goal_handle.publish_feedback(feedback_msg)
        
        time.sleep(5) # 재조리 시간
        
        # 재조리 시에는 무조건 'perfect' 상태로 만듦
        new_food = CookedRamen()
        new_food.order = goal_handle.request.bad_food.order
        new_food.cook_status = "perfect" # 재조리는 무조건 성공
        
        feedback_msg.progress = "Remake_Done"
        goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info('재조리 완료! (상태: perfect)')
        
        # Goal을 성공 상태로 설정
        goal_handle.succeed()
        
        # Result를 채워서 반환
        result = RequestRemake.Result()
        result.new_food = new_food
        return result

    def remake_cancel_callback(self, goal_handle: ServerGoalHandle):
        """취소 요청 시 호출됨."""
        self.get_logger().warn('재조리 취소 요청 수신!')
        return CancelResponse.ACCEPT # 취소 수락


def main(args=None):
    rclpy.init(args=args)
    kitchen_node = KitchenNode()
    rclpy.spin(kitchen_node)
    kitchen_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
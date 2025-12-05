#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ramen_interfaces.srv import OrderService
from ramen_interfaces.msg import RamenOrder


class CounterNode(Node):
    def __init__(self):
        super().__init__('counter_node')
        
        self.order_count = 0
        
        # 서비스 서버 생성
        self.order_service = self.create_service(
            OrderService,
            'order_service',
            self.handle_order
        )
        
        # ✅ 주방으로 주문 전송할 Publisher 추가!
        self.order_publisher = self.create_publisher(
            RamenOrder, 
            '/new_order', 
            10
        )
        
        self.get_logger().info('=== 🍜 카운터(POS) 시스템 가동 중 ===')
        self.get_logger().info('📡 서비스 대기중: /order_service')
        self.get_logger().info('📤 주방 연결: /new_order')

    def handle_order(self, request, response):
        """주문 서비스 처리"""
        self.order_count += 1
        order_id = self.order_count
        
        # 주문 정보 출력
        toppings = list(request.toppings) if request.toppings else []
        sides = list(request.sides) if request.sides else []
        drinks = list(request.drinks) if request.drinks else []
        ramen_type = request.ramen_type if request.ramen_type != "없음" else "없음"
        
        self.get_logger().info(
            f'\n🔔 ═══════════════════════════════════'
            f'\n🔔 새 주문 접수! (주문번호: {order_id})'
            f'\n   📍 테이블: {request.table_number}번'
            f'\n   🍜 라면: {ramen_type}'
            f'\n   🥚 토핑: {toppings if toppings else "없음"}'
            f'\n   🍱 사이드: {sides if sides else "없음"}'
            f'\n   🥤 음료: {drinks if drinks else "없음"}'
            f'\n   💰 금액: {request.total_price:,.0f}원'
            f'\n   💳 결제: {request.payment_method}'
            f'\n🔔 ═══════════════════════════════════'
        )
        
        # ✅ 주방으로 주문 전송 (조건 없이 항상 전송!)
        order_msg = RamenOrder()
        order_msg.table_number = int(request.table_number)
        order_msg.ramen_type = ramen_type
        order_msg.toppings = toppings
        order_msg.sides = sides
        order_msg.drinks = drinks
        order_msg.total_price = float(request.total_price)
        order_msg.payment_method = request.payment_method
        order_msg.pay_now = request.pay_now if hasattr(request, 'pay_now') else True
        order_msg.currency = "KRW"
        
        self.order_publisher.publish(order_msg)
        self.get_logger().info(f'📤 주방으로 주문 전송 완료! (테이블: {request.table_number}번)')
        
        # 응답 설정
        response.success = True
        response.message = f"테이블 {request.table_number}번 주문 접수 완료!"
        response.order_id = order_id
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
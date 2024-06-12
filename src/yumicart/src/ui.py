import rospy
from std_msgs.msg import String

from enum import Enum
import pygame
import os

class Screen(Enum):
    MAIN = 0
    EVENTS = 1
    EVENT1 = 2
    EVENT2 = 3
    EVENT3 = 4
    EVENT4 = 5
    DIRECTIONS = 6
    CHECKOUT = 7
    CART = 8

# ROS 콜백 함수
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

def main():
    # ROS 초기화
    rospy.init_node('pygame_ros_listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)

    # Pygame 초기화
    pygame.init()
    screen = pygame.display.set_mode((1920, 1080), pygame.FULLSCREEN)
    pygame.display.set_caption("Pygame and ROS Example")

    screen_num = 0
    images_name = ['main.png', 'events.png', 'event_1.png', 'event_2.png', 'event_3.png', 'event_4.png', 'directions.png', 'check_out.png', 'cart.png']
    images = []

    # 배경 이미지 로드
    current_dir = os.path.dirname(os.path.abspath(__file__))
    for i in range(9):
        image_path = os.path.join(current_dir, f'../images/{images_name[i]}')
        image = pygame.image.load(image_path)
        images.append(image)

    clock = pygame.time.Clock()
    running = True

    # 특정 영역 설정 (여기서는 (100, 100)에서 (200, 200)까지의 사각형 영역)
    target_rect = pygame.Rect(1306, 766, 519, 222)

    while not rospy.is_shutdown() and running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                print(f"Mouse click at: {pos}")
                
                # 클릭된 위치가 특정 영역 내에 있는지 확인
                if target_rect.collidepoint(pos):
                    print("Hello World!")

        # 배경 이미지 그리기
        screen.blit(images[screen_num], (0, 0))
        pygame.display.flip()

        # 주기적으로 ROS 콜백 함수들을 호출
        rospy.rostime.wallsleep(0.01)  # 10ms 대기

        # Pygame의 프레임 속도 설정 (예: 초당 30프레임)
        clock.tick(30)

    pygame.quit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
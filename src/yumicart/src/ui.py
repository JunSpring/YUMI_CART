#! /usr/bin/env python
#-*- coding: utf-8 -*-

# Basic Import
import rospy

import pygame
import os

# Enum Import
from enum import IntEnum  
from enums import DriveModeNum

# Publisher msg Import
from yumicart.msg import ui_msgs

# Subscriber msg Import
from yumicart.msg import center_msgs

class Screen(IntEnum):
    MAIN = 0
    EVENTS = 1
    EVENT1 = 2
    EVENT2 = 3
    EVENT3 = 4
    EVENT4 = 5
    DIRECTIONS = 6
    CHECKOUT = 7
    CART = 8

class UI():
    def __init__(self):
        # ROS 초기화
        rospy.loginfo('UI is Created')

        # Publish Declaration
        ui_pub = rospy.Publisher('/ui', ui_msgs, queue_size=10)

        drive_mode = DriveModeNum.STOP.value
        product_num = 0

        # Pygame 초기화
        pygame.init()
        screen = pygame.display.set_mode((1920, 1080))#, pygame.FULLSCREEN)
        pygame.display.set_caption("YUMI CART")

        screen_num = Screen.MAIN.value
        images_name = ['main.png', 'events.png', 'event_1.png', 'event_2.png', 'event_3.png', 'event_4.png', 'directions.png', 'check_out.png', 'cart.png']
        images = []

        # 배경 이미지 로드
        current_dir = os.path.dirname(os.path.abspath(__file__))
        for i in range(9):
            image_path = os.path.join(current_dir, f'../images/{images_name[i]}')
            image = pygame.image.load(image_path)
            images.append(image)

        clock = pygame.time.Clock()
        pygame_running = True

        while not rospy.is_shutdown() and pygame_running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()

                    if screen_num == Screen.MAIN.value:
                        shopping_start_rect = pygame.Rect(1306, 766, 519, 222)

                        if shopping_start_rect.collidepoint(pos):
                            screen_num = Screen.CART.value

                    elif screen_num == Screen.EVENTS.value:
                        # event1_rect = pygame.Rect()
                        # event2_rect = pygame.Rect()
                        # event3_rect = pygame.Rect()
                        # event4_rect = pygame.Rect()
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        # if event1_rect.collidepoint(pos):
                        #     screen_num = Screen.EVENT1.value
                        # if event2_rect.collidepoint(pos):
                        #     screen_num = Screen.EVENT2.value
                        # if event3_rect.collidepoint(pos):
                        #     screen_num = Screen.EVENT3.value
                        # if event4_rect.collidepoint(pos):
                        #     screen_num = Screen.EVENT4.value
                        if back_rect.collidepoint(pos):
                            screen_num = Screen.CART.value

                    elif screen_num == Screen.EVENT1.value:
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        if back_rect.collidepoint(pos):
                            screen_num = Sreen.CART.value

                    elif screen_num == Screen.EVENT2.value:
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        if back_rect.collidepoint(pos):
                            screen_num = Sreen.CART.value

                    elif screen_num == Screen.EVENT3.value:
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        if back_rect.collidepoint(pos):
                            screen_num = Sreen.CART.value

                    elif screen_num == Screen.EVENT4.value:
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        if back_rect.collidepoint(pos):
                            screen_num = Sreen.CART.value

                    elif screen_num == Screen.DIRECTIONS.value:
                        cham_rect = pygame.Rect(65, 84, 399, 462)
                        bu_rect = pygame.Rect(526, 84, 399, 462)
                        ho_rect = pygame.Rect(988, 84, 399, 462)
                        cho_rect = pygame.Rect(1450, 84, 399, 462)
                        navi_start = pygame.Rect(116, 722, 799, 275)
                        navi_stop = pygame.Rect(988, 722, 799, 275)

                        if cham_rect.collidepoint(pos):
                            pass
                        if bu_rect.collidepoint(pos):
                            pass
                        if ho_rect.collidepoint(pos):
                            pass
                        if cho_rect.collidepoint(pos):
                            pass
                        if navi_start.collidepoint(pos):
                            pass
                        if navi_stop.collidepoint(pos):
                            screen_num = Screen.CART.value

                    elif screen_num == Screen.CHECKOUT.value:
                        back_rect = pygame.Rect(1103, 431, 715, 248)
                        shopping_quit_rect = pygame.Rect(1103, 744, 715, 248)

                        if back_rect.collidepoint(pos):
                            screen_num = Screen.CART.value
                        if shopping_quit_rect.collidepoint(pos):
                            pygame_running = False

                    elif screen_num == Screen.CART.value:
                        directions_rect = pygame.Rect(860, 137, 977, 252)
                        checkout_rect = pygame.Rect(860, 458, 977, 252)
                        events_rect = pygame.Rect(860, 779, 455, 252)
                        stop_go_rect = pygame.Rect(1382, 779, 455, 252 )

                        if directions_rect.collidepoint(pos):
                            screen_num = Screen.DIRECTIONS.value
                        if checkout_rect.collidepoint(pos):
                            screen_num = Screen.CHECKOUT.value
                        if events_rect.collidepoint(pos):
                            screen_num = Screen.EVENTS.value
                        if stop_go_rect.collidepoint(pos):
                            if drive_mode == DriveModeNum.FOLLOWING.value:
                                drive_mode = DriveModeNum.STOP.value
                            elif drive_mode == DriveModeNum.STOP.value:
                                drive_mode = DriveModeNum.FOLLOWING.value

            # 배경 이미지 그리기
            screen.blit(images[screen_num], (0, 0))
            pygame.display.flip()

            # Int32 메시지 퍼블리시
            temp_ui_msgs = ui_msgs()
            temp_ui_msgs.drive_mode = drive_mode
            temp_ui_msgs.product_number = product_num
            ui_pub.publish(temp_ui_msgs)

            # 주기적으로 ROS 콜백 함수들을 호출
            rospy.rostime.wallsleep(0.01)  # 10ms 대기

            # Pygame의 프레임 속도 설정 (예: 초당 30프레임)
            clock.tick(30)

        pygame.quit()

def run():
    rospy.init_node('ui_node')
    ui = UI()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
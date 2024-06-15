#! /usr/bin/env python
#-*- coding: utf-8 -*-

# Basic Import
import rospy

import pygame
import os

# Enum Import
from enum import IntEnum  
from enums import DriveModeNum, ProductNum

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
    DIRECTIONBU = 7
    DIRECTIONCHAM = 8
    DIRECTIONHO = 9
    DIRECTIONCHO = 10
    CHECKOUT = 11
    CARTGO = 12
    CARTSTOP = 13
    CARTBU = 14
    CARTCHAM = 15
    CARTHO = 16
    CARTCHO = 17

class UI():
    def __init__(self):
        # ROS 초기화
        rospy.loginfo('UI is Created')

        # Publish Declaration
        ui_pub = rospy.Publisher('/ui', ui_msgs, queue_size=10)

        self.drive_mode = DriveModeNum.STOP
        temp_product_num = -1
        product_num = -1

        # Pygame 초기화
        pygame.init()
        screen = pygame.display.set_mode((1920, 1080))#, pygame.FULLSCREEN)
        pygame.display.set_caption("YUMI CART")

        screen_num = Screen.MAIN
        images_name = [ 'main.png', 'events.png', 'event1.png', 'event2.png', 'event3.png', 'event4.png',
                        'directions.png', 'direction_bu.png', 'direction_cham.png', 'direction_ho.png', 'direction_cho.png',
                        'check_out.png',
                        'cart_go.png', 'cart_stop.png',
                        'cart_bu.png', 'cart_cham.png', 'cart_ho.png', 'cart_cho.png']
        images = []

        # 배경 이미지 로드
        current_dir = os.path.dirname(os.path.abspath(__file__))
        for i in range(18):
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

                    if screen_num == Screen.MAIN:
                        shopping_start_rect = pygame.Rect(1306, 766, 519, 222)

                        if shopping_start_rect.collidepoint(pos):
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.EVENTS:
                        # event1_rect = pygame.Rect()
                        # event2_rect = pygame.Rect()
                        # event3_rect = pygame.Rect()
                        # event4_rect = pygame.Rect()
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        # if event1_rect.collidepoint(pos):
                        #     screen_num = Screen.EVENT1
                        # if event2_rect.collidepoint(pos):
                        #     screen_num = Screen.EVENT2
                        # if event3_rect.collidepoint(pos):
                        #     screen_num = Screen.EVENT3
                        # if event4_rect.collidepoint(pos):
                        #     screen_num = Screen.EVENT4
                        if back_rect.collidepoint(pos):
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.EVENT1:
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        if back_rect.collidepoint(pos):
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.EVENT2:
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        if back_rect.collidepoint(pos):
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.EVENT3:
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        if back_rect.collidepoint(pos):
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.EVENT4:
                        back_rect = pygame.Rect(1471, 772, 379, 221)

                        if back_rect.collidepoint(pos):
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.DIRECTIONS or\
                         screen_num == Screen.DIRECTIONBU or\
                         screen_num == Screen.DIRECTIONCHAM or\
                         screen_num == Screen.DIRECTIONHO or\
                         screen_num == Screen.DIRECTIONCHO:
                        cham_rect = pygame.Rect(65, 84, 399, 462)
                        bu_rect = pygame.Rect(526, 84, 399, 462)
                        ho_rect = pygame.Rect(988, 84, 399, 462)
                        cho_rect = pygame.Rect(1450, 84, 399, 462)
                        navi_start = pygame.Rect(116, 722, 799, 275)
                        navi_stop = pygame.Rect(988, 722, 799, 275)

                        if cham_rect.collidepoint(pos):
                            temp_product_num = ProductNum.CHAMKKAERAMEN
                            screen_num = Screen.DIRECTIONCHAM
                        if bu_rect.collidepoint(pos):
                            temp_product_num = ProductNum.BRAVO
                            screen_num = Screen.DIRECTIONBU
                        if ho_rect.collidepoint(pos):
                            temp_product_num = ProductNum.CHAPSSALHOTTEONGMIX
                            screen_num = Screen.DIRECTIONHO
                        if cho_rect.collidepoint(pos):
                            temp_product_num = ProductNum.CHOCOBI
                            screen_num = Screen.DIRECTIONCHO
                        if navi_start.collidepoint(pos):
                            product_num = temp_product_num
                            self.drive_mode = DriveModeNum.SEARCHING
                        if navi_stop.collidepoint(pos):
                            temp_product_num = -1
                            product_num = -1
                            self.drive_mode = DriveModeNum.STOP
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.CHECKOUT:
                        back_rect = pygame.Rect(1103, 431, 715, 248)
                        shopping_quit_rect = pygame.Rect(1103, 744, 715, 248)

                        if back_rect.collidepoint(pos):
                            self.drive_mode = DriveModeNum.STOP
                            screen_num = self.ScreenGoStop()
                        if shopping_quit_rect.collidepoint(pos):
                            self.drive_mode = DriveModeNum.STOP
                            pygame_running = False

                    elif screen_num == Screen.CARTGO or\
                         screen_num == Screen.CARTSTOP:
                        directions_rect = pygame.Rect(860, 137, 977, 252)
                        checkout_rect = pygame.Rect(860, 458, 977, 252)
                        events_rect = pygame.Rect(860, 779, 455, 252)
                        stop_go_rect = pygame.Rect(1382, 779, 455, 252 )

                        if directions_rect.collidepoint(pos):
                            screen_num = Screen.DIRECTIONS
                        if checkout_rect.collidepoint(pos):
                            self.drive_mode = DriveModeNum.PAYMENT
                            screen_num = Screen.CHECKOUT
                        if events_rect.collidepoint(pos):
                            screen_num = Screen.EVENTS
                        if stop_go_rect.collidepoint(pos):
                            if self.drive_mode == DriveModeNum.FOLLOWING:
                                self.drive_mode = DriveModeNum.STOP
                                screen_num = Screen.CARTSTOP
                            elif self.drive_mode == DriveModeNum.STOP:
                                self.drive_mode = DriveModeNum.FOLLOWING
                                screen_num = Screen.CARTGO

            # 배경 이미지 그리기
            screen.blit(images[screen_num], (0, 0))
            pygame.display.flip()

            # Int32 메시지 퍼블리시
            temp_ui_msgs = ui_msgs()
            temp_ui_msgs.drive_mode = self.drive_mode
            temp_ui_msgs.product_number = product_num
            ui_pub.publish(temp_ui_msgs)

            # 주기적으로 ROS 콜백 함수들을 호출
            rospy.rostime.wallsleep(0.01)  # 10ms 대기

            # Pygame의 프레임 속도 설정 (예: 초당 30프레임)
            clock.tick(30)

        pygame.quit()

    def ScreenGoStop(self):
        if self.drive_mode == DriveModeNum.FOLLOWING:
            return Screen.CARTGO
        else:
            return Screen.CARTSTOP

def run():
    rospy.init_node('ui_node')
    ui = UI()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
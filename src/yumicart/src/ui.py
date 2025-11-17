#! /usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import pygame
import os

from enum import IntEnum
from enums import DriveModeNum, ProductNum

from yumicart.msg import ui_msgs
from scale_car_yolov5.msg   import Objects, Yolo_Objects

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
        rospy.loginfo('UI is Created')

        ui_pub = rospy.Publisher('/ui', ui_msgs, queue_size=10)
        rospy.Subscriber('/yolov5_pub', Yolo_Objects, self.yolo_callback)
        self.products = []

        self.drive_mode = DriveModeNum.STOP
        temp_product_num = -1
        product_num = -1

        # -----------------------------
        # Pygame 초기화 및 스케일 설정
        # -----------------------------
        pygame.init()
        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)  # 실제 화면 해상도
        screen_w, screen_h = screen.get_size()

        # 내부 렌더링 해상도(에셋 기준 해상도 = 1920x1080)
        base_w, base_h = 1920, 1080
        base_surface = pygame.Surface((base_w, base_h))

        # 화면 스케일 비율 (가로/세로 각각 별도로 스트레치)
        scale_x = float(screen_w) / float(base_w)
        scale_y = float(screen_h) / float(base_h)

        pygame.display.set_caption("YUMI CART")

        screen_num = Screen.MAIN
        images_name = [
            'main.png', 'events.png', 'event1.png', 'event2.png', 'event3.png', 'event4.png',
            'directions.png', 'direction_bu.png', 'direction_cham.png', 'direction_ho.png', 'direction_cho.png',
            'check_out.png',
            'cart_go.png', 'cart_stop.png',
            'cart_bu.png', 'cart_cham.png', 'cart_ho.png', 'cart_cho.png'
        ]
        images = []

        # 배경 이미지 로드 (1920x1080 기준)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        for i in range(18):
            image_path = os.path.join(current_dir, f'../images/{images_name[i]}')
            img = pygame.image.load(image_path).convert()
            images.append(img)

        # 장바구니 미리보기(체크아웃 리스트용) 소형 이미지만 기존처럼 리스케일
        for i in range(Screen.CARTBU, Screen.CARTCHO+1):
            images[i] = pygame.transform.scale(images[i], (911, 227))  # 3131x780 → 911x227로 표시

        clock = pygame.time.Clock()
        pygame_running = True

        while not rospy.is_shutdown() and pygame_running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame_running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    pygame_running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # 실제 화면 좌표 → 내부(1920x1080) 좌표로 역변환
                    mx, my = pygame.mouse.get_pos()
                    bx = int(mx / scale_x)
                    by = int(my / scale_y)
                    pos = (bx, by)

                    if screen_num == Screen.MAIN:
                        shopping_start_rect = pygame.Rect(1306, 766, 519, 222)
                        if shopping_start_rect.collidepoint(pos):
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.EVENTS:
                        event1_rect = pygame.Rect(69, 238, 1313, 189)
                        event2_rect = pygame.Rect(69, 427, 1313, 189)
                        event3_rect = pygame.Rect(69, 616, 1313, 189)
                        event4_rect = pygame.Rect(69, 805, 1313, 189)
                        back_rect  = pygame.Rect(1471, 772, 379, 221)

                        if event1_rect.collidepoint(pos):
                            screen_num = Screen.EVENT1
                        elif event2_rect.collidepoint(pos):
                            screen_num = Screen.EVENT2
                        elif event3_rect.collidepoint(pos):
                            screen_num = Screen.EVENT3
                        elif event4_rect.collidepoint(pos):
                            screen_num = Screen.EVENT4
                        elif back_rect.collidepoint(pos):
                            screen_num = self.ScreenGoStop()

                    elif screen_num == Screen.EVENT1:
                        back_rect = pygame.Rect(1471, 772, 379, 221)
                        if back_rect.collidepoint(pos):
                            screen_num = Screen.EVENTS

                    elif screen_num == Screen.EVENT2:
                        back_rect = pygame.Rect(1471, 772, 379, 221)
                        if back_rect.collidepoint(pos):
                            screen_num = Screen.EVENTS

                    elif screen_num == Screen.EVENT3:
                        back_rect = pygame.Rect(1471, 772, 379, 221)
                        if back_rect.collidepoint(pos):
                            screen_num = Screen.EVENTS

                    elif screen_num == Screen.EVENT4:
                        back_rect = pygame.Rect(1471, 772, 379, 221)
                        if back_rect.collidepoint(pos):
                            screen_num = Screen.EVENTS

                    elif screen_num in (Screen.DIRECTIONS, Screen.DIRECTIONBU, Screen.DIRECTIONCHAM, Screen.DIRECTIONHO, Screen.DIRECTIONCHO):
                        cham_rect   = pygame.Rect(65, 84, 399, 462)
                        bu_rect     = pygame.Rect(526, 84, 399, 462)
                        ho_rect     = pygame.Rect(988, 84, 399, 462)
                        cho_rect    = pygame.Rect(1450, 84, 399, 462)
                        navi_start  = pygame.Rect(116, 722, 799, 275)
                        navi_stop   = pygame.Rect(988, 722, 799, 275)

                        if cham_rect.collidepoint(pos):
                            temp_product_num = ProductNum.CHAMKKAERAMEN
                            screen_num = Screen.DIRECTIONCHAM
                        elif bu_rect.collidepoint(pos):
                            temp_product_num = ProductNum.BRAVO
                            screen_num = Screen.DIRECTIONBU
                        elif ho_rect.collidepoint(pos):
                            temp_product_num = ProductNum.CHAPSSALHOTTEONGMIX
                            screen_num = Screen.DIRECTIONHO
                        elif cho_rect.collidepoint(pos):
                            temp_product_num = ProductNum.CHOCOBI
                            screen_num = Screen.DIRECTIONCHO
                        elif navi_start.collidepoint(pos):
                            product_num = temp_product_num
                            self.drive_mode = DriveModeNum.SEARCHING
                        elif navi_stop.collidepoint(pos):
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
                        elif shopping_quit_rect.collidepoint(pos):
                            self.drive_mode = DriveModeNum.STOP
                            screen_num = Screen.MAIN

                    elif screen_num in (Screen.CARTGO, Screen.CARTSTOP):
                        directions_rect = pygame.Rect(860, 137, 977, 252)
                        checkout_rect   = pygame.Rect(860, 458, 977, 252)
                        events_rect     = pygame.Rect(860, 779, 455, 252)
                        stop_go_rect    = pygame.Rect(1382, 779, 455, 252)

                        if directions_rect.collidepoint(pos):
                            screen_num = Screen.DIRECTIONS
                        elif checkout_rect.collidepoint(pos):
                            self.drive_mode = DriveModeNum.PAYMENT
                            screen_num = Screen.CHECKOUT
                        elif events_rect.collidepoint(pos):
                            screen_num = Screen.EVENTS
                        elif stop_go_rect.collidepoint(pos):
                            if self.drive_mode == DriveModeNum.FOLLOWING:
                                self.drive_mode = DriveModeNum.STOP
                                screen_num = Screen.CARTSTOP
                            elif self.drive_mode == DriveModeNum.STOP:
                                self.drive_mode = DriveModeNum.FOLLOWING
                                screen_num = Screen.CARTGO

            # -----------------------------
            # 그리기: 내부 Surface(1920x1080)에 모두 그림
            # -----------------------------
            base_surface.blit(images[screen_num], (0, 0))
            if screen_num == Screen.CHECKOUT:
                for i, product in enumerate(self.products):
                    base_surface.blit(images[product + 14], (101, 86 + i * 226))

            # -----------------------------
            # 화면으로 스트레치 출력
            # -----------------------------
            scaled = pygame.transform.smoothscale(base_surface, (screen_w, screen_h))
            screen.blit(scaled, (0, 0))
            pygame.display.flip()

            # ROS 메시지 퍼블리시
            temp_ui_msgs = ui_msgs()
            temp_ui_msgs.drive_mode = self.drive_mode
            temp_ui_msgs.product_number = product_num
            ui_pub.publish(temp_ui_msgs)

            # 콜백 처리 및 FPS
            rospy.rostime.wallsleep(0.01)
            clock.tick(30)

        pygame.quit()

    def ScreenGoStop(self):
        if self.drive_mode == DriveModeNum.FOLLOWING:
            return Screen.CARTGO
        else:
            return Screen.CARTSTOP

    def yolo_callback(self, msg):
        self.products.clear        ()
        for yolo_object in msg.yolo_objects:
            self.products.append(yolo_object.c)
        print(self.products)

def run():
    rospy.init_node('ui_node')
    ui = UI()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

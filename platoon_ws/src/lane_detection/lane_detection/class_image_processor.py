import numpy as np
import cv2 as cv
import math
class ImageProcessor():
    def __init__(self):
        # self.warp_parameters = [(160,220),(55 ,480),(460,220),(550,480)]
        # self.warp_parameters = [(290, 0), (110, 480), (350, 0), (530, 480)]
        # self.warp_parameters = [(274, 0), (0, 480), (366, 0), (640, 480)]
        # self.warp_parameters = [(258, 28), (0, 480), (382, 28), (640, 480)]
        # self.warp_parameters = [(250, 42), (0, 480), (390 , 42), (640, 480)]
        # self.warp_parameters = [(243, 55), (0, 480), (397, 55), (640, 480)]
        # self.warp_parameters = [(269, 55), (110, 480), (371, 55), (530, 480)]
        self.warp_parameters = [(211, 110), (0, 480), (429, 110), (640, 480)]
        # self.warp_parameters = [(249, 110), (110, 480), (391, 110), (530, 480)]
        # self.warp_parameters = [(149, 220), (0, 480), (491, 220), (640, 480)]
        # self.warp_parameters = [(207, 220), (110, 480), (433, 220), (530, 480)]

        self.hsv_values = (0, 0, 140) #(0, 0, 160)
        self.prev_centers = None
        self.front_distance_m = None

    def frame_processor(self, image, truck_id):
        # cv.imshow('Original Image', image)

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        binary_color = cv.inRange(hsv, self.hsv_values, (180, 255, 255))
        # cv.imshow('HSV Image', binary_color)

        # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        # cv.imshow('Gray Image', gray)

        blur = cv.GaussianBlur(gray, (5, 5), 0)
        # cv.imshow('Blur Image', blur)

        edges = cv.Canny(blur, 50, 150)
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(edges, contours, -1, (255), 5)
        # cv.imshow('Edges Image', edges)

        binary = cv.bitwise_and(binary_color, edges)
        # cv.imshow('Filtered Image', binary)

        warped_image, M = self.warp_image(binary)
        M_inv = np.linalg.inv(M) 
        # cv.imshow('Warp Image', warped_image)


        centers, dbg = self.sliding_window_dual(warped_image, nwindows=16, draw=True, truck_id=truck_id)
        #if dbg is not None:
        # cv.imshow(f"Lane Debug {truck_id}", dbg)

        self.prev_centers = centers

        if centers:
            pts = np.float32(centers).reshape(-1,1,2)
            pts_back = cv.perspectiveTransform(pts, M_inv)
            pts_back = pts_back.reshape(-1,2)

            for x,y in pts_back:
                cv.circle(image, (int(x), int(y)), 4, (0,0,0), -1)

            cv.imshow(f"Centers on Original {truck_id}", image)

        cv.waitKey(1)
        return centers

    def warp_image(self, image):
        height, width = image.shape
        pts1 = np.float32([self.warp_parameters[0], self.warp_parameters[1], self.warp_parameters[2], self.warp_parameters[3]]) 
        pts2 = np.float32([[0, 0], [0, height], [width, 0], [width, height]]) 

        matrix = cv.getPerspectiveTransform(pts1, pts2) 
        transformed_frame = cv.warpPerspective(image, matrix, (width,height))

        return transformed_frame, matrix

    def sliding_window_dual(self, binary_warped, nwindows=15, margin=70, minpix=300,
                        draw=False, truck_id=0):
        """
        좌·우 차선을 동시에 찾는 슬라이딩 윈도우
        -----------------------------------------------------------
        반환값
        left_fit, right_fit : 각 차선 2차 다항식 계수 (a,b,c)  (없으면 None)
        out_img             : (선택) 시각화 이미지
        """
        # 0/255 이진화 보장
        if len(binary_warped.shape) == 3:
            binary_warped = cv.cvtColor(binary_warped, cv.COLOR_BGR2GRAY)
        _, binary_warped = cv.threshold(binary_warped, 1, 255, cv.THRESH_BINARY)

        H, W = binary_warped.shape # 480, 640

        # ===== 앞차 거리 기반 ROI 계산 =====
        if self.front_distance_m is not None and truck_id != 0:
            pixel_limit = int(self.front_distance_m * 61.6214)  # 1m 당 45.329px
            pixel_limit = min(pixel_limit, H)
        else:
            pixel_limit = H
        nwindows = 10 if pixel_limit < 300 else 16 # window 개수를 pixel_limit(ROI 높이)에 따라 다르게 설정
        

        # histogram = np.sum(binary_warped[H//2:, :], axis=0)
        # histogram = np.sum(binary_warped, axis=0) # 히스토그램 전체로 다 보기
        histogram = np.sum(binary_warped[H - pixel_limit : H, :], axis=0) # 히스토그램 ROI만큼만 보기

        midpoint     = W // 2
        leftx_base   = np.argmax(histogram[:midpoint])
        rightx_relative = np.argmax(histogram[midpoint:])
        rightx_base = rightx_relative + midpoint
        # if histogram[rightx_base] == 0:
        #     rightx_base = int(W * 0.85)

        # non-zero 픽셀 좌표
        nz_y, nz_x   = binary_warped.nonzero()

        # 윈도우 설정
        # win_h        = H // nwindows
        win_h = pixel_limit // nwindows
        minpix = int(win_h * (300 / 30)) # 차간 거리 - 윈도우 개수 - 윈도우 높이에 영향을 주기 때문에 minpx도 비례상수 이용하여 계산
        leftx_cur    = leftx_base
        rightx_cur   = rightx_base
        left_inds, right_inds = [], []
        # 중앙점 설정
        centers = []

        # 시각화용
        out_img = cv.cvtColor(binary_warped, cv.COLOR_GRAY2BGR) if draw else None

        for window in range(nwindows):
            win_y_low   = H - (window + 1) * win_h
            win_y_high  = H -  window      * win_h

            win_xleft_low   = leftx_cur  - margin
            win_xleft_high  = leftx_cur  + margin
            win_xright_low  = rightx_cur - margin
            win_xright_high = rightx_cur + margin

            # 각 윈도우 안 non-zero 픽셀 인덱스
            good_left  = ((nz_y >= win_y_low) & (nz_y < win_y_high) &
                        (nz_x >= win_xleft_low) & (nz_x < win_xleft_high)).nonzero()[0]
            good_right = ((nz_y >= win_y_low) & (nz_y < win_y_high) &
                        (nz_x >= win_xright_low) & (nz_x < win_xright_high)).nonzero()[0]

            left_inds.append(good_left)
            right_inds.append(good_right)
            print("left: ",good_left.size, "minpix: ", minpix)
            # print("right: ",good_right.size, "minpix: ", minpix)

            # 픽셀 수 충분하면 윈도우 중심 이동
            if good_left.size  > minpix:  leftx_cur  = int(np.mean(nz_x[good_left]))
            if good_right.size > minpix:  rightx_cur = int(np.mean(nz_x[good_right]))


            center_y = (win_y_low + win_y_high) // 2

            # 중앙점 작업
            # param = 300
            # param = 190
            param = 200
            if good_left.size > minpix and good_right.size > minpix:
                center_x = (leftx_cur + rightx_cur) // 2
                # print("all good")
            elif good_left.size  > minpix:
                center_x = leftx_cur + param
                # print("left good")
            elif good_right.size > minpix:
                center_x = rightx_cur - param
                # print("right good")
            elif self.prev_centers is not None and len(self.prev_centers) > window:
                center_x = self.prev_centers[window][0]
            else:
                continue

            centers.append((center_x, center_y))
            prev_center_x = center_x

            if draw:
                # 녹색 윈도우 사각형
                cv.rectangle(out_img, (win_xleft_low,  win_y_low),
                                    (win_xleft_high, win_y_high),  (0,255,0), 2)
                cv.rectangle(out_img, (win_xright_low, win_y_low),
                                    (win_xright_high, win_y_high), (0,255,0), 2)

        # 인덱스 병합
        left_inds  = np.concatenate(left_inds)
        right_inds = np.concatenate(right_inds)

        left_fit = right_fit = None
        plot_y   = np.linspace(0, H-1, H)

        # 중앙점 작업
        # centers = self.smooth_centers(centers, degree=2, n_sample=nwindows)

        if left_inds.size:
            lx = nz_x[left_inds]; ly = nz_y[left_inds]
            left_fit = np.polyfit(ly, lx, 2)
            if draw:
                left_fitx = left_fit[0]*plot_y**2 + left_fit[1]*plot_y + left_fit[2]
                pts = np.array([np.transpose(np.vstack([left_fitx, plot_y]))], dtype=np.int32)
                cv.polylines(out_img, pts, False, (255,0,0), 4)        # 파랑

        if right_inds.size:
            rx = nz_x[right_inds]; ry = nz_y[right_inds]
            right_fit = np.polyfit(ry, rx, 2)
            if draw:
                right_fitx = right_fit[0]*plot_y**2 + right_fit[1]*plot_y + right_fit[2]
                pts = np.array([np.transpose(np.vstack([right_fitx, plot_y]))], dtype=np.int32)
                cv.polylines(out_img, pts, False, (0,0,255), 4)        # 빨강

        if draw and centers:
            for (cx, cy) in centers:
                cv.circle(out_img, (int(cx), int(cy)), 5, (0, 255, 255), -1)  # 노란색 BGR(0,255,255)

        # return left_fit, right_fit, out_img
        return centers, out_img

    def smooth_centers(self, centers, degree=2, n_sample=None):
        """
        centers : [(x,y), ...]  # noisy raw pts  (y=0 상단, y=H 하단)
        degree  : 1 → 직선, 2 → 2차, 3 이상 자유
        n_sample: 재생성할 점 수 (None이면 원본 개수 유지)

        return   : [(x_smooth, y)]  # same order (상단→하단)
        """
        if len(centers) < degree + 1:   # 피팅 불가
            return centers

        # 1) y·x 배열 분리  (y를 독립변수로 쓰는게 안정적)
        centers_sorted = sorted(centers, key=lambda p: p[1])   # y 기준 오름차순
        xs = np.array([p[0] for p in centers_sorted])
        ys = np.array([p[1] for p in centers_sorted])

        # 2) polyfit  (x = a*y^2 + b*y + c)
        coeff = np.polyfit(ys, xs, deg=degree)

        # 3) 원하는 y 위치로 재샘플
        if n_sample is None:
            y_out = ys                               # 원래 개수
        else:
            # y_out = np.linspace(ys.min(), ys.max(), n_sample)
            # print(ys.min(), ys.max())
            y_out = np.linspace(0, 640, n_sample)

        x_out = np.polyval(coeff, y_out)
        smooth_pts = list(map(tuple, np.stack([x_out, y_out], axis=1)))

        return smooth_pts

# -DRONE-TELLO-
🚁 PYTHON PROGRAM SIMULATES AUTONOMY 🚁


```
classDiagram
    class TelloController {
        +width: int
        +height: int
        +area_min: float
        +threshold_x: int
        +threshold_y: int
        +flying: boolean
        +tracking_enabled: boolean
        +red_lower1: array
        +red_upper1: array
        +red_lower2: array
        +red_upper2: array
        +main()
        +clean_exit(drone)
    }

    class Tello {
        +connect()
        +get_battery()
        +streamon()
        +streamoff()
        +get_frame_read()
        +send_rc_control(left_right, forward_back, up_down, yaw)
        +takeoff()
        +land()
        +get_height()
        +end()
    }

    class FrameProcessor {
        +resize(frame, width, height)
        +convert_to_hsv(frame)
        +create_mask(hsv, lower, upper)
        +filter_noise(mask)
        +find_contours(mask)
        +draw_interface(frame, drone, tracking_enabled)
    }

    class ObjectTracker {
        +area_min: float
        +threshold_x: int
        +threshold_y: int
        +forward_threshold: int
        +find_largest_contour(contours)
        +calculate_center(contour)
        +calculate_control_signals(center_x, center_y, area)
    }

    class KeyboardController {
        +process_keyboard_input(key)
        +activate_tracking()
        +deactivate_tracking()
        +manual_control(key)
    }

    TelloController --> Tello : uses
    TelloController --> FrameProcessor : processes frames with
    TelloController --> ObjectTracker : tracks objects with
    TelloController --> KeyboardController : handles input with
    Tello -- FrameProcessor : provides frames to
    ObjectTracker -- FrameProcessor : analyzes frames from


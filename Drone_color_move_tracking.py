from djitellopy import Tello
import cv2
import numpy as np
import time

# Configuración de dimensiones y parámetros
width = 800
height = 600
area_min = 0.02 * (width * height)  # Área mínima para detectar objetos
threshold_x = int(0.05 * width)      # Umbral horizontal para centrar el objeto
threshold_y = int(0.075 * height)    # Umbral vertical para centrar el objeto

# Variables globales
flying = False
tracking_enabled = True  # Tracking habilitado por defecto

# Valores HSV para color rojo
# El rojo en HSV tiene dos rangos (cerca de 0° y cerca de 180°)
# Usamos dos máscaras para capturar ambos rangos
red_lower1 = np.array([0, 120, 70])       # Primer rango de rojo (cerca de 0°)
red_upper1 = np.array([10, 255, 255])
red_lower2 = np.array([160, 120, 70])     # Segundo rango de rojo (cerca de 180°)
red_upper2 = np.array([180, 255, 255])

# Función para salida limpia
def clean_exit(drone):
    global flying
    print("\nCerrando el programa...")
    if flying:
        drone.send_rc_control(0, 0, 0, 0)
        time.sleep(0.5)
        drone.land()
    cv2.destroyAllWindows()
    drone.streamoff()
    drone.end() 
    print("Programa cerrado correctamente.")

def main():
    global flying, tracking_enabled
    
    # Conectando al dron Tello
    print("Iniciando conexión con el dron Tello...")
    drone = Tello()
    drone.connect()
    battery = drone.get_battery()
    print(f"Conexión establecida. Batería actual: {battery}%")
    
    # Comprobar nivel de batería antes de iniciar
    if battery < 20:
        print("ADVERTENCIA: Nivel de batería bajo. Cargue el dron antes de volar.")
        return
    
    # Iniciando stream de video
    print("Iniciando transmisión de video...")
    drone.streamon()
    time.sleep(3)
    
    # Variables para el control de velocidad
    fb_vel_forward_back = 0
    fb_vel_left_right = 0
    fb_vel_up_down = 0
    fb_vel_yaw = 0
    
    # Variables para controlar la distancia al objeto
    forward_threshold = 15000  # Área para considerar que el dron está a buena distancia
    
    # Variable para registrar si se ha encontrado un objeto
    object_found = False
    no_object_counter = 0  # Contador para cuando no se detecta objeto
    
    print("Sistema listo. Comandos:")
    print("  i - Despegar")
    print("  k - Aterrizar")
    print("  t - Activar/desactivar seguimiento")
    print("  q - Salir")
    print("  w/a/s/d - Movimiento manual adelante/izquierda/atrás/derecha")
    print("  e/q - Subir/bajar")
    print("  z/c - Girar izquierda/derecha")
    
    try:
        while True:
            # Obtener frame del dron
            frame = drone.get_frame_read().frame
            if frame is None:
                continue
                
            # Redimensionar el frame
            frame = cv2.resize(frame, (width, height))
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            # Convertir a HSV para la detección de color
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Crear máscaras para el color rojo (dos rangos)
            mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
            mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
            
            # Combinar las dos máscaras de rojo
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Aplicar desenfoque gaussiano
            blurred = cv2.GaussianBlur(mask, (15, 15), 0)
            
            # Erosión y dilatación para eliminar ruido
            mask = cv2.erode(blurred, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Aplicar máscara a la imagen original
            result = cv2.bitwise_and(frame, frame, mask=mask)
            
            # Mostrar información del dron
            cv2.putText(frame, "Drone Camera", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Battery: {drone.get_battery()}%", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Height: {drone.get_height()} cm", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Verificar batería baja
            if drone.get_battery() < 15:
                cv2.putText(frame, "BATERÍA BAJA! ATERRIZANDO", (50, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
                if flying:
                    drone.land()
                    flying = False
                    print("Batería baja. Aterrizaje de seguridad completado.")
            
            # Dibujar líneas de referencia
            cv2.line(frame, (width//2 - threshold_x, 0), (width//2 - threshold_x, height), (255,0,0), thickness=2)
            cv2.line(frame, (width//2 + threshold_x, 0), (width//2 + threshold_x, height), (255,0,0), thickness=2)
            cv2.line(frame, (0, height//2 - threshold_y), (width, height//2 - threshold_y), (255,0,0), thickness=2)
            cv2.line(frame, (0, height//2 + threshold_y), (width, height//2 + threshold_y), (255,0,0), thickness=2)
            
            # Encontrar contornos según el filtro de color
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
            # Resetear flag de objeto encontrado
            object_found = False
            
            # Encontrar el contorno más grande (probablemente el objeto rojo principal)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > area_min:
                    object_found = True
                    no_object_counter = 0
                    
                    # Dibujar contorno y rectángulo
                    cv2.drawContours(frame, [largest_contour], -1, (255, 0, 255), 3)
                    perimeter = cv2.arcLength(largest_contour, True)
                    approx = cv2.approxPolyDP(largest_contour, 0.02 * perimeter, True)
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Calcular centro del objeto
                    center_x = x + w // 2
                    center_y = y + h // 2
                    center = (center_x, center_y)
                    cv2.circle(frame, center, 5, (0, 0, 255), cv2.FILLED)
                    
                    # Mostrar información del objeto
                    cv2.putText(frame, f"Area: {int(area)}", (x, y-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Control automático si el tracking está habilitado y el dron está volando
                    if tracking_enabled and flying:
                        # Reset velocidades
                        fb_vel_forward_back = 0
                        fb_vel_left_right = 0
                        fb_vel_up_down = 0
                        fb_vel_yaw = 0
                        error_x = center_x - (width//2)
                        error_y = center_y - (height//2)
                        Kp = 0.09  # Ganancia proporcional para el control   
                        # Control horizontal (izquierda/derecha)
                        
                        if center_x < width//2 - threshold_x:
                            cv2.putText(frame, "IZQUIERDA", (10, 150), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            fb_vel_yaw = int(Kp * error_x)

                        elif center_x > width//2 + threshold_x:
                            cv2.putText(frame, "DERECHA", (10, 150), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            fb_vel_yaw = int(Kp * error_x)
                        else:
                            cv2.putText(frame, "CENTRO", (10, 150), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                    
                        
                        # Control vertical (arriba/abajo)
               
                        if center_y < height//2 - threshold_y:
                            cv2.putText(frame, "SUBIR", (10, 180), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            fb_vel_up_down = 20 #int(Kp * error_y)   # Subir
                        elif center_y > height//2 + threshold_y:
                            cv2.putText(frame, "BAJAR", (10, 180), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            fb_vel_up_down = -20 #int(Kp * error_y)   # Bajar
                        else:
                            cv2.putText(frame, "CENTRO", (10, 180), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        # Control de distancia (adelante/atrás)
                        if area < forward_threshold:
                            cv2.putText(frame, "Objeto lejos: Acercarse", (10, 210), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            fb_vel_forward_back = 10  # Avanzar
                        elif area > forward_threshold * 1.5:
                            cv2.putText(frame, "Objeto cerca: Alejarse", (10, 210), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            fb_vel_forward_back = -10  # Retroceder
                        else:
                            cv2.putText(frame, "Distancia óptima", (10, 210), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
            # Si no se encuentra objeto durante varios frames, mantener la posición
            if not object_found:
                no_object_counter += 1
                
                # Si llevamos muchos frames sin encontrar objeto, detenemos el dron
                if no_object_counter > 10 and tracking_enabled and flying:
                    fb_vel_forward_back = 0
                    fb_vel_left_right = 0
                    fb_vel_up_down = 0
                    fb_vel_yaw = 0
                    cv2.putText(frame, "No se detecta objeto rojo", (width//2-150, height//2), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Mostrar estado del tracking
            if tracking_enabled:
                cv2.putText(frame, "SEGUIMIENTO ACTIVO", (width-300, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "SEGUIMIENTO DESACTIVADO", (width-340, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Mostrar imágenes
            cv2.imshow('Drone Camera', frame)
            cv2.imshow('Mask', mask)
            cv2.imshow('Filtrado', result)
            
            # Esperar por entrada de teclado
            key = cv2.waitKey(100) & 0xFF
            
            # Procesar comandos por teclado
            if key == ord('q'):  # Salir
                clean_exit(drone)
                break
                
            elif key == ord('i'):  # Despegar
                if not flying:
                    print("Despegando...")
                    drone.takeoff()
                    flying = True
                    print("En el aire.")
                    
            elif key == ord('k'):  # Aterrizar
                if flying:
                    print("Aterrizando...")
                    drone.land()
                    flying = False
                    print("Aterrizaje completado.")
                    
            elif key == ord('t'):  # Activar/desactivar seguimiento
                tracking_enabled = not tracking_enabled
                if tracking_enabled:
                    print("Seguimiento de objetos rojos ACTIVADO")
                else:
                    print("Seguimiento de objetos rojos DESACTIVADO")
                    # Detener movimiento si se desactiva el seguimiento
                    drone.send_rc_control(0, 0, 0, 0)
            
            # Control manual cuando el seguimiento está desactivado
            if not tracking_enabled:
                if key == ord('w'):
                    fb_vel_forward_back = 30
                elif key == ord('s'):
                    fb_vel_forward_back = -30
                else:
                    fb_vel_forward_back = 0
                
                if key == ord('a'):
                    fb_vel_left_right = -30
                elif key == ord('d'):
                    fb_vel_left_right = 30
                else:
                    fb_vel_left_right = 0
                
                if key == ord('e') and drone.get_height() < 300:
                    fb_vel_up_down = 30
                elif key == ord('q'):
                    fb_vel_up_down = -30
                else:
                    fb_vel_up_down = 0
                
                if key == ord('z'):
                    fb_vel_yaw = 30
                elif key == ord('c'):
                    fb_vel_yaw = -30
                else:
                    fb_vel_yaw = 0
            
            # Enviar comandos al dron
            if flying:
                drone.send_rc_control(fb_vel_left_right, fb_vel_forward_back, fb_vel_up_down, fb_vel_yaw)
                
    except KeyboardInterrupt:
        print("Programa interrumpido por el usuario")
        clean_exit(drone)
    except Exception as e:
        print(f"Error: {e}")
        clean_exit(drone)

if __name__ == '__main__':
    main()
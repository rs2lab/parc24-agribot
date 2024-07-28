############################################################
################  Ajuda a ajustar as cores #################
############################################################

import cv2
import numpy as np

def adjust_hsv_limits(image):
    def nothing(x):
        pass

    # Converter a imagem para o espaço de cores HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Criar uma janela
    cv2.namedWindow('Trackbars')

    # Criar sliders para ajustar os valores de HSV
    cv2.createTrackbar('Hue Min', 'Trackbars', 0, 179, nothing)
    cv2.createTrackbar('Hue Max', 'Trackbars', 10, 179, nothing)
    cv2.createTrackbar('Sat Min', 'Trackbars', 120, 255, nothing)
    cv2.createTrackbar('Sat Max', 'Trackbars', 255, 255, nothing)
    cv2.createTrackbar('Val Min', 'Trackbars', 70, 255, nothing)
    cv2.createTrackbar('Val Max', 'Trackbars', 255, 255, nothing)

    while True:
        # Capturar os valores atuais dos sliders
        h_min = cv2.getTrackbarPos('Hue Min', 'Trackbars')
        h_max = cv2.getTrackbarPos('Hue Max', 'Trackbars')
        s_min = cv2.getTrackbarPos('Sat Min', 'Trackbars')
        s_max = cv2.getTrackbarPos('Sat Max', 'Trackbars')
        v_min = cv2.getTrackbarPos('Val Min', 'Trackbars')
        v_max = cv2.getTrackbarPos('Val Max', 'Trackbars')

        # Definir os limites de cor com base nos sliders
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # Aplicar a máscara
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    
        # Mostrar a máscara resultante
        cv2.imshow('Mask', mask)
    
        if cv2.waitKey(1) & 0xFF == 27:  # Pressione 'ESC' para sair
            break

    cv2.destroyAllWindows()

    return lower_bound, upper_bound

def detect_tomatoes(image_path):
    # Carregar a imagem
    image = cv2.imread(image_path)
    if image is None:
        print("Erro ao carregar a imagem.")
        return

    # Ajustar interativamente os limites HSV
    lower_bound, upper_bound = adjust_hsv_limits(image)

    # Converter a imagem para o espaço de cores HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Aplicar a máscara para filtrar as regiões vermelhas
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    # Aplicar filtro de desfoque para reduzir o ruído
    blurred = cv2.GaussianBlur(mask, (9, 9), 10)

    # Detectar círculos usando a Transformada de Hough
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                               param1=100, param2=30, minRadius=10, maxRadius=50)

    # Aplicar a segmentação K-means
    kmeans = kmeans_segmentation(hsv_image)

    # Se círculos forem detectados
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # Desenhar o círculo na imagem original
            cv2.circle(image, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # Desenhar o centro do círculo
            cv2.circle(image, (i[0], i[1]), 2, (0, 0, 255), 3)

    # Contar o número de tomates detectados
    tomato_count = len(circles[0]) if circles is not None else 0
    print(f"Número de tomates detectados: {tomato_count}")

    # Mostrar a imagem original com os círculos detectados
    cv2.imshow('Imagem com Tomates Detectados', image)
    cv2.imshow('HSV Blur', blurred)
    cv2.imshow('Imagem K-means', kmeans)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def kmeans_segmentation(image, attempts=10, k=4, criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)):
    td_img = np.float32(image.reshape((-1, 3)))
    _, label, center = cv2.kmeans(td_img, k, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)

    center = np.uint8(center)
    res = center[label.flatten()]

    return res.reshape((image.shape))

# Exemplo de uso
if __name__ == "__main__":
    detect_tomatoes('captur_image/images/right_image_149.png')

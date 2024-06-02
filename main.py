import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull


# Определяет, находится ли точка p внутри полуплоскости, образованной ребром (cp1, cp2)
def inside(p, cp1, cp2):
    return (cp2[0] - cp1[0]) * (p[1] - cp1[1]) > (cp2[1] - cp1[1]) * (p[0] - cp1[0])


# Вычисляет точку пересечения двух отрезков (cp1, cp2) и (s, e)
def intersection(cp1, cp2, s, e):
    dc = (cp1[0] - cp2[0], cp1[1] - cp2[1])
    dp = (s[0] - e[0], s[1] - e[1])
    n1 = cp1[0] * cp2[1] - cp1[1] * cp2[0]
    n2 = s[0] * e[1] - s[1] * e[0]
    n3 = 1.0 / (dc[0] * dp[1] - dc[1] * dp[0])
    return (n1 * dp[0] - n2 * dc[0]) * n3, (n1 * dp[1] - n2 * dc[1]) * n3


# Алгоритм Sutherland-Hodgman для отсечения многоугольника
def sutherland_hodgman(subject_pol, clip_pol):
    output_list = subject_pol
    cp1 = clip_pol[-1]
    for cp2 in clip_pol:
        input_list = output_list
        output_list = []
        s = input_list[-1]
        for e in input_list:
            if inside(e, cp1, cp2):
                if not inside(s, cp1, cp2):
                    output_list.append(intersection(cp1, cp2, s, e))
                output_list.append(e)
            elif inside(s, cp1, cp2):
                output_list.append(intersection(cp1, cp2, s, e))
            s = e
        cp1 = cp2
    return output_list


# Функция для отрисовки многоугольников
def plot_polygons(subject_pol, clip_pol, clipped_pol):
    plt.figure()
    plt.fill(*zip(*subject_pol), edgecolor='r', fill=False, linewidth=2, label='Subject Polygon')
    plt.scatter(*zip(*subject_pol), color='r', zorder=5)
    plt.fill(*zip(*clip_pol), edgecolor='b', fill=False, linewidth=2, label='Clip Polygon')
    plt.scatter(*zip(*clip_pol), color='b', zorder=5)
    plt.fill(*zip(*clipped_pol), edgecolor='g', fill=False, linewidth=2, label='Clipped Polygon')
    plt.scatter(*zip(*clipped_pol), color='g', zorder=5)
    plt.legend()
    plt.title('Sutherland-Hodgman Polygon Clipping')
    plt.show()


# Генерация выпуклого многоугольника с использованием ConvexHull
def generate_convex_polygon(n):
    points = np.random.rand(n, 2) * 100
    hull = ConvexHull(points)
    return points[hull.vertices]


# Генерация простого многоугольника в заданной области
def generate_simple_polygon(n, bbox):
    angle = np.linspace(0, 2 * np.pi, n, endpoint=False)
    radius = np.random.rand(n) * (max(bbox[1] - bbox[0], bbox[3] - bbox[2]) * 0.5)
    points = np.vstack((radius * np.cos(angle), radius * np.sin(angle))).T
    center = np.array([(bbox[0] + bbox[1]) / 2, (bbox[2] + bbox[3]) / 2])
    points += center
    return points


# Ручной ввод координат
def manual_input(n_subject, n_clip):
    subject_polygon = []
    clip_polygon = []

    print(f"Введите {n_subject} пар координат для произвольного полигона:")
    for i in range(n_subject):
        x = float(input(f"Введите координату x{i + 1} для произвольного полигона: "))
        y = float(input(f"Введите координату y{i + 1} для произвольного полигона: "))
        subject_polygon.append((x, y))

    print(f"Введите {n_clip} пар координат для выпуклого отсекателя:")
    for i in range(n_clip):
        x = float(input(f"Введите координату x{i + 1} для выпуклого отсекателя: "))
        y = float(input(f"Введите координату y{i + 1} для выпуклого отсекателя: "))
        clip_polygon.append((x, y))

    return subject_polygon, clip_polygon


# Генерация многоугольников
def generate_polygons(n_subject, n_clip):
    clip_polygon = generate_convex_polygon(n_clip)
    bbox = [np.min(clip_polygon[:, 0]) - 50, np.max(clip_polygon[:, 0]) + 50, np.min(clip_polygon[:, 1]) - 50,
            np.max(clip_polygon[:, 1]) + 50]
    subject_polygon = generate_simple_polygon(n_subject, bbox)
    return subject_polygon, clip_polygon


# Основная функция
def main():
    n_clip = 5
    n_subject = 4

    while True:
        choose = input(
            "выберите метод ввода координат:\n\t1. ручной ввод координат.\n\t2. автоматическая генерация координат.\n")
        if choose == "1":
            subject_polygon, clip_polygon = manual_input(n_subject, n_clip)
            break
        elif choose == "2":
            subject_polygon, clip_polygon = generate_polygons(n_subject, n_clip)
            break
        else:
            print("неправильный формат ввода\n")

    print("Clip Polygon Points:")
    for point in clip_polygon:
        print(f"({point[0]}, {point[1]})")

    print("Subject Polygon Points:")
    for point in subject_polygon:
        print(f"({point[0]}, {point[1]})")

    # Применение алгоритма отсечения
    clipped_polygon = sutherland_hodgman(subject_polygon, clip_polygon)

    # Вывод координат вершин многоугольников
    print("Clipped Polygon Points:")
    for point in clipped_polygon:
        print(f"({point[0]}, {point[1]})")

    # Отрисовка всех многоугольников
    plot_polygons(subject_polygon, clip_polygon, clipped_polygon)


if __name__ == "__main__":
    main()

import numpy as np
import math
import pygame
import time


# ray – x, y, z, x_dir, y_dir, z_dir
# sphere – x, y, z, size
# vect_type = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
# ray_type = np.dtype([('origin', vect_type), ('direction', vect_type)])
# sphere_type = np.dtype([('position', vect_type), ('size', 'f4')])

# ray_test = np.array([((0, 0, 0), (0, 0, 1))], dtype=ray_type)
# print(ray_test['origin'])


def intersects(sphere, og_ray) -> float:
    # ray = [og_ray[0] - sphere[0], og_ray[1] - sphere[1], og_ray[2] - sphere[2], og_ray[3], og_ray[4]]
    # ray = [og_ray['origin']['x'] - sphere[0], og_ray['origin']['y'] - sphere[1], og_ray['origin']['z'] - sphere[2], og_ray['direction']['x'], og_ray['direction']['y']]
    ray = og_ray[0] - sphere[0]
    a = np.dot(og_ray[1], og_ray[1])

    b = 2.0 * np.dot(ray, og_ray[1])
    c = np.dot(ray, ray) - sphere[1] * sphere[1]
    # print(b)
    discriminant = b * b - 4 * a * c
    # print(discriminant)
    if discriminant <= 0:
        return -1
    else:
        return (-b - np.sqrt(discriminant)) / (2.0 * a)

def shoot_ray(ray, scene, samples = 5):
    pass


def generate_spheres():
    spheres = list()
    spheres.append([[3., 3., 1.], 1.9])
    spheres.append([[0., 0., 2.0], 0.2])
    spheres = np.array(spheres)
    print(spheres)
    return spheres



def main():
    print("LOLLL")
    spheres = generate_spheres()
    pygame.init()
    w = 360
    h = 360
    size = (w, h)
    screen = pygame.display.set_mode(size)
    for s in range(0, 165):
        screen.fill((0, 0, 0))
        spheres[0][0][2] = s * 3
        surf = np.zeros((w, h, 3), dtype=np.float)
        for i in range(1, w):
            for j in range(1, h):
                ray_test = np.array([[0, 0, 0], [(i - 180) * 0.0174533, (j - 180) * 0.0174533, 1]])
                intersection = intersects(spheres[0], ray_test)
                # spherey_shit[i, j] = np.array([180, 180, 180]) * (-intersection)
                if intersection != -1:
                    # print(intersection)
                    surf[i, j] += np.array([160, 180, 190]) * (-intersection)
                # if intersects(spheres[1], [0, 0, -1, i * 0.0174533, j * 0.0174533]):
                #     surf[i, j] += 0.2
        surf_v = pygame.surfarray.make_surface(surf)
        screen.fill((0, 0, 0))
        screen.blit(surf_v, (0, 0))
        pygame.display.flip()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
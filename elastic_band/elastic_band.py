import math

class Elastic_Band :

    def __init__(self, path, obs, max_iters=100, k=1, d_min=0.4, d_max=0.7, delta_t=0.1) :
        self.path = path
        self.obs = obs
        self.max_iters = max_iters
        self.k= k
        self.d_min = d_min
        self.d_max = d_max
        self.delta_t = delta_t

    def elastic_band_path_planning(self):
        # Initialize the elastic band
        band = []
        for point in self.path:
            band.append(point)
        band.insert(0, [self.path[0][0], self.path[0][1]])
        band.append([self.path[-1][0], self.path[-1][1]])
        n = len(band)


        # Main loop
        for i in range(self.max_iters):
            # Compute the distance to obstacles
            dist = []
            for j in range(n):
                dist.append(self.obs_distance(band[j], self.obs))

            # Compute the elastic band forces
            force = [[0, 0, 0]] * n
            for j in range(1, n - 1):
                d = math.sqrt((band[j+1][0] - band[j-1][0])**2 + (band[j+1][1] - band[j-1][1])**2)
                f =  [self.k * (d - self.delta_t) * (band[j+1][0] - 2 * band[j][0] + band[j-1][0])/d, self.k * (d - self.delta_t) *(band[j+1][1] - 2 * band[j][1] + band[j-1][1])/d, 0]
                force[j] = f

            for j in range(n):
                force[j] = [max(-0.5, min(0.5, force[j][0])), max(-0.5, min(0.5, force[j][1])), 0]
                None

            # Compute the repulsive forces from obstacles
            repulse = [[0, 0, 0]] * n
            for j in range(1, n - 1):
                if dist[j] < self.d_min:
                    repulse[j] = [(self.d_min - dist[j]) * g for g in self.obs_gradient(band[j][:2], self.obs)]
                elif self.d_min <= dist[j] < self.d_max:
                    repulse[j] = [(self.d_max - dist[j]) * g for g in self.obs_gradient(band[j][:2], self.obs)]
            for j in range(n):
                repulse[j] = [max(-0.5, min(0.5, repulse[j][0])), max(-0.5, min(0.5, repulse[j][1])), 0]
                None

            # Update the band
            for j in range(n):
                band[j][0] += self.delta_t * force[j][0]
                band[j][1] += self.delta_t * force[j][1]
                band[j][0] += self.delta_t * repulse[j][0]
                band[j][1] += self.delta_t * repulse[j][1]

        # Return the updated path
        return [point[:2] for point in band[1:-1]]

    def obs_distance(self, p, obs):
        """
        Compute the distance from point p to the nearest obstacle
        """
        return min([math.sqrt((p[0] - o[0])**2 + (p[1] - o[1])**2) for o in obs])

    def obs_gradient(self, p, obs):
        """
        Compute the gradient of the distance function around point p
        """
        epsilon = 0.1
        dx = self.obs_distance([p[0] + epsilon, p[1]], obs) - self.obs_distance([p[0] - epsilon, p[1]], obs)
        dy = self.obs_distance([p[0], p[1] + epsilon], obs) - self.obs_distance([p[0], p[1] - epsilon], obs)
        return [dx / (2 * epsilon), dy / (2 * epsilon), 0]


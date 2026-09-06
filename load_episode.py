import imageio
import numpy as np

# z = np.load("outputs/data/ep_0000.npz")
# print({k: z[k].shape for k in z})
# imageio.mimsave("outputs/gifs/head0.gif", z["head_cam"], fps=10)
# imageio.mimsave("outputs/gifs/wrist0.gif", z["left_wrist_cam"], fps=10)

for i in range(6):
    z = np.load(f"outputs/data/ep_000{i}.npz")
    print({k: z[k].shape for k in z})
    imageio.mimsave(f"outputs/gifs/head{i}.gif", z["head_cam"], fps=10)
    imageio.mimsave(f"outputs/gifs/wrist{i}.gif", z["left_wrist_cam"], fps=10)
    # imageio.mimsave("outputs/gifs/wrist0.gif", z["left_wrist_cam"], fps=10)

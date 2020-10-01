# Copyright (c) 2018 Dynamic Robotics Laboratory
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from cassiemujoco import *
import time
import numpy as np
import math


def lerp_keyframes(keyframes, N):
    t = np.linspace(0, len(keyframes)-1, N, endpoint=True)
    i0 = np.floor(t)
    i1 = np.ceil(t)
    w = (i1 - t)[:, np.newaxis]
    out = w*keyframes[i0.astype(np.int)] + (1-w)*keyframes[i1.astype(np.int)]
    return out

# Initialize cassie simulation
sim = CassieSim("../model/cassie_hfield.xml")

print(sim.get_hfield_ncol())
print(sim.get_hfield_nrow())
print(sim.get_nhfielddata())
print(sim.get_hfield_size())
data = np.zeros((sim.get_hfield_nrow(), sim.get_hfield_ncol()))
sim.set_hfield_data(data.flatten())
hfield_data = sim.get_hfield_data()
vis = CassieVis(sim, "../model/cassie_hfield.xml")

t = time.monotonic()
count = 0

draw_state = vis.draw(sim)

keyframes = np.fromfile('keyframes_even.bin', dtype=float)
keyframes = keyframes.reshape((-1, 35))
keyframes = lerp_keyframes(keyframes, 25)
print(keyframes.shape)

sim.hold()
while draw_state and count < len(keyframes):
    if not vis.ispaused():
        sim.set_qpos(keyframes[count])
        count += 1

    draw_state = vis.draw(sim)
    time.sleep(1.0)
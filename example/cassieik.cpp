/*
 * Copyright (c) 2018 Dynamic Robotics Laboratory
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "cassiemujoco.h"
#include <iostream>

using namespace std;


int main(void)
{
    const char modelfile[] = "../model/cassie_hfield.xml";
    cassie_sim_t *c = cassie_sim_init(modelfile, false);
    cassie_vis_t *v = cassie_vis_init(c, modelfile);
    // disable gravity
    c->m->opt.gravity[2] = 0.f;

    state_out_t y;
    pd_in_t u = {0};
    int count = 0;
    float* hfield_data = cassie_sim_hfielddata(c);
    int nhfielddata = cassie_sim_get_nhfielddata(c);
    for (int i = 0; i < nhfielddata; i++) hfield_data[i] = 0.f;

    bool draw_state = cassie_vis_draw(v, c);
    while (draw_state) {
        if (!cassie_vis_paused(v)) {   
            cassie_sim_step_pd(c, &y, &u);
            count += 1;
        }

        int N(2);
        double footp[6];
        cassie_sim_foot_positions(c, footp);
        for (size_t i=0; i<N; i++)
        {
            if (v->nMarkers < v->maxnMarkers) {
              cassie_vis_marker_t *m = v->markers + v->nMarkers++;
              for (size_t j=0; j<3; j++) m->pos[j] = float(footp[3*i+j]);
              m->rgba[0] = 1.f;
              m->rgba[1] = 0.f;
              m->rgba[2] = 0.f;
              m->rgba[3] = 1.f;
              m->size = 0.05f;
            }
        }
        draw_state = cassie_vis_draw(v, c);
    }

    cassie_sim_free(c);
    cassie_vis_free(v);

    return 0;
}

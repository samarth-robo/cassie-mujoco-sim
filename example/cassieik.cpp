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
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Matrix4;

Vector2d project(const Matrix4 &projection, const Matrix4 &modelView,
    const Vector4d &viewport, const Vector4d &p) {
  Vector4d pp = projection * modelView * p;
  pp.head(3) /= pp(3);
  Vector2d out;
  out << (pp(0) + 1.0) * viewport(2) / 2.0 + viewport(0),
         (pp(1) + 1.0) * viewport(3) / 2.0 + viewport(1);
  return out;
}

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

    mjvFigure figjoints;
    mjv_defaultFigure(&figjoints);
    figjoints.flg_legend = 0;
    figjoints.flg_ticklabel[0] = 0;
    figjoints.flg_ticklabel[1] = 0;
    figjoints.linepnt[0] = 2;
    figjoints.linedata[0][0] = 100;
    figjoints.linedata[0][1] = 100;
    figjoints.linedata[0][2] = 200;
    figjoints.linedata[0][3] = 200;
    figjoints.linergb[0][0] = 1.0;
    figjoints.linergb[0][1] = 0.0;
    figjoints.linergb[0][2] = 0.0;
    figjoints.panergba[0] = 0.0;
    figjoints.panergba[1] = 0.0;
    figjoints.panergba[2] = 0.0;
    figjoints.panergba[3] = 0.0;
    figjoints.figurergba[0] = 0.0;
    figjoints.figurergba[1] = 0.0;
    figjoints.figurergba[2] = 0.0;
    figjoints.figurergba[3] = 0.0;
    mjrRect joints_viewport = {v->viewport[0], v->viewport[1], v->viewport[2], v->viewport[3]};
    
    bool draw_state = cassie_vis_draw(v, c);
    while (draw_state) {
        if (!cassie_vis_paused(v)) {   
            cassie_sim_step_pd(c, &y, &u);
            count += 1;
        }
        draw_state = cassie_vis_draw(v, c);
        
        Matrix4 projection, modelView;
        for (size_t i=0; i<16; i++) {
          projection(i) = v->projection[i];
          modelView(i) = v->modelView[i];
        }
        Vector4d viewport;
        for (size_t i=0; i<4; i++) viewport[i] = v->viewport[i];

        Vector4d p = Vector4d::Zero();
        p(3) = 1.0;
        auto pp = project(projection, modelView, viewport, p);
        
        glfwMakeContextCurrent(v->window);
        mjr_figure(joints_viewport, &figjoints, &v->con);
        glfwSwapBuffers(v->window);
    }

    cassie_sim_free(c);
    cassie_vis_free(v);

    return 0;
}

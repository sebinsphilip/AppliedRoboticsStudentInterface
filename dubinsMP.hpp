#ifndef DUBINS_MP_HPP
#define DUBINS_MP_HPP

#include "stdio.h"
#include <math.h>
#include <iostream>
#include <limits>

#include "utils.hpp"

#define _USE_MATH_DEFINES
#define DUBIN_DEBUG 1

typedef enum
{
    LSL = 0,
    RSR = 1,
    LSR = 2,
    RSL = 3,
    RLR = 4,
    LRL = 5
} DubinsPathType;

int ksigns [][3] = {{1, 0, 1},
                    {-1, 0, -1},
                    {1, 0, -1},
                    {-1, 0, 1},
                    {-1, 1, -1},
                    {1, -1, 1},};

float mod2pi (float ang);

int __LSL (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3);
int __RSR (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3);
int __LSR (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3);
int __RSL (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3);
int __RLR (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3);
int __LRL (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3);
int (*OptimalCurves[]) (float const, float const, float const, float&, float&, float&) = {__LSL, __RSR, __LSR, __RSL, __RLR, __LRL};


void scaleFromStandard (float const lambda, float const sc_s1, float const sc_s2, float const sc_s3, float &s1, float &s2, float &s3);
void scaleToStandard (float x0, float y0, float th0, float xf, float yf, float thf, int Kmax,
        float &sc_th0, float &sc_thf, float &sc_Kmax, float &lambda);
float sinc (float const t);
void circline (float const s, float const x0, float const y0, float const k, float const th0,
        Pose &ps);
void dubinsarc (float const x0, float const y0, float const th0, float const k, float const L,
       Path &pth);
void dubinscurve (float const x0, float const y0, float const th0, float const s1, float const s2, float const s3, float const k0, float const k1, float const k2,
       Path &pth1, Path &pth2, Path &pth3, float L);
void dubins (float x0, float y0, float th0, float xf, float yf, float thf, int Kmax,
        DubinsPathType &path, Path &pth1, Path &pth2, Path &pth3, float &Ltotal);

#endif

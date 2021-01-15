#include "dubinsMP.hpp"

float mod2pi (float ang)
{
    float out = ang;
    while (out<0)
        out += 2*M_PI;
    while (out >= 2*M_PI)
        out -= 2*M_PI;
    return out;
}

int __LSL (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3)
{
    float invK = 1 / sc_Kmax;
    float C = cos(sc_thf) - cos(sc_th0);
    float S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    float temp1 = atan2(C, S);
    sc_s1 = invK * mod2pi(temp1 - sc_th0);
    float temp2 = 2+ 4*pow(sc_Kmax, 2) -2*cos(sc_th0 - sc_thf) + 4 * sc_Kmax*(sin(sc_th0) - sin(sc_thf));
    int ok = 0;
    if (temp2 < 0)
    {
        ok = 0;
        sc_s1 = 0;
        sc_s2 = 0;
        sc_s3 = 0;
        return ok;
    }
    sc_s2 = invK * sqrt(temp2);
    sc_s3 = invK * mod2pi(sc_thf-temp1);
    ok = 1;
    return ok;
}

int __RSR (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3)
{
    float invK = 1 / sc_Kmax;
    float C = cos(sc_th0) - cos(sc_thf);
    float S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    float temp1 = atan2(C, S);
    sc_s1 = invK * mod2pi(sc_th0-temp1);
    float temp2 = 2+ 4*pow(sc_Kmax, 2) -2*cos(sc_th0 - sc_thf) - 4 * sc_Kmax*(sin(sc_th0) - sin(sc_thf));
    int ok = 0;
    if (temp2 < 0)
    {
        ok = 0;
        sc_s1 = 0;
        sc_s2 = 0;
        sc_s3 = 0;
        return ok;
    }
    sc_s2 = invK * sqrt(temp2);
    sc_s3 = invK * mod2pi(temp1-sc_thf);
    ok = 1;
    return ok;
}

int __LSR (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3)
{
    float invK = 1 / sc_Kmax;
    float C = cos(sc_th0) + cos(sc_thf);
    float S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
    float temp1 = atan2(-C, S);
    float temp3 = 4*pow(sc_Kmax,2)-2 +2*cos(sc_th0 - sc_thf) + 4 * sc_Kmax*(sin(sc_th0) + sin(sc_thf));
    int ok = 0;
    if (temp3 < 0)
    {
        ok = 0;
        sc_s1 = 0;
        sc_s2 = 0;
        sc_s3 = 0;
        return ok;
    }
    sc_s2 = invK * sqrt(temp3);
    float temp2 = -atan2(-2,sc_s2*sc_Kmax);
    sc_s1 = invK * mod2pi(temp1+temp2-sc_th0);
    sc_s3 = invK * mod2pi(temp1 +temp2-sc_thf);
    ok = 1;
    return ok;
}

int __RSL (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3)
{
    float invK = 1 / sc_Kmax;
    float C = cos(sc_th0) + cos(sc_thf);
    float S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
    float temp1 = atan2(C, S);
    float temp3 = 4*pow(sc_Kmax,2) -2 +2*cos(sc_th0 - sc_thf) - 4 * sc_Kmax*(sin(sc_th0) + sin(sc_thf));
    int ok = 0;
    if (temp3 < 0)
    {
        ok = 0;
        sc_s1 = 0;
        sc_s2 = 0;
        sc_s3 = 0;
        return ok;
    }
    sc_s2 = invK * sqrt(temp3);
    float temp2 = atan2(2,sc_s2*sc_Kmax);
    sc_s1 = invK * mod2pi(sc_th0-temp1+temp2);
    sc_s3 = invK * mod2pi(sc_thf-temp1+temp2);
    ok = 1;
    return ok;
}

int __RLR (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3)
{
    float invK = 1 / sc_Kmax;
    float C = cos(sc_th0) - cos(sc_thf);
    float S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    float temp1 = atan2(C, S);
    float temp2 = 0.125*(6-4 * pow(sc_Kmax,2)  + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax*(sin(sc_th0) - sin(sc_thf)));
    int ok = 0;
    if (abs(temp2) > 1)
    {
        ok = 0;
        sc_s1 = 0;
        sc_s2 = 0;
        sc_s3 = 0;
        return ok;
    }
    sc_s2 = invK * mod2pi(2*M_PI - acos(temp2));
    sc_s1 = invK * mod2pi(sc_th0-temp1+0.5*sc_s2*sc_Kmax);
    sc_s3 = invK * mod2pi(sc_th0-sc_thf+sc_Kmax*(sc_s2-sc_s1));
    ok = 1;
    return ok;
}

int __LRL (float const sc_th0, float const sc_thf, float const sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3)
{
    float invK = 1 / sc_Kmax;
    float C = cos(sc_thf) - cos(sc_th0);
    float S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    float temp1 = atan2(C, S);
    float temp2 = 0.125*(6-4 * pow(sc_Kmax,2)  + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax*(sin(sc_th0) - sin(sc_thf)));
    int ok = 0;
    if (abs(temp2) > 1)
    {
        ok = 0;
        sc_s1 = 0;
        sc_s2 = 0;
        sc_s3 = 0;
        return ok;
    }
    sc_s2 = invK * mod2pi(2*M_PI - acos(temp2));
    sc_s1 = invK * mod2pi(temp1-sc_th0+0.5*sc_s2*sc_Kmax);
    sc_s3 = invK * mod2pi(sc_thf-sc_th0+sc_Kmax*(sc_s2-sc_s1));
    ok = 1;
    return ok;
}


void scaleFromStandard (float const lambda, float const sc_s1, float const sc_s2, float const sc_s3, float &s1, float &s2, float &s3)
{
	s1 = sc_s1 * lambda;
	s2 = sc_s2 * lambda;
	s3 = sc_s3 * lambda;
}


void scaleToStandard (float x0, float y0, float th0, float xf, float yf, float thf, int Kmax,
        float &sc_th0, float &sc_thf, float &sc_Kmax, float &lambda)
{
    float dx = xf - x0;
    float dy = yf - y0;
    float phi = atan2(dy, dx);
    lambda = hypot(dx, dy)/2;
    sc_Kmax = Kmax * lambda;
    sc_th0 = mod2pi(th0 - phi);
    sc_thf = mod2pi(thf - phi);
}

float sinc (float const t)
{
    if (abs(t)<0.002)
        return (1 - pow(t,2)/6*(1 - pow(t,2)/20));
    else
        return (sin(t)/t);
}

void circline (float const s, float const x0, float const y0, float const k, float const th0,
        Pose &ps)
{
  ps.x = x0 + s*sinc(k*s/2.0)*cos(th0+k*s/2);
  ps.y = y0 + s*sinc(k*s/2.0)*sin(th0+k*s/2);
  ps.theta = mod2pi(th0 + k*s);

}

void dubinsarc (float const x0, float const y0, float const th0, float const k, float const L,
       Path &pth)
{
    pth.points[0].x = x0;
    pth.points[0].y = y0;
    pth.points[0].theta = th0;
    pth.points[0].kappa = k;
    pth.points[0].s = L;
    circline (L, x0, y0, k, th0,
            pth.points[1]);
}

void dubinscurve (float const x0, float const y0, float const th0, float const s1, float const s2, float const s3, float const k0, float const k1, float const k2,
       Path &pth1, Path &pth2, Path &pth3, float L)
{
    dubinsarc (x0, y0, th0, k0, s1, pth1);
    dubinsarc (pth1.points[1].x, pth1.points[1].y, pth1.points[1].theta, k1, s2, pth2);
    dubinsarc (pth2.points[1].x, pth2.points[1].y, pth2.points[1].theta, k2, s3, pth3);
    L = pth1.points[0].s + pth2.points[0].s + pth3.points[0].s;
}

/* Calculate the angle between two points from an absolute point */
float calctheta (float const x1, float const y1, float const x2, float const y2)
{
    float ang = 0;
    /* if in the + y axis*/
    if (x1 == x2 && y1 < y2){
      ang = M_PI/2;
    }
    /* if in the - x axis*/
    else if (y1 == y2 && x1 > x2){
      ang = M_PI;
    }
    /* if in the - y axis*/
    else if (x1 == x2 && y1 > y2){
      ang = M_PI + (M_PI/2);
    }

    else {

      ang = atan2(fabs(y2-y1),fabs(x2-x1));

      /* if in the north-west quater*/
      if (x1 > x2 && y1 < y2){
        ang = M_PI - ang;
      }
      /* if in the south-west quater*/
      else if (x1 > x2 && y1 > y2){
        ang = M_PI + ang;
      }
      /* if in the south-east quater*/
      else if (x1 < x2 && y1 > y2){
        ang = (2*M_PI) - ang;
      }

    }
    return ang;

}

void dubins (float x0, float y0, float th0, float xf, float yf, float thf, int Kmax,
        DubinsPathType &path, Path &pth1, Path &pth2, Path &pth3, float &Ltotal)
{
    float sc_th0, sc_thf, sc_Kmax, lambda, sc_s1_c, sc_s2_c, sc_s3_c,
          Lcur, L = std::numeric_limits<float>::infinity(), sc_s1, sc_s2,
          sc_s3, s1, s2, s3;
    int ok = 0, pidx = -1;
    sc_th0 = sc_thf = sc_Kmax = lambda = 0.0;
#if DUBIN_DEBUG
    std::cout << x0 <<"," << y0 <<"," << th0 <<"," << xf <<"," << yf <<"," << thf <<"," << Kmax <<"," << std::endl;
#endif
    scaleToStandard (x0, y0, th0, xf, yf, thf, Kmax, sc_th0, sc_thf, sc_Kmax, lambda);
#if DUBIN_DEBUG
    std::cout << sc_th0 <<"," << sc_thf <<"," << sc_Kmax <<"," << lambda << std::endl;
#endif
    for (int i =0; i<6; i++)
    {

        ok = OptimalCurves[i] (sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
#if DUBIN_DEBUG
        std::cout << "i:[" << i << "] sc_s1_c:" << sc_s1_c << " ,sc_s2_c:" << sc_s2_c << " ,sc_s3_c:" << sc_s3_c << std::endl;
#endif
        Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
        if (ok && Lcur<L)
        {
            L = Lcur;
            sc_s1 = sc_s1_c;
            sc_s2 = sc_s2_c;
            sc_s3 = sc_s3_c;
            pidx = i;
        }
    }
#if DUBIN_DEBUG
    std::cout << "L:" << L << " sc_s1:" << sc_s1 << " sc_s2:" << sc_s2 << " sc_s3:" << sc_s3 << " pidx:" << pidx << std::endl;
#endif
    ok = 0;
    if (pidx>-1)
    {
        ok = 1;
        path = (DubinsPathType)pidx;
        scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);
        dubinscurve(x0, y0,th0, s1, s2, s3, ksigns[pidx][0]*Kmax, ksigns[pidx] [1]*Kmax, ksigns[pidx][2]*Kmax, pth1, pth2, pth3, Ltotal);
        //assert(check(sc_s1, ksigns(pidx,1)*sc_Kmax, sc_s2, ksigns(pidx,2)*sc_Kmax, sc_s3, ksigns(pidx,3)*sc_Kmax, sc_th0, sc_thf));

    }
}

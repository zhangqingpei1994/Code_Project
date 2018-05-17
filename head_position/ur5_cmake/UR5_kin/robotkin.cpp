#include "ur5_kin.h"
#include <math.h>
#include <stdio.h>
#include<stdlib.h>
#include <fstream>
using namespace std;
const double PI = 3.1415926;
const double d1 = 0.089159;
const double a2 = -0.42500;
const double a3 = -0.39225;
const double d4 = 0.10915;
const double d5 = 0.09465;
const double d6 = 0.0823;
void forward(const double* q, double* T)
{
        double s1 = sin(*q), c1 = cos(*q); q++;
        double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
        double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
        q234 += *q; q++;
        double s5 = sin(*q), c5 = cos(*q); q++;
        double s6 = sin(*q), c6 = cos(*q);
        double s234 = sin(q234), c234 = cos(q234);
        *T = ((c1*c234 - s1*s234)*s5) / 2.0 - c5*s1 + ((c1*c234 + s1*s234)*s5) / 2.0; T++;
        *T = (c6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0) -
            (s6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0); T++;
        *T = (-(c6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0 -
            s6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0)); T++;
        *T = ((d5*(s1*c234 - c1*s234)) / 2.0 - (d5*(s1*c234 + c1*s234)) / 2.0 -
            d4*s1 + (d6*(c1*c234 - s1*s234)*s5) / 2.0 + (d6*(c1*c234 + s1*s234)*s5) / 2.0 -
            a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3); T++;
        *T = c1*c5 + ((s1*c234 + c1*s234)*s5) / 2.0 + ((s1*c234 - c1*s234)*s5) / 2.0; T++;
        *T = (c6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0) +
            s6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0)); T++;
        *T = (c6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0) -
            s6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0)); T++;
        *T = ((d5*(c1*c234 - s1*s234)) / 2.0 - (d5*(c1*c234 + s1*s234)) / 2.0 + d4*c1 +
            (d6*(s1*c234 + c1*s234)*s5) / 2.0 + (d6*(s1*c234 - c1*s234)*s5) / 2.0 + d6*c1*c5 -
            a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3); T++;
        *T = ((c234*c5 - s234*s5) / 2.0 - (c234*c5 + s234*s5) / 2.0); T++;
        *T = ((s234*c6 - c234*s6) / 2.0 - (s234*c6 + c234*s6) / 2.0 - s234*c5*c6); T++;
        *T = (s234*c5*s6 - (c234*c6 + s234*s6) / 2.0 - (c234*c6 - s234*s6) / 2.0); T++;
        *T = (d1 + (d6*(c234*c5 - s234*s5)) / 2.0 + a3*(s2*c3 + c2*s3) + a2*s2 -
            (d6*(c234*c5 + s234*s5)) / 2.0 - d5*c234); T++;
        *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
    }

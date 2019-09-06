/*
 * Calculates the optimum drone trajectory
 *
 * @author Joseph Story <jdrs3@cam.ac.uk>
 * @author Vasileios Tsoutsouras
 */

#include <string.h>

/*** SETUP FOR COST CALCULATOR CODE ***/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> 
#include <math.h>

#include "FreeRTOS.h"
#include "console.h"
#include "assert.h"
#include "debug.h"
#include "config.h"
#include "log.h"
#include "param.h"
#include "crtp.h"
#include "task.h"

#include "trajectory.h"

#define MIN_FLIGHT_VELOCITY 5.0f
#define MAX_FLIGHT_VELOCITY 5.0f
#define INC_FLIGHT_VELOCITY 0.5f
#define FLIGHT_VELOCITY_STEPS 15
#define NUM_OF_ITEMS 6

const float earth_radius = 6371000; //metres;
const float pi = 3.1415926f;
const float g = 9.81f; //ms^-2
const float rho = 1.225f; //Density of air in kgm^-3

static float input_light_uniform_A1[NUM_OF_WAYPOINTS][7] = {
	{47.400531f, 8.545726f, 20.0f, 5.0f, 278.3f, 0.1746f, 0.1746f},
	{47.398529f, 8.548123f, 20.0f, 5.0f, 518.1f, 0.135f, 0.135f},
	{47.401395f, 8.548786f, 20.0f, 5.0f, 425.7f, 0.1389f, 0.1389f},
	{47.400935f, 8.548626f, 20.0f, 5.0f, 473.9f, 0.158f, 0.158f},
	{47.399671f, 8.54898f, 20.0f, 5.0f, 280.2f, 0.1598f, 0.1598f},
	{47.399744f, 8.547043f, 20.0f, 5.0f, 460.9f, 0.1373f, 0.1373f},
	{47.397979f, 8.546674f, 20.0f, 5.0f, 457.3f, 0.1337f, 0.1337f},
	{47.399042f, 8.545607f, 20.0f, 5.0f, 393.2f, 0.1216f, 0.1216f},
	{47.400494f, 8.546446f, 20.0f, 5.0f, 283.8f, 0.1806f, 0.1806f},
	{47.400284f, 8.548646f, 20.0f, 5.0f, 510.6f, 0.1157f, 0.1157f},
	{47.400666f, 8.547076f, 20.0f, 5.0f, 480.2f, 0.1856f, 0.1856f},
	{47.401413f, 8.548361f, 20.0f, 5.0f, 408.8f, 0.1197f, 0.1197f},
	{47.401867f, 8.548212f, 20.0f, 5.0f, 508.4f, 0.1605f, 0.1605f},
	{47.400242f, 8.549139f, 20.0f, 5.0f, 335.8f, 0.1135f, 0.1135f},
	{47.401417f, 8.549601f, 20.0f, 5.0f, 379.5f, 0.1064f, 0.1064f}
};

static float input_light_uniform_A2[NUM_OF_WAYPOINTS][7] = {
	{47.400311f, 8.547877f, 20.0f, 5.0f, 447.3f, 0.1479f, 0.1479f},
	{47.400145f, 8.548857f, 20.0f, 5.0f, 447.7f, 0.1347f, 0.1347f},
	{47.39814f, 8.549261f, 20.0f, 5.0f, 423.6f, 0.1516f, 0.1516f},
	{47.401077f, 8.549405f, 20.0f, 5.0f, 309.4f, 0.176f, 0.176f},
	{47.40046f, 8.547032f, 20.0f, 5.0f, 393.0f, 0.1365f, 0.1365f},
	{47.400108f, 8.549792f, 20.0f, 5.0f, 353.8f, 0.1407f, 0.1407f},
	{47.401154f, 8.549313f, 20.0f, 5.0f, 482.8f, 0.1121f, 0.1121f},
	{47.398187f, 8.548391f, 20.0f, 5.0f, 403.5f, 0.1183f, 0.1183f},
	{47.400587f, 8.54828f, 20.0f, 5.0f, 506.6f, 0.1898f, 0.1898f},
	{47.399886f, 8.547931f, 20.0f, 5.0f, 430.4f, 0.1726f, 0.1726f},
	{47.399915f, 8.547646f, 20.0f, 5.0f, 324.2f, 0.1665f, 0.1665f},
	{47.398016f, 8.549082f, 20.0f, 5.0f, 348.2f, 0.1029f, 0.1029f},
	{47.39863f, 8.546945f, 20.0f, 5.0f, 381.2f, 0.1579f, 0.1579f},
	{47.401482f, 8.546467f, 20.0f, 5.0f, 509.7f, 0.1071f, 0.1071f},
	{47.401199f, 8.545607f, 20.0f, 5.0f, 286.0f, 0.1732f, 0.1732f}
};

static float input_light_uniform_A3[NUM_OF_WAYPOINTS][7] = {
	{47.398692f, 8.545934f, 20.0f, 5.0f, 477.0f, 0.1562f, 0.1562f},
	{47.398189f, 8.546263f, 20.0f, 5.0f, 330.9f, 0.1784f, 0.1784f},
	{47.401752f, 8.548625f, 20.0f, 5.0f, 449.7f, 0.1068f, 0.1068f},
	{47.400179f, 8.54987f, 20.0f, 5.0f, 375.5f, 0.1775f, 0.1775f},
	{47.398777f, 8.548875f, 20.0f, 5.0f, 434.3f, 0.1997f, 0.1997f},
	{47.400265f, 8.548621f, 20.0f, 5.0f, 383.9f, 0.1247f, 0.1247f},
	{47.400455f, 8.547714f, 20.0f, 5.0f, 270.0f, 0.1349f, 0.1349f},
	{47.399752f, 8.547179f, 20.0f, 5.0f, 270.0f, 0.1589f, 0.1589f},
	{47.398137f, 8.547708f, 20.0f, 5.0f, 362.1f, 0.1257f, 0.1257f},
	{47.39938f, 8.548166f, 20.0f, 5.0f, 308.1f, 0.1415f, 0.1415f},
	{47.40132f, 8.547872f, 20.0f, 5.0f, 453.0f, 0.1343f, 0.1343f},
	{47.401867f, 8.546956f, 20.0f, 5.0f, 306.5f, 0.1991f, 0.1991f},
	{47.401587f, 8.547204f, 20.0f, 5.0f, 336.2f, 0.1048f, 0.1048f},
	{47.399461f, 8.545967f, 20.0f, 5.0f, 470.8f, 0.1523f, 0.1523f},
	{47.40027f, 8.545779f, 20.0f, 5.0f, 510.6f, 0.1359f, 0.1359f}
};

static float input_mid_uniform_A1[NUM_OF_WAYPOINTS][7] = {
	{47.401908f, 8.548893f, 20.0f, 5.0f, 292.8f, 0.2842f, 0.2842f},
	{47.398082f, 8.54851f, 20.0f, 5.0f, 417.8f, 0.2966f, 0.2966f},
	{47.401836f, 8.548872f, 20.0f, 5.0f, 342.8f, 0.269f, 0.269f},
	{47.398373f, 8.549745f, 20.0f, 5.0f, 387.6f, 0.2023f, 0.2023f},
	{47.398656f, 8.547431f, 20.0f, 5.0f, 449.7f, 0.3972f, 0.3972f},
	{47.398024f, 8.547757f, 20.0f, 5.0f, 505.5f, 0.3832f, 0.3832f},
	{47.401783f, 8.548393f, 20.0f, 5.0f, 499.1f, 0.152f, 0.152f},
	{47.402027f, 8.549529f, 20.0f, 5.0f, 411.0f, 0.2258f, 0.2258f},
	{47.39898f, 8.548501f, 20.0f, 5.0f, 440.7f, 0.2737f, 0.2737f},
	{47.39963f, 8.547295f, 20.0f, 5.0f, 281.1f, 0.1779f, 0.1779f},
	{47.398712f, 8.549651f, 20.0f, 5.0f, 509.7f, 0.3553f, 0.3553f},
	{47.398348f, 8.547932f, 20.0f, 5.0f, 420.7f, 0.2433f, 0.2433f},
	{47.401499f, 8.546505f, 20.0f, 5.0f, 309.4f, 0.2535f, 0.2535f},
	{47.402039f, 8.546774f, 20.0f, 5.0f, 341.0f, 0.3555f, 0.3555f},
	{47.400009f, 8.547382f, 20.0f, 5.0f, 442.3f, 0.3485f, 0.3485f}
};

static float input_mid_uniform_A2[NUM_OF_WAYPOINTS][7] = {
	{47.400611f, 8.549583f, 20.0f, 5.0f, 336.8f, 0.1798f, 0.1798f},
	{47.398877f, 8.547868f, 20.0f, 5.0f, 529.5f, 0.2761f, 0.2761f},
	{47.399676f, 8.548825f, 20.0f, 5.0f, 349.9f, 0.3617f, 0.3617f},
	{47.401896f, 8.546196f, 20.0f, 5.0f, 487.6f, 0.217f, 0.217f},
	{47.400587f, 8.548069f, 20.0f, 5.0f, 363.0f, 0.2165f, 0.2165f},
	{47.398708f, 8.54785f, 20.0f, 5.0f, 378.0f, 0.1962f, 0.1962f},
	{47.397784f, 8.547396f, 20.0f, 5.0f, 385.6f, 0.2647f, 0.2647f},
	{47.399063f, 8.545612f, 20.0f, 5.0f, 369.6f, 0.3685f, 0.3685f},
	{47.401254f, 8.54609f, 20.0f, 5.0f, 470.2f, 0.3907f, 0.3907f},
	{47.400248f, 8.547023f, 20.0f, 5.0f, 386.6f, 0.1064f, 0.1064f},
	{47.401942f, 8.549102f, 20.0f, 5.0f, 394.3f, 0.2516f, 0.2516f},
	{47.401869f, 8.549053f, 20.0f, 5.0f, 472.6f, 0.3528f, 0.3528f},
	{47.399352f, 8.549653f, 20.0f, 5.0f, 417.2f, 0.3413f, 0.3413f},
	{47.399591f, 8.546109f, 20.0f, 5.0f, 433.3f, 0.2043f, 0.2043f},
	{47.398588f, 8.547416f, 20.0f, 5.0f, 345.4f, 0.1167f, 0.1167f}
};

static float input_mid_uniform_A3[NUM_OF_WAYPOINTS][7] = {
	{47.399265f, 8.547727f, 20.0f, 5.0f, 370.9f, 0.3703f, 0.3703f},
	{47.398988f, 8.547734f, 20.0f, 5.0f, 291.4f, 0.1452f, 0.1452f},
	{47.399852f, 8.549769f, 20.0f, 5.0f, 341.4f, 0.3396f, 0.3396f},
	{47.400257f, 8.549799f, 20.0f, 5.0f, 374.4f, 0.2288f, 0.2288f},
	{47.401067f, 8.548227f, 20.0f, 5.0f, 376.2f, 0.2999f, 0.2999f},
	{47.401609f, 8.549754f, 20.0f, 5.0f, 332.4f, 0.3379f, 0.3379f},
	{47.39916f, 8.547769f, 20.0f, 5.0f, 368.8f, 0.3237f, 0.3237f},
	{47.399994f, 8.547208f, 20.0f, 5.0f, 461.7f, 0.2865f, 0.2865f},
	{47.399339f, 8.546526f, 20.0f, 5.0f, 321.5f, 0.3357f, 0.3357f},
	{47.39866f, 8.547615f, 20.0f, 5.0f, 278.4f, 0.1225f, 0.1225f},
	{47.398118f, 8.549748f, 20.0f, 5.0f, 375.7f, 0.3995f, 0.3995f},
	{47.401012f, 8.547312f, 20.0f, 5.0f, 524.9f, 0.2249f, 0.2249f},
	{47.401445f, 8.545864f, 20.0f, 5.0f, 400.7f, 0.2278f, 0.2278f},
	{47.401688f, 8.548747f, 20.0f, 5.0f, 337.5f, 0.2089f, 0.2089f},
	{47.399911f, 8.547065f, 20.0f, 5.0f, 366.4f, 0.3621f, 0.3621f}
};

static float input_heavy_uniform_A1[NUM_OF_WAYPOINTS][7] = {
	{47.398273f, 8.546864f, 20.0f, 5.0f, 448.5f, 0.3708f, 0.3708f},
	{47.397837f, 8.548501f, 20.0f, 5.0f, 327.7f, 0.2422f, 0.2422f},
	{47.401718f, 8.549628f, 20.0f, 5.0f, 383.9f, 0.1969f, 0.1969f},
	{47.401017f, 8.547346f, 20.0f, 5.0f, 447.3f, 0.7385f, 0.7385f},
	{47.40054f, 8.548873f, 20.0f, 5.0f, 505.6f, 0.1517f, 0.1517f},
	{47.399358f, 8.548942f, 20.0f, 5.0f, 498.9f, 0.3547f, 0.3547f},
	{47.401884f, 8.548335f, 20.0f, 5.0f, 367.5f, 0.3931f, 0.3931f},
	{47.399582f, 8.547915f, 20.0f, 5.0f, 461.5f, 0.6422f, 0.6422f},
	{47.399659f, 8.549806f, 20.0f, 5.0f, 337.9f, 0.6084f, 0.6084f},
	{47.398231f, 8.546547f, 20.0f, 5.0f, 362.6f, 0.1105f, 0.1105f},
	{47.401436f, 8.546741f, 20.0f, 5.0f, 360.6f, 0.586f, 0.586f},
	{47.398383f, 8.548167f, 20.0f, 5.0f, 478.5f, 0.7768f, 0.7768f},
	{47.398619f, 8.546152f, 20.0f, 5.0f, 378.0f, 0.1102f, 0.1102f},
	{47.398864f, 8.547519f, 20.0f, 5.0f, 375.6f, 0.4526f, 0.4526f},
	{47.400386f, 8.547886f, 20.0f, 5.0f, 425.6f, 0.2243f, 0.2243f}
};

static float input_heavy_uniform_A2[NUM_OF_WAYPOINTS][7] = {
	{47.399915f, 8.547102f, 20.0f, 5.0f, 395.8f, 0.3166f, 0.3166f},
	{47.398249f, 8.548196f, 20.0f, 5.0f, 313.9f, 0.1153f, 0.1153f},
	{47.398167f, 8.547819f, 20.0f, 5.0f, 461.2f, 0.3928f, 0.3928f},
	{47.3981f, 8.547587f, 20.0f, 5.0f, 519.9f, 0.5972f, 0.5972f},
	{47.398099f, 8.547685f, 20.0f, 5.0f, 377.1f, 0.6217f, 0.6217f},
	{47.40007f, 8.549234f, 20.0f, 5.0f, 423.2f, 0.1373f, 0.1373f},
	{47.401643f, 8.546713f, 20.0f, 5.0f, 387.7f, 0.5547f, 0.5547f},
	{47.401339f, 8.549508f, 20.0f, 5.0f, 314.4f, 0.1827f, 0.1827f},
	{47.401136f, 8.545703f, 20.0f, 5.0f, 306.1f, 0.3658f, 0.3658f},
	{47.399924f, 8.547904f, 20.0f, 5.0f, 356.3f, 0.5222f, 0.5222f},
	{47.40045f, 8.548772f, 20.0f, 5.0f, 353.4f, 0.435f, 0.435f},
	{47.397842f, 8.549019f, 20.0f, 5.0f, 402.8f, 0.5795f, 0.5795f},
	{47.401997f, 8.549075f, 20.0f, 5.0f, 345.1f, 0.4968f, 0.4968f},
	{47.400253f, 8.545827f, 20.0f, 5.0f, 437.3f, 0.2191f, 0.2191f},
	{47.399182f, 8.54653f, 20.0f, 5.0f, 478.5f, 0.2239f, 0.2239f}
};

static float input_heavy_uniform_A3[NUM_OF_WAYPOINTS][7] = {
	{47.401923f, 8.549843f, 20.0f, 5.0f, 438.6f, 0.1298f, 0.1298f},
	{47.400701f, 8.546357f, 20.0f, 5.0f, 297.8f, 0.727f, 0.727f},
	{47.399683f, 8.547873f, 20.0f, 5.0f, 287.1f, 0.5901f, 0.5901f},
	{47.398463f, 8.547258f, 20.0f, 5.0f, 481.0f, 0.4473f, 0.4473f},
	{47.398035f, 8.547075f, 20.0f, 5.0f, 415.8f, 0.4409f, 0.4409f},
	{47.399799f, 8.549891f, 20.0f, 5.0f, 329.0f, 0.4863f, 0.4863f},
	{47.401843f, 8.546182f, 20.0f, 5.0f, 406.8f, 0.4186f, 0.4186f},
	{47.400357f, 8.547087f, 20.0f, 5.0f, 397.0f, 0.2378f, 0.2378f},
	{47.399889f, 8.547714f, 20.0f, 5.0f, 506.1f, 0.3437f, 0.3437f},
	{47.398657f, 8.54565f, 20.0f, 5.0f, 479.2f, 0.1838f, 0.1838f},
	{47.401623f, 8.546507f, 20.0f, 5.0f, 304.9f, 0.7417f, 0.7417f},
	{47.3997f, 8.546655f, 20.0f, 5.0f, 488.3f, 0.5126f, 0.5126f},
	{47.400687f, 8.549807f, 20.0f, 5.0f, 472.8f, 0.7459f, 0.7459f},
	{47.399035f, 8.546572f, 20.0f, 5.0f, 526.0f, 0.4478f, 0.4478f},
	{47.398715f, 8.546153f, 20.0f, 5.0f, 344.7f, 0.3141f, 0.3141f}
};

/* 
static float input_light_uniform_B1[NUM_OF_WAYPOINTS][7] = {
	{47.40167108, 8.54720284, 20.0, 5, 452.3, 0.1701, 0.1701},
	{47.40033428, 8.54520417, 20.0, 5, 784.5, 0.1824, 0.1824},
	{47.398144, 8.5497763, 20.0, 5, 652.3, 0.1146, 0.1146},
	{47.39479804, 8.54856018, 20.0, 5, 581.5, 0.1822, 0.1822},
	{47.39774153, 8.54635528, 20.0, 5, 725.8, 0.156, 0.156},
	{47.40139503, 8.54236207, 20.0, 5, 561.1, 0.1406, 0.1406},
	{47.39614984, 8.54612786, 20.0, 5, 486.2, 0.1672, 0.1672},
	{47.4013789, 8.54266531, 20.0, 5, 421.6, 0.1377, 0.1377},
	{47.39962529, 8.54436337, 20.0, 5, 753.6, 0.1357, 0.1357},
	{47.39367267, 8.54356295, 20.0, 5, 749.9, 0.1272, 0.1272},
	{47.39803545, 8.54895856, 20.0, 5, 504.9, 0.1424, 0.1424},
	{47.4012942, 8.54770495, 20.0, 5, 505.7, 0.1356, 0.1356},
	{47.39483198, 8.54894683, 20.0, 5, 761.5, 0.1524, 0.1524},
	{47.396549, 8.54466141, 20.0, 5, 498.7, 0.1009, 0.1009},
	{47.39458133, 8.54870597, 20.0, 5, 795.0, 0.1567, 0.1567}
};

static float input_light_uniform_B2[NUM_OF_WAYPOINTS][7] = {
	{47.39463171, 8.54821341, 20.0, 5, 742.4, 0.1515, 0.1515},
	{47.39667672, 8.54886743, 20.0, 5, 764.7, 0.1781, 0.1781},
	{47.40012418, 8.54500551, 20.0, 5, 610.1, 0.1962, 0.1962},
	{47.39773829, 8.54481086, 20.0, 5, 490.3, 0.1784, 0.1784},
	{47.40082509, 8.54874238, 20.0, 5, 474.5, 0.1763, 0.1763},
	{47.39682558, 8.54534939, 20.0, 5, 789.3, 0.1063, 0.1063},
	{47.39625256, 8.54772443, 20.0, 5, 440.2, 0.1638, 0.1638},
	{47.39529044, 8.54849244, 20.0, 5, 424.2, 0.1701, 0.1701},
	{47.39690615, 8.5492443, 20.0, 5, 770.9, 0.1789, 0.1789},
	{47.39805608, 8.54809588, 20.0, 5, 483.6, 0.1811, 0.1811},
	{47.40008853, 8.54364423, 20.0, 5, 743.9, 0.1389, 0.1389},
	{47.3950886, 8.54742304, 20.0, 5, 438.2, 0.1505, 0.1505},
	{47.39874595, 8.54255651, 20.0, 5, 632.7, 0.132, 0.132},
	{47.39383839, 8.54179483, 20.0, 5, 745.7, 0.1411, 0.1411},
	{47.399739, 8.54930723, 20.0, 5, 529.8, 0.1964, 0.1964}
};

static float input_light_uniform_B3[NUM_OF_WAYPOINTS][7] = {
	{47.40104873, 8.54319926, 20.0, 5, 519.6, 0.1573, 0.1573},
	{47.40162845, 8.54363155, 20.0, 5, 783.6, 0.1751, 0.1751},
	{47.39416417, 8.54246039, 20.0, 5, 779.6, 0.1692, 0.1692},
	{47.39364311, 8.54475308, 20.0, 5, 798.9, 0.1486, 0.1486},
	{47.40087127, 8.54984184, 20.0, 5, 516.3, 0.1813, 0.1813},
	{47.39472329, 8.5492695, 20.0, 5, 703.5, 0.1084, 0.1084},
	{47.40118363, 8.54593261, 20.0, 5, 734.5, 0.1965, 0.1965},
	{47.39568651, 8.54666024, 20.0, 5, 445.8, 0.1149, 0.1149},
	{47.39821228, 8.54902323, 20.0, 5, 632.7, 0.1836, 0.1836},
	{47.39887574, 8.54364103, 20.0, 5, 790.1, 0.1475, 0.1475},
	{47.4008041, 8.54260586, 20.0, 5, 569.1, 0.1234, 0.1234},
	{47.39843211, 8.54269147, 20.0, 5, 616.6, 0.1976, 0.1976},
	{47.39429514, 8.54712373, 20.0, 5, 718.8, 0.1046, 0.1046},
	{47.39939546, 8.54934892, 20.0, 5, 449.1, 0.1674, 0.1674},
	{47.39668022, 8.54645575, 20.0, 5, 775.1, 0.1415, 0.1415}
};

static float input_mid_uniform_B1[NUM_OF_WAYPOINTS][7] = {
	{47.39931332, 8.54498389, 20.0, 5, 651.7, 0.2112, 0.2112},
	{47.40156277, 8.54556594, 20.0, 5, 433.9, 0.1945, 0.1945},
	{47.39770382, 8.54173689, 20.0, 5, 522.1, 0.139, 0.139},
	{47.39859545, 8.54392461, 20.0, 5, 656.9, 0.2282, 0.2282},
	{47.39532492, 8.54303607, 20.0, 5, 667.8, 0.1662, 0.1662},
	{47.39568738, 8.5442268, 20.0, 5, 612.1, 0.3774, 0.3774},
	{47.39509365, 8.54481391, 20.0, 5, 504.1, 0.1806, 0.1806},
	{47.39577984, 8.54258924, 20.0, 5, 779.1, 0.292, 0.292},
	{47.39508494, 8.54834627, 20.0, 5, 649.3, 0.263, 0.263},
	{47.39749482, 8.54658353, 20.0, 5, 539.5, 0.2401, 0.2401},
	{47.40085868, 8.54717604, 20.0, 5, 800.4, 0.2507, 0.2507},
	{47.39788872, 8.54901278, 20.0, 5, 408.5, 0.2885, 0.2885},
	{47.3999521, 8.54980799, 20.0, 5, 417.2, 0.3823, 0.3823},
	{47.3957284, 8.54862536, 20.0, 5, 464.0, 0.2085, 0.2085},
	{47.39713713, 8.54647483, 20.0, 5, 636.7, 0.3628, 0.3628}
};

static float input_mid_uniform_B2[NUM_OF_WAYPOINTS][7] = {
	{47.39755014, 8.54401945, 20.0, 5, 660.8, 0.3364, 0.3364},
	{47.40187918, 8.54540488, 20.0, 5, 776.6, 0.1134, 0.1134},
	{47.39795436, 8.5414289, 20.0, 5, 503.5, 0.1441, 0.1441},
	{47.40110862, 8.54900705, 20.0, 5, 423.4, 0.2743, 0.2743},
	{47.3943794, 8.54453957, 20.0, 5, 775.1, 0.3381, 0.3381},
	{47.40120934, 8.54508736, 20.0, 5, 514.4, 0.3405, 0.3405},
	{47.40013105, 8.54517979, 20.0, 5, 500.6, 0.3222, 0.3222},
	{47.40119192, 8.54548721, 20.0, 5, 592.4, 0.3571, 0.3571},
	{47.39545336, 8.54891502, 20.0, 5, 626.8, 0.1551, 0.1551},
	{47.39853382, 8.54585795, 20.0, 5, 783.2, 0.2618, 0.2618},
	{47.39829136, 8.54243289, 20.0, 5, 506.6, 0.3457, 0.3457},
	{47.39425366, 8.54790718, 20.0, 5, 414.2, 0.2817, 0.2817},
	{47.39886929, 8.54163337, 20.0, 5, 523.2, 0.1362, 0.1362},
	{47.39913898, 8.54879425, 20.0, 5, 762.9, 0.372, 0.372},
	{47.39486826, 8.54857783, 20.0, 5, 492.7, 0.3504, 0.3504}
};

static float input_mid_uniform_B3[NUM_OF_WAYPOINTS][7] = {
	{47.39396152, 8.54256187, 20.0, 5, 614.0, 0.1298, 0.1298},
	{47.39981367, 8.54474006, 20.0, 5, 766.4, 0.3417, 0.3417},
	{47.40117968, 8.54567098, 20.0, 5, 546.0, 0.3518, 0.3518},
	{47.3938034, 8.54133847, 20.0, 5, 430.4, 0.2076, 0.2076},
	{47.39908922, 8.54919818, 20.0, 5, 578.5, 0.3103, 0.3103},
	{47.39505132, 8.5473655, 20.0, 5, 478.1, 0.1534, 0.1534},
	{47.39979833, 8.54897659, 20.0, 5, 598.6, 0.1253, 0.1253},
	{47.39601587, 8.5478263, 20.0, 5, 531.8, 0.1128, 0.1128},
	{47.39641475, 8.54650241, 20.0, 5, 696.2, 0.2022, 0.2022},
	{47.40161292, 8.54132424, 20.0, 5, 577.6, 0.1536, 0.1536},
	{47.39907098, 8.54648001, 20.0, 5, 575.6, 0.3918, 0.3918},
	{47.39715974, 8.54506433, 20.0, 5, 650.1, 0.1017, 0.1017},
	{47.40046078, 8.54153069, 20.0, 5, 432.6, 0.3653, 0.3653},
	{47.39832849, 8.54592907, 20.0, 5, 535.0, 0.2407, 0.2407},
	{47.39387762, 8.54329799, 20.0, 5, 497.9, 0.3126, 0.3126}
};

float input_heavy_uniform_B1[NUM_OF_WAYPOINTS][7] = {
	{47.39890653, 8.5492624, 20.0, 5, 429.9, 0.7909, 0.7909},
	{47.39945494, 8.54780591, 20.0, 5, 690.9, 0.568, 0.568},
	{47.39942159, 8.54650857, 20.0, 5, 804.2, 0.7953, 0.7953},
	{47.39463473, 8.54797002, 20.0, 5, 548.7, 0.4643, 0.4643},
	{47.39706338, 8.54447815, 20.0, 5, 570.4, 0.1589, 0.1589},
	{47.39793367, 8.54925546, 20.0, 5, 620.4, 0.1713, 0.1713},
	{47.39962592, 8.54902573, 20.0, 5, 531.5, 0.491, 0.491},
	{47.40128857, 8.54524555, 20.0, 5, 732.8, 0.3232, 0.3232},
	{47.39806181, 8.54160948, 20.0, 5, 440.9, 0.7259, 0.7259},
	{47.39808151, 8.54590253, 20.0, 5, 528.5, 0.418, 0.418},
	{47.39527705, 8.54351155, 20.0, 5, 737.0, 0.4534, 0.4534},
	{47.39661326, 8.54195309, 20.0, 5, 552.4, 0.6856, 0.6856},
	{47.40160827, 8.54184833, 20.0, 5, 521.7, 0.6592, 0.6592},
	{47.39374796, 8.54662224, 20.0, 5, 494.1, 0.2306, 0.2306},
	{47.39412687, 8.54758751, 20.0, 5, 560.6, 0.6807, 0.6807}
};

static float input_heavy_uniform_B2[NUM_OF_WAYPOINTS][7] = {
	{47.39932748, 8.54884893, 20.0, 5, 770.4, 0.5504, 0.5504},
	{47.3952487, 8.54363234, 20.0, 5, 460.4, 0.6671, 0.6671},
	{47.39532418, 8.54982967, 20.0, 5, 658.1, 0.2953, 0.2953},
	{47.39486579, 8.54849099, 20.0, 5, 642.4, 0.4621, 0.4621},
	{47.40048125, 8.54558893, 20.0, 5, 444.1, 0.2259, 0.2259},
	{47.39648885, 8.54926356, 20.0, 5, 755.6, 0.3212, 0.3212},
	{47.39592581, 8.54222662, 20.0, 5, 716.8, 0.4369, 0.4369},
	{47.39520518, 8.5486458, 20.0, 5, 504.5, 0.4178, 0.4178},
	{47.39777428, 8.5480329, 20.0, 5, 744.1, 0.4732, 0.4732},
	{47.3992311, 8.54718832, 20.0, 5, 442.5, 0.1244, 0.1244},
	{47.39618123, 8.54943713, 20.0, 5, 562.4, 0.407, 0.407},
	{47.39992708, 8.54668479, 20.0, 5, 428.1, 0.1232, 0.1232},
	{47.39680592, 8.54320949, 20.0, 5, 518.8, 0.2334, 0.2334},
	{47.39559616, 8.54469396, 20.0, 5, 478.5, 0.1486, 0.1486},
	{47.39379423, 8.54306419, 20.0, 5, 427.8, 0.746, 0.746}
};

static float input_heavy_uniform_B3[NUM_OF_WAYPOINTS][7] = {
	{47.39369254, 8.54607044, 20.0, 5, 519.5, 0.2153, 0.2153},
	{47.3953938, 8.54649939, 20.0, 5, 705.7, 0.7922, 0.7922},
	{47.39444388, 8.54424386, 20.0, 5, 601.4, 0.781, 0.781},
	{47.40152253, 8.54618472, 20.0, 5, 615.1, 0.1022, 0.1022},
	{47.39661807, 8.54965409, 20.0, 5, 578.9, 0.5769, 0.5769},
	{47.3992776, 8.54344972, 20.0, 5, 605.2, 0.2709, 0.2709},
	{47.40019179, 8.54129576, 20.0, 5, 480.5, 0.3243, 0.3243},
	{47.39500915, 8.54630477, 20.0, 5, 512.6, 0.1754, 0.1754},
	{47.40123066, 8.54831279, 20.0, 5, 557.6, 0.4277, 0.4277},
	{47.39626845, 8.54382294, 20.0, 5, 484.1, 0.6875, 0.6875},
	{47.39832668, 8.54549874, 20.0, 5, 782.3, 0.3116, 0.3116},
	{47.39878621, 8.54873671, 20.0, 5, 481.8, 0.3567, 0.3567},
	{47.39986115, 8.5425563, 20.0, 5, 523.0, 0.1498, 0.1498},
	{47.40063245, 8.54495185, 20.0, 5, 797.7, 0.4435, 0.4435},
	{47.39926369, 8.54563248, 20.0, 5, 678.4, 0.6602, 0.6602}
};
*/

//Private functions
static void trajectoryTask(void * prm);

//Finds a close to optimal route using the 'Simulated Annealing' algorithm
static trajectory_cost_t solution_sa(mission_waypoint_t *uploadedWpsList, int num_waypoints, int *solutionTrajMatrix, float *solSpeedMatrix, bool startRandom);

//Finds a close to optimal route using the 'Greedy' method
static trajectory_cost_t my_solution_nn(mission_waypoint_t *uploadedWpsList, int num_waypoints, int *solutionTrajMatrix, float *solSpeedMatrix, float speed);

//Calculate the distance between two waypoints in spherical polar coordinates (latitude, longitude, and altitude)
float calc_flight_time(struct mission_item_s waypoint1, struct mission_item_s waypoint2, float flight_speed);

//Calculate an estimate of the battery percentage used between two waypoints
static float calc_energy_use(struct mission_item_s waypoint1, struct mission_item_s waypoint2, float flight_speed, float payload);

static trajectory_cost_t calculateTrajectoryCost(mission_waypoint_t *uploadedWpsList, int num_waypoints, int trajectoryMatrix[], float speedMatrix[]);

static uint32_t exec_scenario (float input_benchmark[15][7], int numItems);

static float delivery_time = 5.0; /* How close to reality is this? */
static float acceleration_time = 3.0;
//Physical drone properties
float mass = 1.38f; // kg
//Battery and electrical properties
float bat_capacity; // mAh
const float bat_voltage = 15.2f; // V
const float bat_energy = 81.3f; // Wh
const float bat_mass = 0.0f; // kg
const float efficiency = 100.0f; // %

static bool isInit = false;
static CRTPPacket p;

void trajectoryInit(void)
{
  if(isInit)
    return;

  /*
  if (owScan(&nbrOwMems))
    isInit = true;
  else
    isInit = false;
  */
  isInit = true;

  DEBUG_PRINT("Create trajectoryTestTask NumItems %d\n", NUM_OF_ITEMS);
  //Start the mem task
  xTaskCreate(trajectoryTask, TRAJ_CRTP_TASK_NAME,
              TRAJ_CRTP_TASK_STACKSIZE, NULL, TRAJ_CRTP_TASK_PRI, NULL);
}

bool trajectoryTest(void)
{
  return isInit;
}

void trajectoryTask(void * param)
{
	//crtpInitTaskQueue(CRTP_PORT_PWM);
    int numItems = NUM_OF_ITEMS;

    // int execScenario=0;
    // uint16_t executeAll=0;
    uint32_t elapsedTimeMs=0;
    static bool executedOnce = false;

	while(1)
	{
		if (executedOnce == false) {
            vTaskDelay(M2T(10000));
        
            DEBUG_PRINT("Executing input_light_uniform_A1 \n");
            elapsedTimeMs = exec_scenario (input_light_uniform_A1, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            DEBUG_PRINT("Executing input_light_uniform_A2 \n");    
            elapsedTimeMs = exec_scenario (input_light_uniform_A2, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            DEBUG_PRINT("Executing input_light_uniform_A3 \n");
            elapsedTimeMs = exec_scenario (input_light_uniform_A3, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            DEBUG_PRINT("Executing input_mid_uniform_A1 \n");
            elapsedTimeMs = exec_scenario (input_mid_uniform_A1, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            DEBUG_PRINT("Executing input_mid_uniform_A2 \n");
            elapsedTimeMs = exec_scenario (input_mid_uniform_A2, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            DEBUG_PRINT("Executing input_mid_uniform_A3 \n");
            elapsedTimeMs = exec_scenario (input_mid_uniform_A3, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            DEBUG_PRINT("Executing input_heavy_uniform_A1 \n");
            elapsedTimeMs = exec_scenario (input_heavy_uniform_A1, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            DEBUG_PRINT("Executing input_heavy_uniform_A2 \n");
            elapsedTimeMs = exec_scenario (input_heavy_uniform_A2, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            DEBUG_PRINT("Executing input_heavy_uniform_A3 \n");
            elapsedTimeMs = exec_scenario (input_heavy_uniform_A3, numItems);
            DEBUG_PRINT("Elapsed time was %lu ms\n",elapsedTimeMs);

            vTaskDelay(M2T(5*1000));

            executedOnce = true;
        } else {
            vTaskDelay(M2T(100000));
        }   
        
        
		
		
        
        // crtpReceivePacketBlock(CRTP_PORT_PWM, &p);

        // if (p.channel == 1) {

        // } else if (p.channel == 2) { /* Take advantage of crtp pwm with two values to check what to execute */
        //     if (executedOnce == false) {
        //         memcpy(&executeAll,&p.data[0],2); 
        //         memcpy(&execScenario,&p.data[2],4);

        //         #ifdef DEBUG
        //             DEBUG_PRINT("executeAll %d, execScenario %d\n",executeAll,execScenario);
        //         #endif 
                
        //         if (executeAll == 1) {
        //             DEBUG_PRINT("Executing input_light_uniform_A1 \n");
        //             elapsedTimeMs = exec_scenario (input_light_uniform_A1, numItems);
        //             DEBUG_PRINT("Executing input_light_uniform_A2 \n");    
        //             elapsedTimeMs = exec_scenario (input_light_uniform_A2, numItems);
        //             DEBUG_PRINT("Executing input_light_uniform_A3 \n");
        //             elapsedTimeMs = exec_scenario (input_light_uniform_A3, numItems);
        //             DEBUG_PRINT("Executing input_mid_uniform_A1 \n");
        //             elapsedTimeMs = exec_scenario (input_mid_uniform_A1, numItems);
        //             DEBUG_PRINT("Executing input_mid_uniform_A2 \n");
        //             elapsedTimeMs = exec_scenario (input_mid_uniform_A2, numItems);
        //             DEBUG_PRINT("Executing input_mid_uniform_A3 \n");
        //             elapsedTimeMs = exec_scenario (input_mid_uniform_A3, numItems);
        //             DEBUG_PRINT("Executing input_heavy_uniform_A1 \n");
        //             elapsedTimeMs = exec_scenario (input_heavy_uniform_A1, numItems);
        //             DEBUG_PRINT("Executing input_heavy_uniform_A2 \n");
        //             elapsedTimeMs = exec_scenario (input_heavy_uniform_A2, numItems);
        //             DEBUG_PRINT("Executing input_heavy_uniform_A3 \n");
        //             elapsedTimeMs = exec_scenario (input_heavy_uniform_A3, numItems);
        //         } else if (execScenario == 1) {
        //             DEBUG_PRINT("Executing input_light_uniform_A1 \n");
        //             elapsedTimeMs = exec_scenario (input_light_uniform_A1, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 2) {
        //             DEBUG_PRINT("Executing input_light_uniform_A2 \n");    
        //             elapsedTimeMs = exec_scenario (input_light_uniform_A2, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 3) {
        //             DEBUG_PRINT("Executing input_light_uniform_A3 \n");
        //             elapsedTimeMs = exec_scenario (input_light_uniform_A3, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 4) {
        //             DEBUG_PRINT("Executing input_mid_uniform_A1 \n");
        //             elapsedTimeMs = exec_scenario (input_mid_uniform_A1, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 5) {
        //             DEBUG_PRINT("Executing input_mid_uniform_A2 \n");
        //             elapsedTimeMs = exec_scenario (input_mid_uniform_A2, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 6) {
        //             DEBUG_PRINT("Executing input_mid_uniform_A3 \n");
        //             elapsedTimeMs = exec_scenario (input_mid_uniform_A3, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 7) {
        //             DEBUG_PRINT("Executing input_heavy_uniform_A1 \n");
        //             elapsedTimeMs = exec_scenario (input_heavy_uniform_A1, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 8) {
        //             DEBUG_PRINT("Executing input_heavy_uniform_A2 \n");
        //             elapsedTimeMs = exec_scenario (input_heavy_uniform_A2, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 9) {
        //             DEBUG_PRINT("Executing input_heavy_uniform_A3 \n");
        //             elapsedTimeMs = exec_scenario (input_heavy_uniform_A3, numItems);
        //             DEBUG_PRINT("Elaped time was %lu ms\n",elapsedTimeMs);
        //         } else if (execScenario == 10) {

        //         } else if (execScenario == 11) {

        //         }

        //         //vTaskDelay(M2T(profileSeconds*1000));
        //         if (!((executeAll == 0) && (execScenario == 0))) {
        //             executedOnce = true;
        //         }
                
        //     }
        // } else if (p.channel == 0) {
        // } else if (p.channel == 3) {

        // } else {
        //     DEBUG_PRINT("Unknown channel value %d\n",p.channel);    
        // }
	}
}

float calc_flight_time(struct mission_item_s waypoint1, struct mission_item_s waypoint2, float flight_speed)
{
    float lat1_rad = (waypoint1.lat/180)*pi;
    float lat2_rad = (waypoint2.lat/180)*pi;
    float lon1_rad = (waypoint1.lon/180)*pi;
    float lon2_rad = (waypoint2.lon/180)*pi;

    float alt1 = waypoint1.altitude;
    float alt2 = waypoint2.altitude;
    
    float x = sqrtf(powf(earth_radius+alt1, 2)+powf(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                    (earth_radius+alt2)*(sinf(lat1_rad)*sinf(lat2_rad)+cosf(lat1_rad)*cosf(lat2_rad)*cosf(lon1_rad-lon2_rad)));

    float time = (x/flight_speed) + delivery_time + acceleration_time; // Approximate time required to deliver an item

    return time;
}

//Calculate an estimate of the battery percentage used between two waypoints
float calc_energy_use(struct mission_item_s waypoint1, struct mission_item_s waypoint2, float flight_speed, float payload)
{
    float lat1_rad = (waypoint1.lat/180)*pi;
    float lat2_rad = (waypoint2.lat/180)*pi;
    float lon1_rad = (waypoint1.lon/180)*pi;
    float lon2_rad = (waypoint2.lon/180)*pi;

    float alt1 = waypoint1.altitude;
    float alt2 = waypoint2.altitude;

    float x = sqrtf(powf(earth_radius+alt1, 2)+powf(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                    (earth_radius+alt2)*(sinf(lat1_rad)*sinf(lat2_rad)+cosf(lat1_rad)*cosf(lat2_rad)*cosf(lon1_rad-lon2_rad)));

    float time = (x/flight_speed) + delivery_time + acceleration_time; // Approximate time required to deliver an item

    float power;
    //Object instantiation might be missing
    
    //power = sqrt( (pow(mass+bat_mass+payload,3)*pow(g,3)) / (2*rho*rotor_area) );
    power = sqrtf( (powf(mass+bat_mass+payload,3)*powf(g,3)) / (2*1.225f*0.568f) ); //Calculate the rotor area?

    float percent_used = (((power*time) / (bat_energy*3600)) * 100) / (efficiency/100);

    return percent_used;
}

trajectory_cost_t calculateTrajectoryCost(mission_waypoint_t *uploadedWpsList, int num_waypoints, int trajectoryMatrix[], float speedMatrix[]) {
    trajectory_cost_t curTrajCost = {.requiredEnergy = 0.0, .missedDeadlines = 0, .avgDelay = 0.0};
    int j;
    float sumOfMissedDelaysValues = 0;
    float arrival_at_j = 0;
    float trajectory_payload = uploadedWpsList[0].waypoint.payload_weight;

    //printf("Init curTrajCost.requiredEnergy is %f\n",curTrajCost.requiredEnergy);

    for (j=0; j<num_waypoints-1; j++) {
        curTrajCost.requiredEnergy += calc_energy_use(uploadedWpsList[trajectoryMatrix[j]].waypoint, uploadedWpsList[trajectoryMatrix[j+1]].waypoint, speedMatrix[j], trajectory_payload);
        //printf("j = %d curTrajCost.requiredEnergy is %f speedMatrix is %f\n",j,curTrajCost.requiredEnergy,speedMatrix[j]);
        /* Time to fly from j to j+1 */
        arrival_at_j += calc_flight_time(uploadedWpsList[trajectoryMatrix[j]].waypoint, uploadedWpsList[trajectoryMatrix[j+1]].waypoint, speedMatrix[j]);

        if (arrival_at_j > uploadedWpsList[trajectoryMatrix[j+1]].waypoint.deadline) {
            curTrajCost.missedDeadlines++;
            sumOfMissedDelaysValues += arrival_at_j - uploadedWpsList[trajectoryMatrix[j+1]].waypoint.deadline;
        }

        trajectory_payload -= uploadedWpsList[trajectoryMatrix[j+1]].waypoint.payload_weight;
    }

    /* Add energy cost of returning to takeoff -- there is not deadline cost yet*/
    curTrajCost.requiredEnergy += calc_energy_use(uploadedWpsList[trajectoryMatrix[j]].waypoint, uploadedWpsList[num_waypoints].waypoint, speedMatrix[j], trajectory_payload);
    
    /* TODO import in mainstream code  */
    if (curTrajCost.missedDeadlines == 0) {
        curTrajCost.avgDelay = 0.0;
    } else {
        curTrajCost.avgDelay = sumOfMissedDelaysValues / (float) curTrajCost.missedDeadlines;
    }

    return curTrajCost;
}

trajectory_cost_t my_solution_nn(mission_waypoint_t *uploadedWpsList, int num_waypoints, int *solutionTrajMatrix, float *solSpeedMatrix, float speed)
{
    int startWp=0, chosenWp=0, visitedNode[NUM_OF_ITEMS+1];
    trajectory_cost_t curSolutionTrajCost;
    float minCurDist, distTimeArray[NUM_OF_ITEMS+1][NUM_OF_ITEMS+1];

    //visitedNode = (int *) malloc(num_waypoints * sizeof(int));
    //distTimeArray = (double **) malloc(num_waypoints * sizeof(double *));

    for (int i=0; i<num_waypoints; i++) {
        visitedNode[i] = 0;
        solSpeedMatrix[i] = 5.0;

        //distTimeArray[i] = (double *) malloc(num_waypoints * sizeof(double));
        for (int t = 0; t < num_waypoints; t++){
            distTimeArray[i][t] = calc_flight_time(uploadedWpsList[i].waypoint, uploadedWpsList[t].waypoint, speed);
        }
    }    

    //Set the depot node as visited
    visitedNode[0]=1;
    solutionTrajMatrix[0] = 0;

    for (int curWpIndex=1; curWpIndex<num_waypoints; curWpIndex++) {
        minCurDist = INFINITY;
        
        for (int examiningWp=1; examiningWp<num_waypoints; examiningWp++) {
            if (visitedNode[examiningWp] == 0) {
                if (distTimeArray[startWp][examiningWp] < minCurDist) {
                    minCurDist = distTimeArray[startWp][examiningWp];
                    chosenWp = examiningWp;
                }    
            }
        }

        solutionTrajMatrix[curWpIndex] = chosenWp;
        visitedNode[chosenWp] = 1;
    }
    
    curSolutionTrajCost = calculateTrajectoryCost(uploadedWpsList, num_waypoints, solutionTrajMatrix, solSpeedMatrix);

    return curSolutionTrajCost;  
}

//Finds a close to optimal route using the 'Simulated Annealing' algorithm
trajectory_cost_t solution_sa(mission_waypoint_t *uploadedWpsList, int num_waypoints, int *solutionTrajMatrix, float *solSpeedMatrix, bool startRandom)
{
    int iterationsMax = 100, evaluatedSolutions=0;
    bool terminate = false;
    int random_index, tmpTrajPoint, tmpTrajMatrix[NUM_OF_ITEMS+1], accSolutionThreshold;
    float tmpSpeedMatrix[NUM_OF_ITEMS+1], randomSpeed;
    float accProbability, probThreshold=0.5, Temperature=0.5, coolingTemp=0.02f;
    trajectory_cost_t curTrajCost, curSolutionTrajCost;

    //tmpTrajMatrix = (int *) malloc(num_waypoints * sizeof(int));
    //tmpSpeedMatrix = (float *) malloc(num_waypoints * sizeof(float));

    tmpTrajMatrix[0] = 0;
    solutionTrajMatrix[0] = 0;
    tmpSpeedMatrix[0] = solSpeedMatrix[0];

    /* Initialise rand */
    //std::srand ( unsigned ( std::time(0) ) );

    if (startRandom == true) {
        /*
        // Initialise structures 
        std::vector<int> tempVector;

        // Start with random trajectory

        for (int i=1; i<num_waypoints; i++) {
            tempVector.push_back(i); // Initialize with the waypoints index
        }

        // using built-in random generator to generate initial point
        std::random_shuffle (tempVector.begin(), tempVector.end());
        */

        for (int i=1; i<num_waypoints; i++) {
            //tmpTrajMatrix[i] = tempVector[i-1];
            //solutionTrajMatrix[i] = tempVector[i-1];
            tmpSpeedMatrix[i] = solSpeedMatrix[0];
        }
    } else {
        /* solution should already be in solution solutionTrajMatrix */
        for (int i=1; i<num_waypoints; i++) {
            tmpTrajMatrix[i] = solutionTrajMatrix[i];
            tmpSpeedMatrix[i] = solSpeedMatrix[0];
        }
    }

    accSolutionThreshold = (int) round(num_waypoints * 0.1); /* Acceptable solution if less or equal of 10% of missed deadlines */

    curSolutionTrajCost = calculateTrajectoryCost(uploadedWpsList, num_waypoints, solutionTrajMatrix, tmpSpeedMatrix);

    while (terminate == false) {

        for (int curIter=0; curIter < iterationsMax; curIter++) {
            /* Generate new solution */
            random_index = rand() % (num_waypoints-1);
            //Don't allow index 0 to be swapped
            while ((random_index == 0)){
                random_index = rand() % (num_waypoints-1); /* Mod sets the result within the required range */
            }

            /* Swap points */
            tmpTrajPoint = tmpTrajMatrix[random_index];
            tmpTrajMatrix[random_index] = tmpTrajMatrix[random_index+1];
            tmpTrajMatrix[random_index+1] = tmpTrajPoint;

            /* get random speed too */
            //randomSpeed = INC_FLIGHT_VELOCITY * (std::rand() % FLIGHT_VELOCITY_STEPS);
            randomSpeed = solSpeedMatrix[0];
            tmpSpeedMatrix[random_index] = randomSpeed;

            /* Calculate feasibility and cost */
            curTrajCost = calculateTrajectoryCost(uploadedWpsList, num_waypoints, tmpTrajMatrix, tmpSpeedMatrix);

            evaluatedSolutions++;
            /* Check improvement and acceptance */
            if ((curTrajCost.missedDeadlines < curSolutionTrajCost.missedDeadlines) || ((curTrajCost.requiredEnergy < curSolutionTrajCost.requiredEnergy) && (curTrajCost.missedDeadlines == curSolutionTrajCost.missedDeadlines))) {
                curSolutionTrajCost = curTrajCost;

                for (int i=1; i<num_waypoints; i++) {
                    solutionTrajMatrix[i] = tmpTrajMatrix[i];
                    solSpeedMatrix[i] = tmpSpeedMatrix[i];
                }

            //} else if (newEnergy == minEnergy) {
            } else {
                /* if random [ 0, 1 ] ≥ accProbability then accept -- Don't like that */
                /* Metropolis rule */
                accProbability = expf(-((curTrajCost.requiredEnergy < curSolutionTrajCost.requiredEnergy)/Temperature));

                if (accProbability > probThreshold) {
                    curSolutionTrajCost = curTrajCost;

                    for (int i=1; i<num_waypoints; i++) {
                        solutionTrajMatrix[i] = tmpTrajMatrix[i];
                        solSpeedMatrix[i] = tmpSpeedMatrix[i];
                    }
                }
            }
        }

        /* Adjust temperature */
        Temperature -= coolingTemp;

        // Terminate when there are zero missed deadlines
        /* Evaluate termination condition */
        if (curSolutionTrajCost.missedDeadlines <= accSolutionThreshold) {
            terminate = true;
        } else if (Temperature <= 0.0f) {
            terminate = true;
        }
    }

    DEBUG_PRINT("SA evaluated %d\n",evaluatedSolutions);
    //free(tmpTrajMatrix);
    //free(tmpSpeedMatrix);

    return curSolutionTrajCost;
}

//Sets up all the variables required for the mincost function, and returns the estimated minimum cost route
trajectory_cost_t calc_solution(mission_waypoint_t *uploadedWpsList, int num_waypoints, int algoId, mission_waypoint_t *finalWpsList)
{
    //int *visitedNodes, *solutionTrajMatrix, *nnSolutionTrajMatrix;
    //float *departureSpeedMatrix; //, solutionSpeed=0.0;
    trajectory_cost_t solutionTrajCost = {.requiredEnergy = INFINITY, .missedDeadlines = num_waypoints, .avgDelay = 0.0};
    int i;
    //trajectory_cost_t tmpSolutionTrajCost;

    int solutionTrajMatrix[NUM_OF_ITEMS+1], nnSolutionTrajMatrix[NUM_OF_ITEMS+1];
    float departureSpeedMatrix[NUM_OF_ITEMS+1];
    /*
    trajectory = (int *) malloc(num_waypoints * sizeof(int));
    visitedNodes = (int *) malloc(num_waypoints * sizeof(int));
    solutionTrajMatrix = (int *) malloc(num_waypoints * sizeof(int));
    nnSolutionTrajMatrix = (int *) malloc(num_waypoints * sizeof(int));
    departureSpeedMatrix = (float *) malloc(num_waypoints * sizeof(float));
    */

    //DEBUG_PRINT("\nThe takeoff weight is: %f kg\n",uploadedWpsList[0].waypoint.payload_weight);
    
    if (algoId == 1) {
        /*** CALL MINCOST/NEAREST NEIGHBOUR ***/
        
        /*
        //Allow for variable speed
        for (float tmpSpeed = MIN_FLIGHT_VELOCITY; tmpSpeed <= MAX_FLIGHT_VELOCITY; tmpSpeed += INC_FLIGHT_VELOCITY) {
            
            // Re-initialize input data for greedy
            for (i=0; i<num_waypoints; i++){
                visitedNodes[i] = 0;
                nnSolutionTrajMatrix[i] = 0;
            }

            //Check for improvements
            if (tmpSpeed == MIN_FLIGHT_VELOCITY){
                solutionTrajCost=tmpSolutionTrajCost;
            } else if (tmpSolutionTrajCost.missedDeadlines <= solutionTrajCost.missedDeadlines){
                solutionTrajCost=tmpSolutionTrajCost;
            } else {

            }
        }
        */
        solutionTrajCost = my_solution_nn(uploadedWpsList, num_waypoints, nnSolutionTrajMatrix, departureSpeedMatrix, MIN_FLIGHT_VELOCITY);

        //Print Results
        DEBUG_PRINT("\nMy Nearest Neighbour Chosen solution is:\n");
        for (i=0; i<num_waypoints; i++) {
            DEBUG_PRINT("%d--->", nnSolutionTrajMatrix[i]);
            finalWpsList[i] = uploadedWpsList[nnSolutionTrajMatrix[i]]; // Initialize with the waypoints index
            finalWpsList[i].departureSpeed = departureSpeedMatrix[i];
        }
        DEBUG_PRINT("0\n");
        DEBUG_PRINT("Number of missed deadlines: %d\n\n", solutionTrajCost.missedDeadlines);
    } else {
        for (i=0; i<num_waypoints; i++) {
            nnSolutionTrajMatrix[i] = i; 
        }    
    }    

    //DEBUG_PRINT("Energy consumption: %f, number of missed deadlines: %d, Avg Delay %f\n\n",
    //       solutionTrajCost.requiredEnergy, solutionTrajCost.missedDeadlines, solutionTrajCost.avgDelay);

    if (algoId == 2) {
        /*** CALL SIMULATED ANNEALING ***/
        /* Re-initialize input data for SA */
        for (i=0; i<num_waypoints; i++){
            departureSpeedMatrix[i] = 5.0;
            
            //solutionTrajMatrix[i] = 0; -- Random
            solutionTrajMatrix[i] = nnSolutionTrajMatrix[i];
        }

        solutionTrajCost = solution_sa(uploadedWpsList, num_waypoints, solutionTrajMatrix, departureSpeedMatrix, false);

        DEBUG_PRINT("SA Chosen solution is:\n");
        for (i=0; i<num_waypoints; i++) {
            DEBUG_PRINT("%d--->", solutionTrajMatrix[i]);
            finalWpsList[i] = uploadedWpsList[solutionTrajMatrix[i]]; // Initialize with the waypoints index
            finalWpsList[i].departureSpeed = departureSpeedMatrix[i];
        }
        DEBUG_PRINT("0\n");

        //DEBUG_PRINT("Energy consumption: %f, number of missed deadlines: %d, Avg Delay %f\n\n",
        //	solutionTrajCost.requiredEnergy, solutionTrajCost.missedDeadlines, solutionTrajCost.avgDelay);
        DEBUG_PRINT("Number of missed deadlines: %d\n\n", solutionTrajCost.missedDeadlines);
    }

    /*
    free(trajectory);
    free(visitedNodes);
    free(solutionTrajMatrix);
    free(nnSolutionTrajMatrix);
    free(departureSpeedMatrix);
    */

    return solutionTrajCost;
}


uint32_t /* Returns elapsed time in ms */ 
exec_scenario (float input_benchmark[15][7], int numItems)
{
    //mission_waypoint_t *uploadedWpsList, *finalWpsList;
    mission_waypoint_t uploadedWpsList[NUM_OF_ITEMS+2], finalWpsList[NUM_OF_ITEMS+2];
    mission_waypoint_t oneWaypoint;
    trajectory_cost_t solutionTrajCost;
    struct mission_item_s mission_item;
    int numOfWaypoints, listIndex=0, userId=0, columnIndex=0;
    char userChar = 'A';
    uint32_t start_ms, finish_ms; 
    
    start_ms = xTaskGetTickCount(); //T2M(xTaskGetTickCount());
    DEBUG_PRINT("start_ms is %lu\n",start_ms);
    /*    
    uploadedWpsList = (mission_waypoint_t *) malloc((numItems+2) * sizeof(mission_waypoint_t));
    finalWpsList = (mission_waypoint_t *) malloc((numItems+2) * sizeof(mission_waypoint_t));
    */

    mission_item.id = userId++;
    mission_item.user = userChar++;
    mission_item.lat = 47.397751f; //telemetry->position().latitude_deg;
    mission_item.lon = 8.545608f; //telemetry->position().longitude_deg;
    mission_item.altitude = 20.0; //telemetry->position().absolute_altitude_m;
    mission_item.speed = 5.0f;
    mission_item.payload_weight = 0.0;
    mission_item.deadline = 0.0f;
    oneWaypoint.waypoint = mission_item;
    oneWaypoint.originalIndex = 0;
    uploadedWpsList[listIndex++] = oneWaypoint;
    numOfWaypoints = 1;

    if (numItems > 0) {
        /* READ MISSION ITEMS */
        for (int i = 0; i < numItems; i++) {
                mission_item.id = userId++;
                mission_item.user = userChar++;

                columnIndex=0;

                mission_item.lat = input_benchmark[i][columnIndex++];
                mission_item.lon = input_benchmark[i][columnIndex++];
                mission_item.altitude = input_benchmark[i][columnIndex++];
                mission_item.speed = input_benchmark[i][columnIndex++];
                mission_item.deadline = input_benchmark[i][columnIndex++];
                mission_item.payload_weight = input_benchmark[i][columnIndex++];

                uploadedWpsList[0].waypoint.payload_weight += mission_item.payload_weight;

                oneWaypoint.waypoint = mission_item;
                oneWaypoint.originalIndex = i+1;
                //printf("Waypoint weight is %f deadline is %f\n",oneWaypoint.waypoint.payload_weight,oneWaypoint.waypoint.deadline);
                uploadedWpsList[listIndex++] = oneWaypoint;
                numOfWaypoints++; 
        }
        /*
        //Print the number of waypoints received
        DEBUG_PRINT("Num of Waypoints is: %d\n", numOfWaypoints);

        //Print the received waypoints
        for (int i = 0; i < numOfWaypoints; i++){
            DEBUG_PRINT("Waypoint %d: Lat: %f Lon: %f Waypoint weight is %f deadline is %f\n",
                i, uploadedWpsList[i].waypoint.lat, uploadedWpsList[i].waypoint.lon,uploadedWpsList[i].waypoint.payload_weight,uploadedWpsList[i].waypoint.deadline);
        }
        */

        //printf("\nCalculating best flight path:\n");

        /* Adding return to takeoff */
        uploadedWpsList[listIndex++] = uploadedWpsList[0];
        uploadedWpsList[numOfWaypoints].waypoint.payload_weight = 0;

        /* Calculate the optimal trajectory */
        solutionTrajCost = calc_solution(uploadedWpsList, numOfWaypoints, 1, finalWpsList);
        finish_ms = xTaskGetTickCount(); //T2M(xTaskGetTickCount());
        DEBUG_PRINT("finish_ms is %lu\n",finish_ms);
        //DEBUG_PRINT("Energy consumption: %f, number of missed deadlines: %d, Avg Delay %f\n\n",
	    //    solutionTrajCost.requiredEnergy, solutionTrajCost.missedDeadlines, solutionTrajCost.avgDelay);
        //DEBUG_PRINT("Number of missed deadlines: %d\n\n", solutionTrajCost.missedDeadlines);

        /*
        numOfWaypoints++;

        //Print the final trajectory
        for (int i = 0; i < numOfWaypoints; i++){
            printf("Waypoint %d: Lat: %f Lon: %f\n",
                i, finalWpsList[i].waypoint.lat, finalWpsList[i].waypoint.lon);
        }
        */
    }
    
    /*
    free(uploadedWpsList);
    free(finalWpsList); 
    */

    return (finish_ms - start_ms);
}
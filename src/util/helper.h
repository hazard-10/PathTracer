#include "../lib/vec2.h"
#include "../lib/vec3.h"
#include "../lib/vec4.h"
#include "../lib/spectrum.h"

float cross2D(Vec3 a, Vec3 b);

bool segmentCross(Vec3 a1, Vec3 a2, Vec3 b1, Vec3 b2);
bool inDiamond(Vec3 test, int x, int y);
bool exitDiamond_vertical( int x, int y, Vec3 lower, Vec3 upper);

/* flag used to indicate whether va or vb should stay outside the diamond
    'a' means  va stay outside 
	'b' means  vb stay outside
	'n' means  no checking

*/
bool exitDimaond_nonVertical( int x, int y, Vec3 va, Vec3 vb, char flag);
Vec2 StartingDiamondPos_NoVertical_No45Deg(Vec3 start, Vec3 end);
Vec2 StartingDiamondPos_Vertical(Vec3 start, Vec3 end);

double triangleArea(Vec3 A, Vec3 B, Vec3 C);
bool inTriangle(Vec3 test, Vec3 A, Vec3 B, Vec3 C);
double interpolateZ(Vec3 test, Vec3 A, Vec3 B, Vec3 C);

std::array< float, 5> interpolateAttr(Vec3 test, Vec3 A, Vec3 B, Vec3 C, 
										std::array< float, 5> attrA, 
										std::array< float, 5> attrB, 
										std::array< float, 5> attrC);
std::array< float, 5> interpolateAttrWInverse(Vec3 test, Vec3 A, Vec3 B, Vec3 C, 
										std::array< float, 5> attrA, 
										std::array< float, 5> attrB, 
										std::array< float, 5> attrC,
										float ivA, float ivB, float ivC);
Vec3 barycentricCoord(Vec3 test, Vec3 A, Vec3 B, Vec3 C);

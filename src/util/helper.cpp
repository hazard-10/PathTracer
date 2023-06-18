#include "helper.h"
#include "../src/rasterizer/pipeline.h"
#include <iostream>



float cross2D(Vec3 a, Vec3 b){
	return a.x * b.y - a.y * b.x;
}

bool segmentCross(Vec3 a1, Vec3 a2, Vec3 b1, Vec3 b2){
	Vec3 a(a2.x - a1.x, a2.y - a1.y, 0.f);  // direction vector of segment a
	Vec3 b(b2.x - b1.x, b2.y - b1.y, 0.f);  // direction vector of segment b
	float cross_ab = cross2D(a, b);
	if(cross_ab == 0.f){ // parallel
		return false;
	}
	Vec3 c(b1.x - a1.x, b1.y - a1.y, 0.f);  // vector from a1 to b1
	float t = cross2D(c, b) / cross_ab;
	float u = cross2D(c, a) / cross_ab;
	
	return t >= 0.f && t <= 1.f && u >= 0.f && u <= 1.f ;

}

bool inDiamond(Vec3 test, int x, int y){
	// Assume no over the camera bound situation
	if(test.x + 0.5f >= x && test.x + 0.5f < x + 1 && test.y + 0.5f >= y && test.y + 0.5f < y + 1){
		return true;
	}
	return false;
}

bool exitDiamond_vertical( int x, int y, Vec3 lower, Vec3 upper){
	if(lower.x + 0.5f >= x && lower.x + 0.5f < x + 1){ // x in dimamond range
		bool lowerIn = inDiamond(lower, x, y);
		bool upperIn = inDiamond(upper, x, y);
		if(lowerIn && upperIn){
			return false;
		}
		if(!lowerIn && !upperIn){
			if(lower.y <= y && upper.y >= y){
				return true;
			}
			return false; // Both above the diamond
		}
		return true; // One in one out
	}
	return false;
}

/* flag used to indicate whether va or vb should stay outside the diamond
    'a' means  va stay outside 
	'b' means  vb stay outside
	'n' means  no checking

*/
bool exitDimaond_nonVertical( int x, int y, Vec3 va, Vec3 vb, char flag){ 
	// float slope = (vb.y - va.y) / (vb.x - va.x);
	bool aIn = inDiamond(va, x, y);
	bool bIn = inDiamond(vb, x, y);
	if(aIn && bIn){
		return false;
	}
	else if(!aIn && !bIn){ // at least cross twice
		// construct 4 outline segments from the diamond
		Vec3 top = Vec3(x*1.0f, y + 0.5f, 0.f);
		Vec3 bottom = Vec3(x*1.0f, y - 0.5f, 0.f);
		Vec3 left = Vec3(x - 0.5f, y*1.0f, 0.f);
		Vec3 right = Vec3(x + 0.5f, y*1.0f, 0.f);
		
		int count = 0;
		if(segmentCross(va, vb, left, top )){
			count++;	
		}
		if(segmentCross(va, vb, right, top )){
			count++;	
		}
		if(segmentCross(va, vb, left, bottom )){
			count++;	
		}
		if(segmentCross(va, vb, right, bottom )){
			count++;	
		}
		return count == 2;

	}else{
		if (flag == 'n') return true;
		else if (flag == 'a') return !aIn;
		else return !bIn;
	}
}


Vec3 shorternSegmentByOne(Vec3 start, Vec3 end){ 
	float slope = (end.y - start.y) / (end.x - start.x);
	// return the new end point
	if(slope >= 0){
		if(end.x > start.x){
			return Vec3(end.x - 1.f, end.y - slope, end.z);
		}else{
			return Vec3(end.x + 1.f, end.y + slope, end.z);
		}
	}else{
		if(end.x > start.x){
			return Vec3(end.x - 1.f, end.y - slope, end.z);
		}else{
			return Vec3(end.x + 1.f, end.y + slope, end.z);
		}
	}
}


Vec2 EndDiamondPos_NoVertical_No45Deg(Vec3 end, Vec3 start){
	// if end is precisely on the center of a pixel 
	if(end.x == floor(end.x) && end.y == floor(end.y)){
		Vec3 newEnd = shorternSegmentByOne(start, end);
		return StartingDiamondPos_NoVertical_No45Deg(newEnd, start);
	}
	// if end x or y is on the line of center
	else if(end.x == floor(end.x)){ 
		int x_int = int(floor(end.x));
		int y1 = int(floor(end.y));
		int y2 = int(ceil(end.y));
		if(inDiamond(end, x_int, y1)){
			Vec3 newEnd = shorternSegmentByOne(start, end);
			return StartingDiamondPos_NoVertical_No45Deg(newEnd, start);
		}
		else{
			return Vec2(end.x, y2*1.0f);
		}
	}
	else if(end.y == floor(end.y)){
		int x1 = int(floor(end.x));
		int x2 = int(ceil(end.x));
		int y_int = int(floor(end.y));
		if(inDiamond(end, x1, y_int)){
			Vec3 newEnd = shorternSegmentByOne(start, end);
			return StartingDiamondPos_NoVertical_No45Deg(newEnd, start);
		}
		else{
			return Vec2(x2*1.0f, end.y);
		}
	}
	// next check if it is in either four of the adjacent diamonds.
	else{ 
		int x1 = int(floor(end.x));
		int x2 = int(ceil(end.x));
		int y1 = int(floor(end.y));
		int y2 = int(ceil(end.y));
		if(inDiamond(end, x1, y1)){ 
			Vec3 newEnd = shorternSegmentByOne(start, end);
			return StartingDiamondPos_NoVertical_No45Deg(newEnd, start);
		}
		else if(inDiamond(end, x1, y2)){ 
			Vec3 newEnd = shorternSegmentByOne(start, end);
			return StartingDiamondPos_NoVertical_No45Deg(newEnd, start);
		}
		else if(inDiamond(end, x2, y1)){ 
			Vec3 newEnd = shorternSegmentByOne(start, end);
			return StartingDiamondPos_NoVertical_No45Deg(newEnd, start);
		}
		else if(inDiamond(end, x2, y2)){ 
			Vec3 newEnd = shorternSegmentByOne(start, end);
			return StartingDiamondPos_NoVertical_No45Deg(newEnd, start);
		}
        // if not in either diamonds, return the one it crosses
		else if (exitDimaond_nonVertical(x1, y1, start, end, 'n')){
			return Vec2(x1*1.0f, y1*1.0f);
		}
		else if (exitDimaond_nonVertical(x1, y2, start, end, 'n')){
			return Vec2(x1*1.0f, y2*1.0f);
		}
		else if (exitDimaond_nonVertical(x2, y1, start, end, 'n')){
			return Vec2(x2*1.0f, y1*1.0f);
		}
		else if (exitDimaond_nonVertical(x2, y2, start, end, 'n')){
			return Vec2(x2*1.0f, y2*1.0f);
		}
		else{
			return Vec2(-1.f, -1.f);
		}

	}
		
}

Vec2 StartingDiamondPos_NoVertical_No45Deg(Vec3 start, Vec3 end){
	// if start is precisely on the center of a pixel
	if(start.x == floor(start.x) && start.y == floor(start.y)){
		return Vec2(start.x, start.y);
	}
	// if start x or y is on the line of center
	else if(start.x == floor(start.x)){
		int x_int = int(floor(start.x));
		int y1 = int(floor(start.y));
		int y2 = int(ceil(start.y));
		if(inDiamond(start, x_int, y1)){
			return Vec2(start.x, y1*1.0f);
		}
		else{
			return Vec2(start.x, y2*1.0f);
		}
	}
	else if(start.y == floor(start.y)){
		int x1 = int(floor(start.x));
		int x2 = int(ceil(start.x));
		if(inDiamond(start, x1, int(start.y))){
			return Vec2(x1*1.0f, start.y);
		}
		else{
			return Vec2(x2*1.0f, start.y);
		}
	}
	// next check if it is in either four of the adjacent diamonds.
    else{
        int x1 = int(floor(start.x));
        int x2 = int(ceil(start.x));
        int y1 = int(floor(start.y));
        int y2 = int(ceil(start.y));
        if(inDiamond(start, x1, y1)){
            return Vec2(x1*1.0f, y1*1.0f);
        }
        else if(inDiamond(start, x1, y2)){
            return Vec2(x1*1.0f, y2*1.0f);
        }
        else if(inDiamond(start, x2, y1)){
            return Vec2(x2*1.0f, y1*1.0f);
        }
        else if(inDiamond(start, x2, y2)){
            return Vec2(x2*1.0f, y2*1.0f);
        }
        // if not in either diamonds, return the one it crosses
        else if (exitDimaond_nonVertical(x1, y1, start, end, 'n')){
            return Vec2(x1*1.0f, y1*1.0f);
        }
        else if (exitDimaond_nonVertical(x1, y2, start, end, 'n')){
            return Vec2(x1*1.0f, y2*1.0f);
        }
        else if (exitDimaond_nonVertical(x2, y1, start, end, 'n')){
            return Vec2(x2*1.0f, y1*1.0f);
        }
        else if (exitDimaond_nonVertical(x2, y2, start, end, 'n')){
            return Vec2(x2*1.0f, y2*1.0f);
        }
        else{
			// this shouldn't happen
            return Vec2(-1.f, -1.f);
        }

        
    }
}


Vec2 StartingDiamondPos_Vertical(Vec3 start, Vec3 end){ // start is lower
	// if start is precisely on the center of a pixel
	if(start.x == floor(start.x) && start.y == floor(start.y)){
		return Vec2(start.x, start.y);
	}
	// if start x or y is on the line of center
	else if(start.x == floor(start.x)){
		int x_int = int(floor(start.x));
		int y1 = int(floor(start.y));
		int y2 = int(ceil(start.y));
		if(inDiamond(start, x_int, y1)){
			return Vec2(start.x, y1*1.0f);
		}
		else{
			return Vec2(start.x, y2*1.0f);
		}
	}
	else if(start.y == floor(start.y)){
		int x1 = int(floor(start.x));
		int x2 = int(ceil(start.x));
		if(inDiamond(start, x1, int(start.y))){
			return Vec2(x1*1.0f, start.y);
		}
		else{
			return Vec2(x2*1.0f, start.y);
		}
	}
	// next check if it is in either four of the adjacent diamonds.
    else{
        int x1 = int(floor(start.x));
        int x2 = int(ceil(start.x));
        int y1 = int(floor(start.y));
        int y2 = int(ceil(start.y));
        if(inDiamond(start, x1, y1)){
            return Vec2(x1*1.0f, y1*1.0f);
        }
        else if(inDiamond(start, x1, y2)){
            return Vec2(x1*1.0f, y2*1.0f);
        }
        else if(inDiamond(start, x2, y1)){
            return Vec2(x2*1.0f, y1*1.0f);
        }
        else if(inDiamond(start, x2, y2)){
            return Vec2(x2*1.0f, y2*1.0f);
        }
        // if not in either diamonds, return the one it crosses
        else if (exitDiamond_vertical(x1, y1, start, end)){
            return Vec2(x1*1.0f, y1*1.0f);
        }
        else if (exitDiamond_vertical(x1, y2, start, end)){
            return Vec2(x1*1.0f, y2*1.0f);
        }
        else if (exitDiamond_vertical(x2, y1, start, end)){
            return Vec2(x2*1.0f, y1*1.0f);
        }
        else if (exitDiamond_vertical(x2, y2, start, end)){
            return Vec2(x2*1.0f, y2*1.0f);
        }
        else{
			// this shouldn't happen
            return Vec2(-1.f, -1.f);
        }

    }
}

double triangleArea(Vec3 A, Vec3 B, Vec3 C) {
    return abs((A.x*(B.y - C.y) + B.x*(C.y - A.y) + C.x*(A.y - B.y)) / 2.0);
}


bool inTriangle(Vec3 test, Vec3 A, Vec3 B, Vec3 C){
	double error = .05;
	double totalArea = triangleArea(A, B, C);
	double area1 = triangleArea(test, B, C);
	double area2 = triangleArea(A, test, C);
	double area3 = triangleArea(A, B, test);
	return abs(totalArea -( area1 + area2 + area3)) <= error;
}

double interpolateZ(Vec3 test, Vec3 A, Vec3 B, Vec3 C){
	double totalArea = triangleArea(A, B, C);
	double area1 = triangleArea(test, B, C);
	double area2 = triangleArea(A, test, C);
	double area3 = triangleArea(A, B, test);
	return (area1 * A.z + area2 * B.z + area3 * C.z) / totalArea;
}

std::array< float, 5> interpolateAttr(Vec3 test, Vec3 A, Vec3 B, Vec3 C, 
										std::array< float, 5> attrA, 
										std::array< float, 5> attrB, 
										std::array< float, 5> attrC){
	double totalArea = triangleArea(A, B, C);
	double area1 = triangleArea(test, B, C);
	double area2 = triangleArea(A, test, C);
	double area3 = triangleArea(A, B, test);
	std::array< float, 5> result;
	for(int i = 0; i < 5; i++){
		result[i] = float((area1 * attrA[i] + area2 * attrB[i] + area3 * attrC[i]) / totalArea);
	}
	return result;
}

std::array< float, 5> interpolateAttrWInverse(Vec3 test, Vec3 A, Vec3 B, Vec3 C, 
										std::array< float, 5> attrA, 
										std::array< float, 5> attrB, 
										std::array< float, 5> attrC,
										float ivA, float ivB, float ivC){
	double totalArea = triangleArea(A, B, C);
	double area1 = triangleArea(test, B, C);
	double area2 = triangleArea(A, test, C);
	double area3 = triangleArea(A, B, test);

	Vec3 baryPixel = Vec3(float(area1 / totalArea), 
					float(area2 / totalArea), 
					float(area3 / totalArea));
									
	float interpolateInverse = 	float(ivA * baryPixel.x) + 
								float(ivB * baryPixel.y) +  
								float(ivC * baryPixel.z);

	std::array< float, 5> result;
	for(int i = 0; i < 5; i++){
		result[i] = float((area1 * attrA[i] * ivA + area2 * attrB[i] * ivB + area3 * attrC[i] * ivC) / totalArea);
	
		result[i] = result[i] / interpolateInverse;
	}
	return result;
}

Vec3 barycentricCoord(Vec3 test, Vec3 A, Vec3 B, Vec3 C){
	double totalArea = triangleArea(A, B, C);
	double area1 = triangleArea(test, B, C);
	double area2 = triangleArea(A, test, C);
	double area3 = triangleArea(A, B, test);
	return Vec3(float(area1 / totalArea), 
				float(area2 / totalArea), 
				float(area3 / totalArea));
}




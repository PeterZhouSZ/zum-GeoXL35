#ifndef BOX_H
#define BOX_H

#include "CommonHdrXWu.h"

namespace X4
{
	struct Box {
		Matrix3f basis;
		Vector3f coeffs;
		Vector3f centroid;
		Vector3f cenref;
		float32 radiusref;
		int32 label;
		Box(){
			basis[0] = makeVector3f(1, 0, 0);
			basis[1] = makeVector3f(0, 1, 0);
			basis[2] = makeVector3f(0, 0, 1);
			coeffs = makeVector3f(0, 0, 0);
			centroid = makeVector3f(0, 0, 0);
			cenref = makeVector3f(0, 0, 0);
			radiusref = 0;
			label = -1;
		}
	};
}
#endif
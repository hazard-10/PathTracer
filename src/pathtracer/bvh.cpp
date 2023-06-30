
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

#include <stack>
#include <iostream>

namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};

struct customBinData{
	BBox bin_bbox;
	std::vector<uint32_t> bin_prims; // stores the indices of the primitives in the bin
	float totalSurfaceArea;
};

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.

	// Keep these
    nodes.clear();
    primitives = std::move(prims);
	
	BBox sceneBBox;

	// 1. Compute the bounding box of all primitives in the scene and for self
	for (uint32_t i = 0; i < primitives.size(); i++) {
		sceneBBox.enclose(primitives[i].bbox());
	}

	// 2. Build the BVH recursively
	// 2.1 Initialize the root node
	Node rootNode;
	nodes.push_back(rootNode);
	nodes[root_idx].bbox = sceneBBox;
	nodes[root_idx].start = 0;
	nodes[root_idx].size = primitives.size();
	std::cout<<" nodes[0] size " << nodes[0].size <<std::endl;

	
	uint32_t numBinsPerDim = 8;
	if(nodes[root_idx].size > max_leaf_size){
		buildRecursive(root_idx, numBinsPerDim, max_leaf_size);
	}else{
		nodes[root_idx].l = nodes[root_idx].r = 0;
	}

	
}


	// helper function to build the BVH recursively

template<typename Primitive>
void BVH<Primitive>::buildRecursive(size_t parentNodeIndex, uint32_t numBinsPerDim, size_t max_leaf_size ) { 
	Node& parentNode = nodes[parentNodeIndex];
	std::cout<<" start buildRecursive " << parentNode.start <<" - "<< parentNode.size<<std::endl;
	// parentNode is not a leaf

	// given the primitives from one of the previous partitions
	BBox parentBBox = parentNode.bbox;
	float lowerestCost = INFINITY;
	uint32_t bestAxis = 0;
	uint32_t bestSplit = 0;
	customBinData bestLeftBin;
	customBinData bestRightBin;
	
	// initialize bin data, need the range of the primitives
	for(uint32_t axis = 0; axis < 3; axis++){
		
		std::cout<<"Initialization Axis= "<<axis<<std::endl;
		// x,y,z
		std::vector<customBinData> bins(numBinsPerDim);
		// compute the bin span
		float binSpanMin = parentBBox.min[axis];
		float binSpanMax = parentBBox.max[axis];
		float binLength = (binSpanMax - binSpanMin) / numBinsPerDim;
		
		std::cout<<" binSpanMin " << binSpanMin <<" binSpanMax "<< binSpanMax << " binLength "<<binLength<<std::endl;
		// assign primitives to bins, compute the bin bbox
		for(size_t p_index = parentNode.start; p_index < parentNode.start + parentNode.size; p_index++){
			BBox currPrimBox = primitives[p_index].bbox();
			float pCoord = currPrimBox.center()[axis];
			uint32_t binIndex = uint32_t(std::floor((pCoord - binSpanMin) / binLength));
			bins[binIndex].bin_prims.push_back(uint32_t(p_index));
			bins[binIndex].bin_bbox.enclose(currPrimBox);
			bins[binIndex].totalSurfaceArea += currPrimBox.surface_area();
		}
		std::cout<<"Computing cost of each bin: "<<std::endl;

		// compute the cost of each bin, record the best split
		for(uint32_t split = 1; split<numBinsPerDim; split++){
			std::cout<<"\tsplit index is "<<split<<std::endl;
			customBinData leftBBox;
			customBinData rightBBox;
			uint32_t leftNumPrims = 0;
			uint32_t rightNumPrims = 0;
			float leftAvgBinCost = 0;
			float rightAvgBinCost = 0;
			for(uint32_t i = 0; i < split; i++){
				leftNumPrims += uint32_t(bins[i].bin_prims.size());
				leftBBox.bin_bbox.enclose(bins[i].bin_bbox);
				leftBBox.totalSurfaceArea += bins[i].totalSurfaceArea;
				leftBBox.bin_prims.insert(leftBBox.bin_prims.end(), bins[i].bin_prims.begin(), bins[i].bin_prims.end());
				if(bins[i].bin_bbox.surface_area() > 0)	leftAvgBinCost += bins[i].totalSurfaceArea / bins[i].bin_bbox.surface_area();
			}
			for(uint32_t i = split; i < numBinsPerDim; i++){
				rightNumPrims += uint32_t(bins[i].bin_prims.size());
				rightBBox.bin_bbox.enclose(bins[i].bin_bbox);
				rightBBox.totalSurfaceArea += bins[i].totalSurfaceArea;
				rightBBox.bin_prims.insert(rightBBox.bin_prims.end(), bins[i].bin_prims.begin(), bins[i].bin_prims.end());
				if(bins[i].bin_bbox.surface_area() > 0)	rightAvgBinCost += bins[i].totalSurfaceArea / bins[i].bin_bbox.surface_area();
			}
			float cost = leftBBox.bin_bbox.surface_area() * leftNumPrims + rightBBox.bin_bbox.surface_area() * rightNumPrims;
			// cost /= parentBBox.surface_area();
			std::cout<<"\t\tcost "<<cost<<std::endl;
			std::cout<<"\t\tleftNum "<<leftNumPrims<<std::endl;
			std::cout<<"\t\trightNum "<<rightNumPrims<<std::endl;
			std::cout<<"\t\tlowerestCost "<<lowerestCost<<std::endl;
			if(cost < lowerestCost){ // cost will be non-zero
				lowerestCost = cost;
				bestAxis = axis;
				bestSplit = split;
				bestLeftBin = leftBBox;
				bestRightBin = rightBBox;
			}
		}
	}
	std::cout<<"bestAxis "<<bestAxis<<std::endl;
	std::cout<<"bestSplit "<<bestSplit<<std::endl;
	std::cout<<"bestLeftBin "<<bestLeftBin.bin_prims.size()<<std::endl;
	std::cout<<"bestRightBin "<<bestRightBin.bin_prims.size()<<std::endl;
	std::cout<<"lowerestCost "<<lowerestCost<<std::endl;
	// partition the primitives into left and right
	auto isPrimLeft = [&](Primitive& p){
		Vec3 pCenter = p.bbox().center();
		float binSpanMin = parentBBox.min[bestAxis];
		float binSpanMax = parentBBox.max[bestAxis];
		float binLength = (binSpanMax - binSpanMin) / numBinsPerDim;
		return pCenter[bestAxis] < binSpanMin + binLength * bestSplit;
	};
	std::partition(primitives.begin() + parentNode.start, 
					primitives.begin() + parentNode.start + parentNode.size, isPrimLeft);
	/*
	1. if left or right has more than max_leaf_size, continue recurse on left / right
	2. if left or right has less than max_leaf_size, make it a leaf node
	3. left or right can't be zero, 
	*/

	Node leftNode;
	leftNode.bbox = bestLeftBin.bin_bbox;
	leftNode.start = parentNode.start;
	leftNode.size = bestLeftBin.bin_prims.size();
	
	Node rightNode;
	rightNode.bbox = bestRightBin.bin_bbox;
	rightNode.start = leftNode.start + leftNode.size;
	rightNode.size = bestRightBin.bin_prims.size();
	
	nodes.push_back(leftNode);
	nodes.push_back(rightNode);
	size_t leftNodeIndex = nodes.size() - 2;
	size_t rightNodeIndex = nodes.size() - 1;
	parentNode.l = leftNodeIndex;
	parentNode.r = rightNodeIndex;

	std::cout<<"Constructing nodes, maxleafsize is "<<max_leaf_size<<std::endl;

	if(bestLeftBin.bin_prims.size() > max_leaf_size){
		
		std::cout<<"left recur"<<std::endl;
		buildRecursive(leftNodeIndex, numBinsPerDim, max_leaf_size);
	}else{
		std::cout<<"left leaf"<<std::endl;
		nodes[leftNodeIndex].l = nodes[leftNodeIndex].r = nodes[leftNodeIndex].start ;
	}

	if(bestRightBin.bin_prims.size() > max_leaf_size){
		
		std::cout<<"right recur"<<std::endl;
		buildRecursive(rightNodeIndex, numBinsPerDim, max_leaf_size); 
	}else{
		std::cout<<"right leaf"<<std::endl;
		nodes[rightNodeIndex].l = nodes[rightNodeIndex].r  = nodes[rightNodeIndex].start;
	}
	std::cout<<"end of recursion"<<std::endl;

};


template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH

    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
    Trace ret;
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT

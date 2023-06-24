
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = emplace_face();
    HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
    erase_face(f_not_used);
    erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge
	

	//helper for calculating face area
	auto area = [](Vertex v1, Vertex v2, Vertex v3) {
		return cross(v2.position - v1.position, v3.position - v1.position).norm() / 2.0f;
	};

	auto list_of_halfedges = [](HalfedgeRef h){
		std::vector<HalfedgeCRef> halfedges;
		halfedges.push_back(h);
		for(auto h_it = h->next; h_it != h; h_it = h_it->next){
			halfedges.push_back(h_it);
		}
		return halfedges;
	};
	
	auto print_single_halfEdge = [](HalfedgeRef h){
		std::cout <<"halfedge id " << h->id << std::endl;
		std::cout <<"	vertex id " << h->vertex->id << std::endl;
		for(HalfedgeRef hv = h->vertex->halfedge; hv!= h; hv++){
			std::cout <<"		vertex's halfedge id has" << hv-> id << std::endl;
		}
		std::cout <<"	next id " << h->next->id << std::endl;
		std::cout <<"	twin id " << h->twin->id << std::endl;
	};

	auto print_full_mesh = [](HalfedgeRef h){
		std::cout <<"Debug log"<< std::endl;

		std::cout <<"halfedge id " << h->id << std::endl;
		std::cout <<"	face id " << h->face->id << std::endl;
		std::cout <<"	edge id " << h->edge->id << std::endl;
		std::cout <<"	vertex id " << h->vertex->id << std::endl;
		for(HalfedgeRef hv = h->vertex->halfedge; hv!= h; hv++){
			std::cout <<"		vertex's halfedge id has" << hv-> id << std::endl;
		}
		std::cout <<"	next id " << h->next->id << std::endl;
		std::cout <<"	twin id " << h->twin->id << std::endl;

		for(HalfedgeRef h_iter = h->next ; h_iter !=h; h_iter = h_iter->next){
			std::cout <<"halfedge id " << h_iter->id << std::endl;
			std::cout <<"	face id " << h->face->id << std::endl;
			std::cout <<"	edge id " << h->edge->id << std::endl;
			std::cout <<"	vertex id " << h_iter->vertex->id << std::endl;
			for(HalfedgeRef hv = h_iter->vertex->halfedge; hv!= h_iter; hv++){
				std::cout <<"		vertex's halfedge id has" << hv-> id << std::endl;
			}
			std::cout <<"	next id " << h_iter->next->id << std::endl;
			std::cout <<"	twin id " << h_iter->twin->id << std::endl;
		}

	};
	
	/* 1. bisect_edge - Create new vertex, edge and halfedge
	   2. split_face - Create new edge 1~2, face 1~2, and halfedges (2~4)
	   3. Reassign connectivity
	   For each step, 
	   		a. collect existing elements, 
			b. allocate new elements & set data, 
			c. reassign connectivity
	*/

	// a. collect required references
	HalfedgeRef h1 = e->halfedge;
	HalfedgeRef h2 = h1->twin;
	HalfedgeRef h1_next = h1->next;
	HalfedgeRef h2_next = h2->next;
	HalfedgeRef h1_2next = h1_next->next;
	HalfedgeRef h2_2next = h2_next->next;

	VertexRef v1 = h1->vertex;
	VertexRef v2 = h2->vertex;

	// print_full_mesh(h1);
	// b. Create new elements, set data
	// b.1. bisect_edge
	VertexRef v_middle = emplace_vertex();
	v_middle->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, v_middle); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h1_new = emplace_halfedge();
	interpolate_data({h1, h1->next}, h1_new); //set corner_uv, corner_normal
	HalfedgeRef h2_new = emplace_halfedge();
	interpolate_data({h2, h2->next}, h2_new); //set corner_uv, corner_normal

	// b.2 Split_face
	// new edge 1~2, face 1~2, and halfedges (2~4)
	EdgeRef e3 = emplace_edge(e->sharp); 
	FaceRef f1_new = emplace_face(false);

	HalfedgeRef h3 = emplace_halfedge();
	interpolate_data(list_of_halfedges(h1), h3);
	HalfedgeRef h3_rev = emplace_halfedge();
	interpolate_data(list_of_halfedges(h1), h3_rev);	
	
	// Prepare probably unused data
	EdgeRef e4 = emplace_edge(e->sharp);
	FaceRef f2_new = emplace_face(false);
	HalfedgeRef h4 = emplace_halfedge();
	HalfedgeRef h4_rev = emplace_halfedge();
	interpolate_data(list_of_halfedges(h2), h4);
	interpolate_data(list_of_halfedges(h2), h4_rev);
	
	// c. Reassign connectivity 
	// (careful about ordering so you don't overwrite values you may need later!)

	// c.1. non_halfedge connectivity
	v_middle->halfedge = h1_new;
	e2->halfedge = h1_new;
	e3->halfedge = h3;
	f1_new->halfedge = h3_rev;
	if(!e->on_boundary()){
		e4->halfedge = h4;
		f2_new->halfedge = h4_rev;
	}
	// c.2. halfedge connectivity
	// twin = twin_; next = next_; vertex = vertex_; edge = edge_; face = face_;
	if(e->on_boundary()){
		h1->set_tnvef(h2_new, h3, h1->vertex, e, h1->face);
		h2->set_tnvef(h1_new, h2_new, h2->vertex, e2, h2->face);
		h1_new->set_tnvef(h2, h1_next, v_middle, e2, f1_new);
		h2_new->set_tnvef(h1, h2_next, v_middle, e, h2->face);
		h1_next->set_tnvef(h1_next->twin, h3_rev, h1_next->vertex, h1_next->edge, f1_new);
		h3->set_tnvef(h3_rev, h1_2next, v_middle, e3, h1->face);
		h3_rev->set_tnvef(h3, h1_new, h1_2next->vertex, e3, f1_new);
		// double check existing face and edge connectivity
		e->halfedge = h1;
		h1->face->halfedge = h1;

		// delete unused
		erase_edge(e4);
		erase_face(f2_new);
		erase_halfedge(h4);
		erase_halfedge(h4_rev);
	}else{
		h1->set_tnvef(h2_new, h3, h1->vertex, e, h1->face);
		h2->set_tnvef(h1_new, h4, h2->vertex, e2, h2->face);
		h1_new->set_tnvef(h2, h1_next, v_middle, e2, f1_new);
		h2_new->set_tnvef(h1, h2_next, v_middle, e, f2_new);

		h1_next->set_tnvef(h1_next->twin, h3_rev, h1_next->vertex, h1_next->edge, f1_new);
		h3->set_tnvef(h3_rev, h1_2next, v_middle, e3, h1->face);
		h3_rev->set_tnvef(h3, h1_new, h1_2next->vertex, e3, f1_new);
		// second new face
		h2_next->set_tnvef(h2_next->twin, h4_rev, h2_next->vertex, h2_next->edge, f2_new);
		h4->set_tnvef(h4_rev, h2_2next, v_middle, e4, h2->face);
		h4_rev->set_tnvef(h4, h2_new, h2_2next->vertex, e4, f2_new);
		// double check existing face and edge connectivity
		e->halfedge = h1;
		h1->face->halfedge = h1;

	}
	
	return v_middle;
	
}



/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	
	(void)f;
    return std::nullopt;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	//A2L4: Extrude Face
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in extrude_helper (A2L4h)

	(void)f;
    return std::nullopt;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge

	// check if edge is boundary
	if(e->on_boundary()) return std::nullopt;

	// check if flipping would create invalid mesh, TODO
	

	// rotate edge
	// update vertex, halfedge, edge, face

	// helper function for debug and print
	auto print_halfEdge = [](HalfedgeRef h){
		std::cout <<"halfedge id " << h->id << std::endl;
		std::cout <<"	vertex id " << h->vertex->id << std::endl;
		
		
		for(HalfedgeRef hv = h->vertex->halfedge; hv!= h; hv++){
			std::cout <<"		vertex's halfedge id has" << hv-> id << std::endl;
		}

		std::cout <<"	next id " << h->next->id << std::endl;
		std::cout <<"	twin id " << h->twin->id << std::endl;
	};

	// get two updated vertices
	HalfedgeRef h1 = e->halfedge;
	HalfedgeRef h2 = e->halfedge->twin;

	
	// std::cout <<"Debug Info"<< std::endl;
	// print_halfEdge(h1);
	// for(HalfedgeRef h_iter = h1->next ; h_iter !=h1; h_iter = h_iter->next){
	// 	print_halfEdge(h_iter);
	// }
	// std::cout <<"Debug h2"<< std::endl;
	// print_halfEdge(h2);
	// for(HalfedgeRef h_iter = h2->next ; h_iter !=h2; h_iter= h_iter->next){ 
	// 	print_halfEdge(h_iter);
	// }

	//prepare vertices
	VertexRef v1_old = h1->vertex;
	VertexRef v2_old = h2->vertex;
	VertexRef v1_new = h2->next->next->vertex;
	VertexRef v2_new = h1->next->next->vertex;

	//Prepare 3 halfedges whose links will be changed
	HalfedgeRef h1_new_prev = h2->next;
	HalfedgeRef h2_new_prev = h1->next;   

	HalfedgeRef h1_new_next = h1->next->next;
	HalfedgeRef h2_new_next = h2->next->next;

	HalfedgeRef h1_old_prev = h1;
	HalfedgeRef h2_old_prev = h2;

	for(HalfedgeRef h_iter = h1->next ; h_iter !=h1; h_iter= h_iter->next){ // find e's prev
		if(h_iter->next == h1){
			h1_old_prev = h_iter;
			break;
		}
	}
	for(HalfedgeRef h_iter = h2->next ; h_iter !=h2; h_iter= h_iter->next){ // find e's prev
		if(h_iter->next == h2){
			h2_old_prev = h_iter;
			break;
		}
	}
	// reassign vertices
	v1_old->halfedge = h1_new_prev;
	v2_old->halfedge = h2_new_prev;
	h1->vertex = v1_new;
	h2->vertex = v2_new;

	// reconnect halfedges
	h1_old_prev->next = h1_new_prev;
	h2_old_prev->next = h2_new_prev;

	h1_new_prev->next = h1;
	h2_new_prev->next = h2;

	h1->next = h1_new_next;
	h2->next = h2_new_next;

	// reassign face-halfedge iterator
	h1_new_prev->face = h1->face;
	h2_new_prev->face = h2->face;
	h1->face->halfedge = h1;
	h2->face->halfedge = h2;


	return h1->edge;
    // return std::nullopt;
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex

    return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	
    return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge

	/*
	break collapsing into micro steps, and assume valid operation
	For the edge deleted,
		if some faces loses one edge
			if the face originally has 3 edges
				remove the face
				remove unused inner halfedges, plus outter if on boundary
				if vertex & edge not used, remove them
			else:
				remove the edge, update halfedges
				remove the halfedge
	For the two vertex collapsed,
		for affected faces
			collapse edge, Update halfedges
			collapse vertex
	*/
	std::cout << std::endl << "Debug: before" << describe() << std::endl;
	// construct references
	HalfedgeRef h1 = e->halfedge;
	HalfedgeRef h2 = h1->twin;
	VertexRef v1 = h1->vertex;
	VertexRef v2 = h2->vertex;
	std::unordered_map<uint32_t, HalfedgeRef> h_map; // that connected to v1 or v2, except h1 and h2

	for (VertexRef v : {v1, v2}){
		HalfedgeRef h_iter = v->halfedge;
		do{
			h_map[h_iter->id] = h_iter;
			h_iter = h_iter->twin->next;
		}
			while(h_iter != v->halfedge);
	}

	// declare micro steps
		
	auto find_prev_halfedge = [&](HalfedgeRef h){
		HalfedgeRef iter = h->next;
		while(iter != h){
			if(iter->next == h){
				return iter;
			}
			iter = iter->next;
		}
		return h;
	};

	auto remove_edge_both_he = [&](EdgeRef e){
		HalfedgeRef h1 = e->halfedge;
		HalfedgeRef h2 = h1->twin;
		h_map.erase(h1->id);
		h_map.erase(h2->id);
		erase_edge(e);
		erase_halfedge(h1);
		erase_halfedge(h2);
	};

	auto collapse_edge_connected_face = [&](HalfedgeRef h){
		FaceRef f = h->face;
		if(f->degree() == 3){   
			// remove faces, unused halfedges, vertex, edge, leave a valid mesh + e
			// re-assign outer twins
			HalfedgeRef h_prev = find_prev_halfedge(h);
			HalfedgeRef h_next = h->next;
			// if both sides are boundary, their edges will be deleted right away
			if(h_prev->edge->on_boundary() && h_next->edge->on_boundary()){
				HalfedgeRef h_prev_t_prev = find_prev_halfedge(h_prev->twin);
				HalfedgeRef h_next_t_next = h_next->twin->next;
				h_prev_t_prev->next = h;
				h->next = h_next_t_next;
				h->face = h_prev_t_prev->face;

				remove_edge_both_he(h_prev->edge);
				remove_edge_both_he(h_next->edge);
				erase_vertex(h_prev->vertex);
			}
			// if neither sides is boundary, reassign outer twins, then remove inner halfedges only
			else if(!h_prev->edge->on_boundary() && !h_next->edge->on_boundary()){
				EdgeRef e_prev = h_prev->edge;
				EdgeRef e_next = h_next->edge;
				h_prev->twin->twin = h_next->twin;
				h_next->twin->twin = h_prev->twin;
				h_next->twin->edge = e_prev;
				// re-assign vertex, edge, face
				h_prev->vertex->halfedge = h_prev->twin->next;
				e_prev->halfedge = h_prev->twin;
				h->twin->face->halfedge = h->twin->next;

				h_map.erase(h_prev->id);
				h_map.erase(h_next->id);
				erase_halfedge(h_prev);
				erase_halfedge(h_next);
				erase_edge(e_next);
				if(h->edge->on_boundary()){
					HalfedgeRef h_twin_prev = find_prev_halfedge(h->twin);
					HalfedgeRef h_twin_next = h->twin->next;
					h_twin_prev->next = h_twin_next;
				}
			}else{ // if one side is boundary, clamp to the other side's halfedge, then delete
				HalfedgeRef is_bound = h_prev;
				HalfedgeRef not_bound = h_next;
				if(h_next->edge->on_boundary()){
					is_bound = h_next;
					not_bound = h_prev;
				}
				// reaassign twin, edge, vertex of the boundary outer halfedge
				HalfedgeRef bound_twin = is_bound->twin;
				bound_twin->twin = not_bound->twin;
				bound_twin->edge = not_bound->edge;
				bound_twin->vertex = not_bound->vertex;
				not_bound->twin = bound_twin;
				if(h->edge->on_boundary()){
					if(bound_twin->next == h->twin){
						bound_twin->next = h->twin->next;
					}else{
						HalfedgeRef h_twin_prev = find_prev_halfedge(h->twin);
						h_twin_prev->next = bound_twin;
					}
					
				}
				// delete two inner halfedges and the un-used edge
				h_map.erase(is_bound->id);
				h_map.erase(not_bound->id);
				erase_halfedge(is_bound);
				erase_halfedge(not_bound);
				erase_edge(is_bound->edge);
			}
			
			// collapsed edge will not be deleted here, in case deleted twice
			erase_face(f);


		}else{
			// Reconnect inner adjacent halfedges
			HalfedgeRef h_prev = find_prev_halfedge(h);
			HalfedgeRef h_next = h->next;
			h_prev->next = h_next;
			// reassign face in case face was linked to next-deleted h
			h->face->halfedge = h_next;
			// delete halfedge, and the other half if needed
			if(h->edge->on_boundary()){
				h_prev = find_prev_halfedge(h->twin);
				h_next = h->twin->next;
				h_prev->next = h_next;
			}
			// collapsed edge will not be deleted here, in case deleted twice
		}
	};



	// execute the micro steps
	collapse_edge_connected_face(h1);	
	if(!e->on_boundary()){
		collapse_edge_connected_face(h2);
	}
	// collapse vertex
	VertexRef v_mid = emplace_vertex();
	v_mid->position = (v1->position + v2->position)/2;

	// delete unused
	h_map.erase(h1->id);
	h_map.erase(h2->id);
	erase_halfedge(h1);
	erase_halfedge(h2);
	erase_edge(e);
	
	// ready to migrate halfedges from two vertices to middle, delete duplicated edges if needed to
	// find edge, then halfedges, excluding: e, h1, h2, same_vertex_end_point 
	for (const auto& pair : h_map) {
		HalfedgeRef h_iter = pair.second;
		h_iter->vertex = v_mid;
		v_mid->halfedge = h_iter;
    }

		
	
	erase_vertex(v1);
	erase_vertex(v2);


	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)
	
	std::cout << "Debug: after" << describe() << std::endl;
    return v_mid;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

    return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper

	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink,
	// offset by move
	
}


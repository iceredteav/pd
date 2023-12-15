#include <igl/opengl/glfw/Viewer.h>
#include <igl/AABB.h>
#include <igl/in_element.h>
#include <igl/signed_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/Timer.h>


#include <iostream>
#include <algorithm>

Eigen::MatrixXd V;
Eigen::MatrixXi F;


double mesh_vertices[] = {
	-4.00,4.00,0.00,-3.00,4.00,0.00,-2.00,4.00,0.00,-1.00,4.00,0.00,0.00,4.00,0.00,
	1.00,4.00,0.00,2.00,4.00,0.00,3.00,4.00,0.00,4.00,4.00,0.00,
	-4.00,3.00,0.00,-3.00,3.00,0.00,-2.00,3.00,0.00,-1.00,3.00,0.00,0.00,3.00,0.00,
	1.00,3.00,0.00,2.00,3.00,0.00,3.00,3.00,0.00,4.00,3.00,0.00,
	-4.00,2.00,0.00,-3.00,2.00,0.00,-2.00,2.00,0.00,-1.00,2.00,0.00,0.00,2.00,0.00,
	1.00,2.00,0.00,2.00,2.00,0.00,3.00,2.00,0.00,4.00,2.00,0.00,
	-4.00,1.00,0.00,-3.00,1.00,0.00,-2.00,1.00,0.00,-1.00,1.00,0.00,0.00,1.00,0.00,
	1.00,1.00,0.00,2.00,1.00,0.00,3.00,1.00,0.00,4.00,1.00,0.00,
	-4.00,0.00,0.00,-3.00,0.00,0.00,-2.00,0.00,0.00,-1.00,0.00,0.00,0.00,0.00,0.00,
	1.00,0.00,0.00,2.00,0.00,0.00,3.00,0.00,0.00,4.00,0.00,0.00,
	-4.00,-1.00,0.00,-3.00,-1.00,0.00,-2.00,-1.00,0.00,-1.00,-1.00,0.00,0.00,-1.00,0.00,
	1.00,-1.00,0.00,2.00,-1.00,0.00,3.00,-1.00,0.00,4.00,-1.00,0.00,
	- 4.00,-2.00,0.00,-3.00,-2.00,0.00,-2.00,-2.00,0.00,-1.00,-2.00,0.00,0.00,-2.00,0.00,
	1.00,-2.00,0.00,2.00,-2.00,0.00,3.00,-2.00,0.00,4.00,-2.00,0.00,
	-4.00,-3.00,0.00,-3.00,-3.00,0.00,-2.00,-3.00,0.00,-1.00,-3.00,0.00,0.00,-3.00,0.00,
	1.00,-3.00,0.00,2.00,-3.00,0.00,3.00,-3.00,0.00,4.00,-3.00,0.00,
	-4.00,-4.00,0.00,-3.00,-4.00,0.00,-2.00,-4.00,0.00,-1.00,-4.00,0.00,0.00,-4.00,0.00,
	1.00,-4.00,0.00,2.00,-4.00,0.00,3.00,-4.00,0.00,4.00,-4.00,0.00

	
};
int mesh_faceTriIds[] = {
	0,1,10,0,10,9,1,2,11,1,11,10,2,3,11,3,12,11,3,4,12,4,13,12,4,5,14,4,14,13,5,6,14,6,15,14,6,7,15,7,16,15,7,8,16,8,17,16,
	9,10,18,10,19,18,10,11,19,11,20,19,11,12,21,11,21,20,12,13,22,12,22,21,13,14,23,13,23,22,14,15,24,14,24,23,15,16,25,15,25,24,16,17,26,16,26,25,
	18,19,28,18,28,27,19,20,28,20,29,28,20,21,30,20,30,29,21,22,30,22,31,30,22,23,32,22,32,31,23,24,32,24,33,32,24,25,33,25,34,33,25,26,35,25,35,34,
	27,28,36,28,37,36,28,29,37,29,38,37,29,30,39,29,39,38,30,31,39,31,40,39,31,32,40,32,41,40,32,33,42,32,42,41,33,34,43,33,43,42,34,35,43,35,44,43,
	36,37,46,36,46,45,37,38,46,38,47,46,38,39,48,38,48,47,39,40,48,40,49,48,40,41,50,40,50,49,41,42,51,41,51,50,42,43,52,42,52,51,43,44,52,44,53,52,
	45,46,55,45,55,54,46,47,56,46,56,55,47,48,56,48,57,56,48,49,58,48,58,57,49,50,59,49,59,58,50,51,59,51,60,59,51,52,61,51,61,60,52,53,62,52,62,61,
	54,55,64,54,64,63,55,56,64,56,65,64,56,57,66,56,66,65,57,58,66,58,67,66,58,59,67,59,68,67,59,60,69,59,69,68,60,61,70,60,70,69,61,62,71,61,71,70,
	63,64,73,63,73,72,64,65,74,64,74,73,65,66,75,65,75,74,66,67,75,67,76,75,67,68,76,68,77,76,68,69,78,68,78,77,69,70,78,70,79,78,70,71,79,71,80,79
};


void init_mesh() {
	int num_vertices = sizeof(mesh_vertices) / sizeof(mesh_vertices[0]) / 3;
	int num_tris = sizeof(mesh_faceTriIds) / sizeof(mesh_faceTriIds[0]) / 3;
	V.resize(num_vertices, 3);
	F.resize(num_tris, 3);
	for (int i = 0; i < num_vertices;i++) {
		for (int j = 0;j < 3;j++) {
			V(i, j) = mesh_vertices[3 * i + j ];
		}
	}

	for (int i = 0; i < num_tris;i++) {
		for (int j = 0; j < 3;j++) {
			F(i, j) = mesh_faceTriIds[3 * i + j];
		}
	}
}

class Cloth {
	private:
		void sort_vec(const Eigen::MatrixXi& mtrx, Eigen::MatrixXi& sorted_mtrx, Eigen::VectorXi& ind) {
			ind = Eigen::VectorXi::LinSpaced(mtrx.rows(), 0, mtrx.rows() - 1);
			auto rule = [mtrx](int i, int j)->bool
			{
				return mtrx(i, 2) < mtrx(j, 2);
			};
			std::sort(ind.data(), ind.data() + ind.size(), rule);
			//data成员函数返回VectorXd的第一个元素的指针，类似于begin()
			sorted_mtrx.resize(mtrx.rows(), mtrx.cols());
			for (int i = 0;i < mtrx.rows();i++) {
				sorted_mtrx.row(i) = mtrx.row(ind(i));
			}
		}
		struct Edge {
			int id0;
			int id1;
			int edgeNr;
		};

		Eigen::MatrixXi getEdges(int triIds[],int numTris) {
			// create common edges
			std::vector<Edge> edges;
			Eigen::MatrixXi sorted_edges;
			//int numTris = sizeof(triIds) / sizeof(triIds[0]) / 3;

			for (int i = 0; i < numTris; i++) {
				for (int j = 0; j < 3; j++) {
					int id0 = triIds[3 * i + j];
					int id1 = triIds[3 * i + (j + 1) % 3];
					edges.push_back({ std::min(id0, id1), std::max(id0, id1), 3 * i + j });
				}
			}
			//get sorted edges
			// sort so common edges are next to each other
			std::sort(edges.begin(), edges.end(),
				[](const Edge& a, const Edge& b) {
					return (a.id0 < b.id0) || (a.id0 == b.id0 && a.id1 <  b.id1);
				});

			// find matching edges
			std::vector<int> neighbors(3 * numTris, -1);

			int nr = 0;
			while (nr < edges.size()) {
				Edge e0 = edges[nr];
				nr++;
				if (nr < edges.size()) {
					Edge e1 = edges[nr];
					if (e0.id0 == e1.id0 && e0.id1 == e1.id1) {
						neighbors[e0.edgeNr] = e1.edgeNr;
						//neighbors[e1.edgeNr] = e0.edgeNr;
					}
				}
			}
			//delete repeated edges
			int k = 0;
			sorted_edges.resize(edges.size(),2);
			for (int i = 0;i < edges.size();i++) {
				int id0 = edges[i].id0;
				int id1 = edges[i].id1;
				int n = neighbors[edges[i].edgeNr];
				if (n < 0) {
					sorted_edges(k, 0) = id0;
					sorted_edges(k, 1) = id1;
					k++;
				}
			}
			num_edges = k;

			return sorted_edges;
		}

	public:
		Eigen::MatrixXd pos_; //position N*3
		Eigen::MatrixXd old_pos_;//N*3
		Eigen::MatrixXd vel_; //velocity N*3
		Eigen::MatrixXd w_;//M N*N 
		Eigen::MatrixXd inv_w_;//M-1 N*N
		Eigen::MatrixXd gravity;//N*3
		Eigen::MatrixXi faces_;
		Eigen::MatrixXi edges_;//M*2
		Eigen::VectorXd rest_length_;//M*1
		Eigen::MatrixXd b;
		Eigen::MatrixXd sn;
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> cholesky_decomposition_;

		int num_vertices;//N
		int num_edges;//M
		int num_tris;
		double wi;//stiffness?
		int grabId = -1;

		Cloth() {
			init_mesh();
			//std::cout << V.rows() << std::endl;
			//std::cout << F.rows() << std::endl;
			//init pos_ faces_
			pos_ = V;
			faces_ = F;
			num_vertices = pos_.rows();
			num_tris = faces_.rows();
			/*std::cout << pos_;*/
			//get edges_,rest_length_ for 
			edges_ = getEdges(mesh_faceTriIds, num_tris);
			//std::cout << edges_ << std::endl;
			
			init_physical_data();

		}
		void init_physical_data() {
			old_pos_ = pos_;
			vel_ = Eigen::MatrixXd::Zero(num_vertices, 3);
			w_ = Eigen::MatrixXd::Zero(num_vertices, num_vertices);
			inv_w_= Eigen::MatrixXd::Zero(num_vertices, num_vertices);
			for (int i = 0;i < num_vertices;i++) {
				w_(i, i) = 1;
				inv_w_(i, i) = 1;
			}
			inv_w_(0,0) = 0;
			inv_w_(8,8) = 0;
			w_(0, 0) = 2 ^ 30;
			w_(8, 8) = 2 ^ 30;
			//std::cout << pos_.row(0) << std::endl;
			//std::cout << pos_.row(20) << std::endl;
			wi = 0.5;
			gravity = Eigen::MatrixXd::Zero(num_vertices, 3);
			for (int i = 0;i < num_vertices;i++) {
				gravity(i, 1) = -9.8;
			}
			b = Eigen::MatrixXd::Zero(num_vertices, 3);
		}

		void init_pd(double dt) {
			init_constraints();
			Eigen::SparseMatrix<double> A (num_vertices, num_vertices);
			//A=M/h^2+for(constraint:constraints)wi_(AiSi)T_AiSi
			A = stretching_constraint_get_wi_SiT_AiT_Ai_Si();
			for (int i = 0; i < num_vertices;i++) {
				A.coeffRef(i, i) += w_(i, i) / dt / dt;
			}
			//A.insert(1, 1) = 1;
			cholesky_decomposition_.compute(A);
		}

		void init_constraints() {
			rest_length_.resize(num_edges);
			for (int i = 0;i < num_edges;i++) {
				int id0 = edges_(i, 0);
				int id1 = edges_(i, 1);
				Eigen::Vector3d const p1 = pos_.row(id0);
				Eigen::Vector3d const p2 = pos_.row(id1);
				rest_length_(i) = (p1 - p2).norm();
			}
		}

		Eigen::SparseMatrix<double> stretching_constraint_get_wi_SiT_AiT_Ai_Si() {
			//return N*N matrix
			//Ai=Bi=M^1/2
			//nonzero in constraint vertices Si
			Eigen::SparseMatrix<double> res(num_vertices, num_vertices);
			for (int i = 0;i < num_edges;i++) {
				int id0 = edges_(i, 0);
				int id1 = edges_(i, 1);
				res.coeffRef(id0,id0) += wi * w_(id0,id0);
				res.coeffRef(id1,id1) += wi * w_(id1,id1);
			}
			return res;

		}

		void PD_presolve(double dt) {
			/*std::cout << "pos[0]:";
			std::cout << pos_.row(0) << std::endl;
			std::cout << "pos[8]:";
			std::cout << pos_.row(8) << std::endl;
			std::cout << "vel[0]:";
			std::cout << vel_.row(0) << std::endl;
			std::cout << "vel[8]:";
			std::cout << vel_.row(8) << std::endl;
			std::cout << "inv_w_*g[0]:";
			std::cout << (inv_w_*gravity).row(0) << std::endl;
			std::cout << "inv_w_*g[8]:";
			std::cout << (inv_w_ * gravity).row(8) << std::endl;*/
			sn = pos_ + dt * vel_ + dt * dt * inv_w_ * gravity;//N*3
			pos_ = sn;
			/*std::cout << "sn[0]:";
			std::cout << sn.row(0) << std::endl;
			std::cout << "sn[8]:";
			std::cout << sn.row(8) << std::endl;*/
		}

		void PBD_presolve(double dt) {
			old_pos_ = pos_;
			for (int i = 0;i < num_vertices;i++) {
				if (inv_w_(i, i) != 0) {
					vel_.row(i) += gravity.row(i) * dt;
					pos_.row(i) += vel_.row(i) * dt;
				}
			}

		}

		void PBD_solve() {
			PBD_stretchingsolve();
		}

		void PBD_stretchingsolve() {
			for (int i = 0;i < num_edges;i++) {
				int id1 = edges_(i, 0);
				int id2 = edges_(i, 1);
				Eigen::Vector3d  q1 = pos_.row(id1);
				Eigen::Vector3d  q2 = pos_.row(id2);
				double res_length = rest_length_(i);
				double length = (q1 - q2).norm();
				//if (id1 == 0 && id2 == 1) {
				//	std::cout << "point12_length:" << std::endl;
				//	std::cout << length << std::endl;
				//	std::cout << q1 << std::endl;
				//	std::cout << q2 << std::endl;
				//}
				Eigen::Vector3d  n = (q1 - q2) / length;
				double lambda1 = inv_w_(id1, id1) / (inv_w_(id1, id1) + inv_w_(id2, id2));
				double lambda2 = inv_w_(id2, id2) / (inv_w_(id1, id1) + inv_w_(id2, id2));
				pos_.row(id1) -= lambda1 * (length - res_length) * n;//mass相等
				pos_.row(id2) += lambda2 * (length - res_length) * n;
			}
		}

		void PBD_postsolve(double dt) {
			vel_ = (pos_ - old_pos_) / dt;
		}


		void PD_globalsolve() {
			//resize
			//Apos_=b; A(N*N) pos(N*3) b(N*3)
			pos_.col(0) = cholesky_decomposition_.solve(b.col(0));
			pos_.col(1) = cholesky_decomposition_.solve(b.col(1));
			pos_.col(2) = cholesky_decomposition_.solve(b.col(2));
			std::cout << "pos_.row(1)" << std::endl;
			std::cout << pos_.row(1)<<std::endl;
		}

		void PD_localsolve() {
			//project constraints
			//for(constrain:constraints)
			PD_stretchingsolve();
			//bendingsolve();

		}

		void PD_stretchingsolve() {
			//Ai=Bi=M^1/2
			//project_wi_SiT_AiT_Bi_pi
			for (int i = 0;i < num_edges;i++) {
				int id1 = edges_(i, 0);
				int id2 = edges_(i, 1);
				Eigen::Vector3d  q1 = pos_.row(id1);
				Eigen::Vector3d  q2 = pos_.row(id2);
				double res_length = rest_length_(i);
				double length = (q1 - q2).norm();
				//if (id1 == 0 && id2 == 1) {
				//	std::cout << "point12_length:" << std::endl;
				//	std::cout << length << std::endl;
				//	std::cout << q1 << std::endl;
				//	std::cout << q2 << std::endl;
				//}
				Eigen::Vector3d  n = (q1 - q2) / length;
				double lambda1 = inv_w_(id1,id1) / (inv_w_(id1,id1) + inv_w_(id2,id2));
				double lambda2 = inv_w_(id2,id2) / (inv_w_(id1,id1) + inv_w_(id2,id2));
				Eigen::Vector3d  p1 = q1 - lambda1 * (length - res_length) * n;//mass相等
				Eigen::Vector3d  p2 = q2 + lambda2 * (length - res_length) * n;
				//if (id1 == 0 && id2 == 1) {
				//	std::cout << "point12_projective_length:" << std::endl;
				//	std::cout << (p1-p2).norm() << std::endl;
				//	std::cout << p1 << std::endl;
				//	std::cout << p2 << std::endl;
				//}
				b(id1, 0) += wi * w_(id1,id1) * p1.x();
				b(id1, 1) += wi * w_(id1,id1) * p1.y();
				b(id1, 2) += wi * w_(id1,id1) * p1.z();
				b(id2, 0) += wi * w_(id2,id2) * p2.x();
				b(id2, 1) += wi * w_(id2,id2) * p2.y();
				b(id2, 2) += wi * w_(id2,id2) * p2.z();

			}
		}


		void PD_postsolve(double dt) {
			for (int i = 0; i < num_vertices; i++)
			{
				if (inv_w_(i,i) != 0.0f)
				{
					vel_.row(i) = (pos_.row(i) - old_pos_.row(i)) / dt;
				}
			}
			old_pos_ = pos_;
		}

		void update(double dt, int maxIte) {
			//PD
			////step	
			//PD_presolve(dt);
			////std::cout << pos_.row(1) << std::endl;
			//b.resize(num_vertices, 3);
			//for (int ite = 0;ite < maxIte;ite++) {
			//	b.setZero();
			//	PD_localsolve();
			//	b += w_ * sn / dt / dt; //N*N N*3
			//	//std::cout << b.row(1) << std::endl;
			//	PD_globalsolve();
			//	//std::cout << "============" << std::endl;
			//}
			//PD_postsolve(dt);

			//PBD
			PBD_presolve(dt);
			for (int i = 0;i < maxIte;i++) {
				PBD_solve();
			}
			PBD_postsolve(dt);

		}

		void startGrab(Eigen::Vector3d mouse_pos,int faceId) {
			//get grabId
			int id0 = faces_(faceId, 0);
			int id1 = faces_(faceId, 1);
			int id2 = faces_(faceId, 2);
			int minId = id0;
			Eigen::Vector3d id0_pos = pos_.row(id0);
			Eigen::Vector3d id1_pos = pos_.row(id1);
			Eigen::Vector3d id2_pos = pos_.row(id2);
			double minDis = (id0_pos- mouse_pos).norm();
			if ((id1_pos - mouse_pos).norm() < minDis) {
				minDis = (id1_pos - mouse_pos).norm();
				minId = id1;
			}
			if ((id2_pos - mouse_pos).norm() < minDis) {
				minDis = (id2_pos - mouse_pos).norm();
				minId = id2;
			}
			grabId = minId;;
		}

		void moveGrabbed(Eigen::Vector3d pos) {
			if (grabId >= 0) {
				pos_(grabId,0) = pos.x();
				pos_(grabId,1) = pos.y();
				pos_(grabId,2) = pos.z();
			}
		}

		void endGrab(Eigen::Vector3d vel) {
			if (grabId >= 0) {
				vel_(grabId, 0) = vel.x();
				vel_(grabId, 1) = vel.y();
				vel_(grabId, 2) = vel.z();

			}
		}
};

Cloth cloth;
Eigen::Vector3d prePos;
Eigen::Vector3d vel;
//double time;
bool gMouseDown;
igl::Timer timer;

bool mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier) {
	gMouseDown = true;
	int fid;
	Eigen::Vector3f bc;
	std::cout << "mouse_down" << std::endl;
	// Cast a ray in the view direction starting from the mouse position
	float x = viewer.current_mouse_x;
	float y = viewer.core().viewport(3) - viewer.current_mouse_y;
	float z = viewer.down_mouse_z;
	Eigen::Matrix<float,3,1> mouse_screen_pos;
	//mouse_screen_pos.resize(3, 1);
	mouse_screen_pos(0,0) = x;
	mouse_screen_pos(1,0) = y;
	mouse_screen_pos(2,0) = z;
	Eigen::Matrix<float,3,1> mouse_world_pos;
	//mouse_world_pos.resize(3, 1);
	//double objx, objy, objz;//获得的世界坐标值
	std::cout << "proj matrix "<< std::endl;
	std::cout << viewer.core().proj << std::endl;
	std::cout << "viewerport matrix " << std::endl;
	std::cout << viewer.core().viewport << std::endl;

	mouse_world_pos = mouse_world_pos = igl::unproject(mouse_screen_pos, viewer.core().view,viewer.core().proj, viewer.core().viewport);
	//gluUnProject((GLdouble)x, (GLdouble)y, (GLdouble)z, modelview, projection, viewport, &prePos(0), &prePos(1), &prePos(2));
	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
		viewer.core().proj, viewer.core().viewport, cloth.pos_, cloth.faces_, fid, bc))
	{
		// paint hit red
		//fid -> picked faces
		//prePos = bc;
		prePos(0) = mouse_world_pos(0,0);
		prePos(1) = mouse_world_pos(1,0);
		prePos(2) = mouse_world_pos(2,0);
		std::cout << "mouse_pos:";
		std::cout << prePos << std::endl;
		cloth.startGrab(prePos, fid);
		vel = Eigen::Vector3d::Zero();
		timer.start();

		return true;
	}
	return false;
}


bool mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier) {
	cloth.endGrab(vel);
	timer.stop();
	gMouseDown = false;
	std::cout << "mouse_up" << std::endl;

	return false;
}



bool mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)
{
	if (gMouseDown) {
		std::cout << "mouse_move" << std::endl;
		Eigen::Vector3d mouse_pos;
		mouse_pos(0) = viewer.down_mouse_x;
		mouse_pos(1) = viewer.down_mouse_y;
		mouse_pos(2) = viewer.down_mouse_z;
		if (timer.getElapsedTime() > 0) {
			vel = (mouse_pos - prePos) / timer.getElapsedTime();
		}
		else {
			vel = Eigen::Vector3d::Zero();
		}

		prePos = mouse_pos;
		timer.stop();
		timer.start();

		cloth.moveGrabbed(prePos);


	}

	return false;
}


double dt = 0.1;
int maxIte = 5;
int frame = 0;
bool pause = true;

//Grabber grabber;



bool pre_draw(igl::opengl::glfw::Viewer& viewer)
{
	if (!pause)
	{
		frame++;
		// update_sphere(Vec3d(0.0, height, 0.0));

		// for (int i = 0; i < pull_points.size(); i++)
		// softbody.pos_.row(pull_points[i]) += Vec3d(0.0, height, 0.0);

		cloth.update(dt,maxIte);
		//std::cout << cloth.pos_.row(0) << std::endl;
		//std::cout << cloth.pos_.row(8) << std::endl;
	}
	// softbody.write_obj("../model/liver2/f" +std::to_string(frame++) + ".obj");

	viewer.data().clear();
	//std::cout << cloth.pos_ << std::endl;
	viewer.data().set_mesh(cloth.pos_, cloth.faces_);
	//viewer.core().align_camera_center(cloth.pos_, cloth.faces_);
	/*viewer.append_mesh();
	viewer.data().set_mesh(V, F);*/
	// viewer.data_list[0].set_colors(Eigen::RowVector3d(1, 0, 0));
	// viewer.data_list[1].set_colors(Eigen::RowVector3d(0, 1, 0));
	return false;
}


bool post_draw(igl::opengl::glfw::Viewer& viewer)
{
	for (auto& data : viewer.data_list)
	{
		data.clear();
	}
	return false;
}

bool key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
{
	if (key == 'p')
	{
		pause = !pause;
		//std::cout << std::boolalpha << pause << std::endl;
	}

	return false;
}







int main()
{	
	cloth.init_constraints();
	//cloth.init_pd(dt);
	//std::cout << cloth.pos_ << std::endl;
    igl::opengl::glfw::Viewer viewer;
	/*cloth.update(dt,maxIte);
	std::cout << cloth.pos_.row(0) << std::endl;
	std::cout << cloth.pos_.row(8) << std::endl;
    viewer.data().set_mesh(cloth.pos_, cloth.faces_);
    viewer.data().set_face_based(true);
    viewer.launch();*/
	viewer.callback_pre_draw = &pre_draw;
	viewer.callback_post_draw = &post_draw;
	viewer.callback_key_pressed = &key_pressed;
	viewer.callback_mouse_down = &mouse_down;
	viewer.callback_mouse_move = &mouse_move;
	viewer.callback_mouse_up = &mouse_up;
	viewer.core().is_animating = true;
	viewer.data().set_face_based(false);
	viewer.launch();

	return 0;
}
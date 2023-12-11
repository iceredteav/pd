#include <igl/opengl/glfw/Viewer.h>
#include <igl/AABB.h>
#include <igl/in_element.h>
#include <igl/signed_distance.h>
#include <igl/barycentric_coordinates.h>

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
	0,1,10,0,9,10,1,2,11,1,10,11,2,3,11,3,11,12,3,4,12,4,12,13,4,5,14,4,13,14,5,6,14,6,14,15,6,7,15,7,15,16,7,8,16,8,16,17,
	9,10,18,10,18,19,10,11,19,11,19,20,11,12,21,11,20,21,12,13,22,12,21,22,13,14,23,13,22,23,14,15,24,14,23,24,15,16,25,15,24,25,16,17,26,16,25,26,
	18,19,28,18,27,28,19,20,28,20,28,29,20,21,30,20,29,30,21,22,30,22,30,31,22,23,32,22,31,32,23,24,32,24,32,33,24,25,33,25,33,34,25,26,35,25,34,35,
	27,28,36,28,36,37,28,29,37,29,37,38,29,30,39,29,38,39,30,31,39,31,39,40,31,32,40,32,40,41,32,33,42,32,41,42,33,34,43,33,42,43,34,35,43,35,43,44,
	36,37,46,36,45,46,37,38,46,38,46,47,38,39,48,38,47,48,39,40,48,40,48,49,40,41,50,40,49,50,41,42,51,41,50,51,42,43,52,42,51,52,43,44,52,44,52,53,
	45,46,55,45,54,55,46,47,56,46,55,56,47,48,56,48,56,57,48,49,58,48,57,58,49,50,59,49,58,59,50,51,59,51,59,60,51,52,61,51,60,61,52,53,62,52,61,62,
	54,55,64,54,63,64,55,56,64,56,64,65,56,57,66,56,65,66,57,58,66,58,66,67,58,59,67,59,67,68,59,60,69,59,68,69,60,61,70,60,69,70,61,62,71,61,70,71,
	63,64,73,63,72,73,64,65,74,64,73,74,65,66,75,65,74,75,66,67,75,67,75,76,67,68,76,68,76,77,68,69,78,68,77,78,69,70,78,70,78,79,70,71,79,71,79,80
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
			rest_length_.resize(num_edges);
			for (int i = 0;i < num_edges;i++) {
				int id0 = edges_(i, 0);
				int id1 = edges_(i, 1);
				Eigen::Vector3d const p1 = pos_.row(id0);
				Eigen::Vector3d const p2 = pos_.row(id1);
				rest_length_(i) = (p1 - p2).norm();
			}
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
			Eigen::SparseMatrix<double> A (num_vertices, num_vertices);
			//A=M/h^2+for(constraint:constraints)wi_(AiSi)T_AiSi
			A = stretching_constraint_get_wi_SiT_AiT_Ai_Si();
			for (int i = 0; i < num_vertices;i++) {
				A.coeffRef(i, i) += w_(i, i) / dt / dt;
			}
			//A.insert(1, 1) = 1;
			cholesky_decomposition_.compute(A);
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

		void presolve(double dt) {
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

		void globalsolve() {
			//resize
			//Apos_=b; A(N*N) pos(N*3) b(N*3)
			pos_.col(0) = cholesky_decomposition_.solve(b.col(0));
			pos_.col(1) = cholesky_decomposition_.solve(b.col(1));
			pos_.col(2) = cholesky_decomposition_.solve(b.col(2));
			std::cout << "pos_.row(1)" << std::endl;
			std::cout << pos_.row(1)<<std::endl;
		}

		void localsolve() {
			//project constraints
			//for(constrain:constraints)
			stretchingsolve();
			//bendingsolve();

		}

		void stretchingsolve() {
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


		void postsolve(double dt) {
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
			//step	
			presolve(dt);
			//std::cout << pos_.row(1) << std::endl;
			b.resize(num_vertices, 3);
			for (int ite = 0;ite < maxIte;ite++) {
				b.setZero();
				localsolve();
				b += w_ * sn / dt / dt; //N*N N*3
				//std::cout << b.row(1) << std::endl;
				globalsolve();
				//std::cout << "============" << std::endl;
			}
			postsolve(dt);
		}

};

double dt = 0.1;
int maxIte = 5;
int frame = 0;
bool pause = true;
Cloth cloth;

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
		std::cout << std::boolalpha << pause << std::endl;
	}

	return false;
}

int main()
{	

	cloth.init_pd(dt);
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
	viewer.core().is_animating = true;
	viewer.data().set_face_based(false);
	viewer.launch();

	return 0;
}
// #include <gtest/gtest.h>
#include "systems/controllers/c3/c3_controller.h"
#include "solvers/c3_miqp.h"
#include "solvers/c3_qp.h"
#include "solvers/lcs_factory.h"
#include "examples/pushbot/parameters/pushbot_c3_controller_params.h"
#include "examples/pushbot/parameters/pushbot_sim_params.h"
#include "examples/pushbot/parameters/pushbot_sim_scene_params.h"

namespace dairlib {

  using drake::systems::BasicVector;
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using solvers::C3;
  using solvers::C3MIQP;
  using solvers::LCS;
  using solvers::LCSFactory;

  int do_main(){

    // load parameters
      PushbotSimSceneParams scene_params =
	  drake::yaml::LoadYamlFile<PushbotSimSceneParams>("examples/pushbot/parameters/scene.yaml");
      C3Options c3_options = drake::yaml::LoadYamlFile<C3Options>("examples/pushbot/parameters/c3_options.yaml");
      drake::solvers::SolverOptions solver_options =
	  drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
	      FindResourceOrThrow("examples/pushbot/parameters/pushbot_c3_qp_settings.yaml"))
	      .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

      std::vector<Eigen::MatrixXd> G_ = std::vector<MatrixXd>(c3_options.N, c3_options.G);
      std::vector<Eigen::MatrixXd> U_ = std::vector<MatrixXd>(c3_options.N, c3_options.U);
      int N_ = c3_options.N;
      std::vector<MatrixXd> Q_;
      std::vector<MatrixXd> R_;

      double discount_factor = 1;
      for (int i = 0; i < N_; ++i) {
	Q_.push_back(discount_factor * c3_options.Q);
	R_.push_back(discount_factor * c3_options.R);
	discount_factor *= c3_options.gamma;
      }
      Q_.push_back(discount_factor * c3_options.Q);

      int n_x_ = 4;
      int n_u_ = 2;
      int n_lambda_ = 2;
      MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
      MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
      VectorXd d = VectorXd::Zero(n_x_);
      MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
      MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
      MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
      MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
      VectorXd c = VectorXd::Zero(n_lambda_);

      
      auto lcs_placeholder =  LCS(A, B, D, d, E, F, H, c, c3_options.N, c3_options.dt); //CreatePlaceholderLCS();
      auto x_desired_placeholder = std::vector<VectorXd>(N_ + 1, VectorXd::Zero(4));
      auto c3_ = std::make_unique<C3MIQP>(lcs_placeholder,
				     C3::CostMatrices(Q_, R_, G_, U_),
				     x_desired_placeholder, c3_options);
      c3_->SetOsqpSolverOptions(solver_options);

      auto x_des = std::vector<VectorXd>(N_ + 1, VectorXd::Zero(4));;
      drake::VectorX<double> x_lcs = Eigen::VectorXd::Zero(4);
      x_lcs[0] = 0.4;    // theta = about pi/8
      //x_lcs[2] = 0.52;   // dtheta = about pi/6

      // std::cout << x_lcs;
      std::cout << "testing" << std::endl;
      c3_->UpdateTarget(x_des);
      c3_->Solve(x_lcs);

      auto z_sol = c3_->GetFullSolution();

      std::cout << "Printing z_sol:\n";
      for (size_t i = 0; i < z_sol.size(); ++i) {
	std::cout << "Vector " << i << ": [ ";
	for (int j = 0; j < z_sol[i].size(); ++j) {
	    std::cout << z_sol[i](j) << " ";
	    }
	    std::cout << "]\n";
      }

      return 0;

  }
    
} // namespace dairlib

int main(){ return dairlib::do_main(); }

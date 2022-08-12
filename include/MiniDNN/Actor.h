#ifndef ACTOR_H_
#define ACTOR_H_
#include "MiniDNN.h"
using namespace MiniDNN;

  class MLP_Actor {
  public:
    MLP_Actor() {
      pi_depth = 3;
      n_hidden1 = 128;
      n_hidden2 = 128;
      n_output = 12;
      n_input = 38;

      acStd.setZero(n_output);
      acMean.setZero(n_output);
      obStd.setZero(n_input);
      obMean.setZero(n_input);
    }

    ~MLP_Actor() {};

    void deployNetwork(std::string path) {
      policy_file_path = path;

      loadNorm();
      loadWeights();
      buildArch();
    }

    /** 
     * Computes Denormalized Action with Raw States. Only computes col(0).
     * @param[in] state Raw state from environment. DONT NORMALIZE IT. Better be Matrix<double, obDim, 1>
     * @param[out] pred Denormalized Action Matrix.  number of cols will be same as state. */
    void getAction(Eigen::MatrixXd state, Eigen::MatrixXd &pred) {
      state.col(0) = (state.col(0) - obMean).cwiseQuotient(obStd);
      getActionRaw(state, pred);
      pred.col(0) = pred.col(0).cwiseProduct(acStd)+acMean;
    }

    /** 
     * Simply forwards the NN to get a normalized action. Deterministic Policy.
     * Only computes col(0).
     * @param[in] state Normalized state. Better be Matrix<double, obDim, 1>
     * @param[out] pred RAW Action Matrix. Needs normalization to be used. Number of cols will be same as state. */
    void getActionRaw(Eigen::MatrixXd state, Eigen::MatrixXd &pred) {
      pred.col(0) = pi.predict(state.col(0));
    }

    void getHidden(Eigen::MatrixXd state, Eigen::MatrixXd &hid1, Eigen::MatrixXd &hid2) {
      pi_h1->forward(state.col(0));
      hid1 = pi_h1->output();

      pi_h2->forward(hid1);
      hid2 = pi_h2->output();
    }

    Eigen::MatrixXd action_bound;
    Eigen::MatrixXd system_joint_bound;
    Eigen::MatrixXd network_bound;
    Eigen::MatrixXd vel_bound;
    Eigen::MatrixXd vel_max;
    Eigen::VectorXd acMean, acStd, obMean, obStd;
    Eigen::VectorXd jPGain, jDGain;

    std::string policy_file_path;
    int n_input, n_output;
    int n_hidden1, n_hidden2;

    static Eigen::MatrixXd readCSV(std::string file, int rows, int cols) {
      // std::cout << "Going to load csv with " << rows << " rows and " << cols << " columns." << std::endl;
      std::ifstream in;
      in.open(file.c_str());
      std::string line;
      int row = 0;
      int col;

      Eigen::MatrixXd res = Eigen::MatrixXd(rows, cols);
      if (in.is_open()) {
        while (std::getline(in, line)) {
          char *ptr = (char *) line.c_str();
          int len = line.length();
          col = 0;
          char *start = ptr;
          for (int i = 0; i < len; i++)
            if (ptr[i] == ',') {
              res(row, col++) = atof(start);
              start = ptr + i + 1;
            }
          // std::cout<<row<<", "<<col<<", "<< start<<std::endl;
          res(row, col) = atof(start);
          row++;
          if(row==rows) break;
        }
        in.close();
      }
      return res;
    }

  private:

    /** Load network parameters from CSV files
    * For fully connected layers,  output y = act(z), z = W'x + b
    * W is weight matrix shaped (input, output)
    * in CSV exported from Torch weights shapes (output, input) 
    * b is bias vector shaped (output, 1) **/
    void loadWeights() {
      // std::cout << "Going to load weights.\n";
      pi_b0.setZero(n_hidden1, 1);
      pi_b1.setZero(n_hidden2, 1);
      pi_b2.setZero(n_output, 1);
      pi_w0.setZero(n_input, n_hidden1);
      pi_w1.setZero(n_hidden1, n_hidden2);
      pi_w2.setZero(n_hidden2, n_output);

      std::cout << "Going to open CSVs.\n";

      pi_b0 = readCSV(policy_file_path + "/architecture.0.bias.csv", n_hidden1, 1);
      pi_b1 = readCSV(policy_file_path + "/architecture.2.bias.csv", n_hidden2, 1);
      pi_b2 = readCSV(policy_file_path + "/architecture.4.bias.csv", n_output, 1);
      pi_w0 = readCSV(policy_file_path + "/architecture.0.weight.csv", n_hidden1, n_input).transpose();
      pi_w1 = readCSV(policy_file_path + "/architecture.2.weight.csv", n_hidden2, n_hidden1).transpose();
      pi_w2 = readCSV(policy_file_path + "/architecture.4.weight.csv", n_output, n_hidden2).transpose();


//       std::cout<<"B0:("<<pi_b0.rows()<<", "<<pi_b0.cols()<<")\n";
//       std::cout<<"B1:("<<pi_b1.rows()<<", "<<pi_b1.cols()<<")\n";
//       std::cout<<"B2:("<<pi_b2.rows()<<", "<<pi_b2.cols()<<")\n";
       std::cout<<"W0:("<<pi_w0.rows()<<", "<<pi_w0.cols()<<")\n";
       std::cout<<"W1:("<<pi_w1.rows()<<", "<<pi_w1.cols()<<")\n";
       std::cout<<"W2:("<<pi_w2.rows()<<", "<<pi_w2.cols()<<")\n";
      std::cout << "B0:(" << pi_b0.rows() << ", " << pi_b0.cols() << ") = " << pi_b0.transpose() << std::endl;
      std::cout << "B1:(" << pi_b1.rows() << ", " << pi_b1.cols() << ") = " << pi_b1.transpose() << std::endl;
      std::cout << "B2:(" << pi_b2.rows() << ", " << pi_b2.cols() << ") = " << pi_b2.transpose() << std::endl;
//      std::cout << "W0:(" << pi_w0.rows() << ", " << pi_w0.cols() << ") = " << pi_w0.transpose() << std::endl;
//      std::cout << "W1:(" << pi_w1.rows() << ", " << pi_w1.cols() << ") = " << pi_w1.transpose() << std::endl;
//      std::cout << "W2:(" << pi_w2.rows() << ", " << pi_w2.cols() << ") = " << pi_w2.transpose() << std::endl;
    }

    void loadNorm() {
      std::cout << "Going to load norms.\n";
      obMean.setZero(n_input);
      obStd.setZero(n_input);
      acMean.setZero(n_output);
      acStd.setZero(n_output);

      obMean = readCSV(policy_file_path + "/obMean.csv", n_input, 1).col(0);
      std::cout << "obMean:" << obMean.transpose() << std::endl;
      obStd = readCSV(policy_file_path + "/obStd.csv", n_input, 1).col(0);
      std::cout << "obStd:" << obStd.transpose() << std::endl;
      acMean = readCSV(policy_file_path + "/acMean.csv", n_output, 1).col(0);
      std::cout << "acMean:" << acMean.transpose() << std::endl;
      acStd = readCSV(policy_file_path + "/acStd.csv", n_output, 1).col(0);

      for (int i = 0; i < n_input; i++) {
        if (fabs(obStd[i]) < 0.0001) {
          obStd[i] = 0.0001;
        }
      }
    }

    void buildArch() {
      std::vector <MiniDNN::Scalar> pi_hidden1_param(pi_w0.size() + pi_b0.size());
      std::vector <MiniDNN::Scalar> pi_hidden2_param(pi_w1.size() + pi_b1.size());
      std::vector <MiniDNN::Scalar> pi_output_param(pi_w2.size() + pi_b2.size());

      std::copy(pi_w0.data(), pi_w0.data() + pi_w0.size(), pi_hidden1_param.begin());
      std::copy(pi_b0.data(), pi_b0.data() + pi_b0.size(), pi_hidden1_param.begin() + pi_w0.size());

      std::copy(pi_w1.data(), pi_w1.data() + pi_w1.size(), pi_hidden2_param.begin());
      std::copy(pi_b1.data(), pi_b1.data() + pi_b1.size(), pi_hidden2_param.begin() + pi_w1.size());

      std::copy(pi_w2.data(), pi_w2.data() + pi_w2.size(), pi_output_param.begin());
      std::copy(pi_b2.data(), pi_b2.data() + pi_b2.size(), pi_output_param.begin() + pi_w2.size());

      std::vector <std::vector<MiniDNN::Scalar>> pi_param;
      pi_param.reserve(pi_depth);
      pi_param.push_back(pi_hidden1_param);
      pi_param.push_back(pi_hidden2_param);
      pi_param.push_back(pi_output_param);

      pi_h1 = new MiniDNN::FullyConnected<MiniDNN::LeakyReLU>(n_input, n_hidden1);
      pi_h2 = new MiniDNN::FullyConnected<MiniDNN::LeakyReLU>(n_hidden1, n_hidden2);
      pi_out = new MiniDNN::FullyConnected<MiniDNN::Identity>(n_hidden2, n_output);

      pi.add_layer(pi_h1);
      pi.add_layer(pi_h2);
      pi.add_layer(pi_out);

      pi.init(0, 0.01, 123);
      pi.set_parameters(pi_param);
    }

    Eigen::MatrixXd pi_w0, pi_w1, pi_w2;
    Eigen::MatrixXd pi_b0, pi_b1, pi_b2;
    MiniDNN::FullyConnected<MiniDNN::LeakyReLU> *pi_h1;
    MiniDNN::FullyConnected<MiniDNN::LeakyReLU> *pi_h2;
    MiniDNN::FullyConnected<MiniDNN::Identity> *pi_out;

    int pi_depth;
    MiniDNN::Network pi;
  };

#endif /* ACTOR_H_ */

#include "lm_library_pro_Huimin.h"

// PARAMETERS --------------------------------------------------------------------------------------------------------------
// Connection with MUX and sensors
uint8_t sensorPorts[NUM_SENSORS] = { 0, 1 };
Eigen::MatrixXf sensorData = Eigen::MatrixXf::Zero(NUM_SENSORS+1, 3);
Eigen::MatrixXf sensorMagneticField = Eigen::MatrixXf::Zero(NUM_SENSORS+1, 3);
Eigen::MatrixXf sensorField = Eigen::MatrixXf::Zero(NUM_SENSORS, 3);
Eigen::MatrixXf sPos = Eigen::MatrixXf::Zero(NUM_SENSORS, 3);
std::array<Eigen::MatrixXf, NUM_SENSORS + 1> Rsens;

// Ring parameters
Eigen::Matrix<float, 3, 1> d0{ { RING_RADIUS }, { 0 }, { 0 } };
Eigen::MatrixXf wRs1{ { 0, -cos(PI/6), -sin(PI/6) }, { 1, 0, 0 }, { 0, -sin(PI/6), cos(PI/6) } }; // Rotation matrix from World to sensor 0 frame
Eigen::MatrixXf wRs2{ { 0, -cos(PI/6),  sin(PI/6) }, { 1, 0, 0 }, { 0,  sin(PI/6), cos(PI/6) } };
Eigen::MatrixXf wRs3{ { 0, 1, 0 }, { -1, 0, 0 }, { 0, 0, 1 } };

// Magnet(s) parameter(s)
float K = ((float)M_COEFF) / 1000.0;
float Kcyl = 0.004f;

// DIPOLE
// One magnet
//float x0[NUM_MAGNETS*6] = { -0.02, 0.03, 0.015, 0.707, 0, 0.707};
//float x0_dist[NUM_MAGNETS*6+3] = {-0.02, 0.03, 0.015, 0.707, 0, 0.707, 0.0, 0.0, 0.0};
// // Two magnets
float x0[NUM_MAGNETS*3] = { 0, -0.009, -0.0073 };
float x0_dist[NUM_MAGNETS*6+3] = {-0.02, 0.03, 0.015, 0.707, 0, 0.707};

// CYLINDER
// One magnet
float x0_cyl[NUM_MAGNETS*7] = { -0.02, 0.03, 0.015, 0.707, 0, 0.707, 0.01};
float x0_cyl_dist[NUM_MAGNETS*7+3] = {-0.02, 0.03, 0.015, 0.707, 0, 0.707, 0.01, 0.0, 0.0, 0.0};
// Two magnets
// float x0_cyl[NUM_MAGNETS*7] = { -0.02, 0.03, 0.015, 0.707, 0, 0.707, 0.01, -0.02, 0.03, 0.015, 0.707, 0, 0.707, 0.01};
// float x0_cyl_dist[NUM_MAGNETS*7+3] = {-0.02, 0.03, 0.015, 0.707, 0, 0.707, 0.01, -0.02, 0.03, 0.015, 0.707, 0, 0.707, 0.01, 0.0, 0.0, 0.0};

uint8_t calibration_flag[NUM_SENSORS] = {1, 1}; // true
// uint8_t calibration_flag[NUM_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0}; // false
uint16_t calibIdx[NUM_SENSORS] = {0, 0};
const uint16_t calibIter = 100;
Eigen::MatrixXf magFieldDataOffset = Eigen::MatrixXf::Zero(NUM_SENSORS, 3);

// Variables for LM (lmdif1 or lmder1) optimization
float residuals[m] = {0};
int valrel = 10;
float Tol = std::sqrt(__cminpack_func__(dpmpar)(1));
// NO DISTURBANCE
float Wa_lmder[Lwa_lmder];
float Wa_lmdif[Lwa_lmdif];
int Iwa[n];
float fjac[m*n];
int ipvt[n];
// YES DISTURBANCE
float Wa_lmder_dist[Lwa_lmder_dist];
float Wa_lmdif_dist[Lwa_lmdif_dist];
int Iwa_dist[n+3];
float fjac_dist[m*(n+3)];
int ipvt_dist[n+3];

float R_j = 0.006;
float L_j = 0.006;
float V_j = PI*R_j*R_j*L_j;
Eigen::MatrixXf M_star {{0.0f}, {0.0f}, {((float)M_COEFF)/V_j} }; // axial magnet version
// float M_starf[3] = {1178925.504f, 0.0f, 0.0f};     // diametric magnet
float M_starf[3] = {0.0f, 0.0f, 1178925.504f};    // axial magnet
//Eigen::Vector<float,3> M_star { {V_j*((float)M_COEFF)},{0},{0} }; // diametric magnet version

// FUNCTIONS --------------------------------------------------------------------------------------------------------------

// Utility functions -----------------------------------------------------

// Print the data coming from the measured magnetic field by the sensors
void LM_library::printMagData(Eigen::MatrixXf magData) {
  // sending these data takes around 600 us
  Serial.print(SERIAL_HEADER);
  Serial.print(",");
  for (uint8_t sensID = 0; sensID < NUM_SENSORS; sensID++) {
    Serial.print(sensID);
    Serial.print(",");
    Serial.print(magData(sensID, 0), 2);
    Serial.print(",");
    Serial.print(magData(sensID, 1), 2);
    Serial.print(",");
    Serial.print(magData(sensID, 2), 2);
    Serial.print(",");
  }
  Serial.print(SERIAL_TAIL);
  Serial.print(",");
}

// Stream to Matlab the position and orientation of the magnet
void LM_library::streamMagTracking(float* vec, float duration) {
  // sending these data takes around 600 us
  float to_stream[NUM_MAGNETS * POSE_PARAMS + 3];
  to_stream[0] = (float) SERIAL_HEADER;
  to_stream[NUM_MAGNETS * POSE_PARAMS + 1] = duration;
  to_stream[NUM_MAGNETS * POSE_PARAMS + 2] = (float) SERIAL_TAIL;
  for (int i = 1; i <= NUM_MAGNETS*POSE_PARAMS; i++)
    to_stream[i] = vec[i-1];
  
  Serial.write((uint8_t*)to_stream, sizeof(to_stream));
}

// Print the position and orientation of the magnet
void LM_library::printMagTracking(float* vec) {
  // sending these data takes around 600 us
  float to_stream[NUM_MAGNETS * 6 + 2];
  to_stream[0] = (float) SERIAL_HEADER;
  to_stream[NUM_MAGNETS * 6 + 1] = (float) SERIAL_TAIL;
  for (int i = 1; i <= NUM_MAGNETS*6; i++)
    to_stream[i] = vec[i-1];

  for (int i = 0; i < NUM_MAGNETS * 6 + 2; i++) {
    Serial.print(to_stream[i]);
    Serial.print(", ");
  }
  Serial.println();
}

// Print the position and orientation of the magnet in cylindrical case
void LM_library::printMagTrackingCyl(float* vec) {
  // sending these data takes around 600 us
  float to_stream[NUM_MAGNETS * 9 + 2];
  to_stream[0] = (float) SERIAL_HEADER;
  to_stream[NUM_MAGNETS * 9 + 1] = (float) SERIAL_TAIL;
  uint8_t k = 1;
  for (uint8_t id = 0; id < NUM_MAGNETS; id++){
    to_stream[k++] = vec[7*id + 0];
    to_stream[k++] = vec[7*id + 1];
    to_stream[k++] = vec[7*id + 2];

    to_stream[k++] = 1-2*(vec[7*id + 5]*vec[7*id + 5]+vec[7*id + 6]*vec[7*id + 6]);   // e_bot1
    to_stream[k++] = 2*(vec[7*id + 4]*vec[7*id + 5]+vec[7*id + 3]*vec[7*id + 6]);     // e_bot2
    to_stream[k++] = 2*(vec[7*id + 4]*vec[7*id + 6]-vec[7*id + 3]*vec[7*id + 5]);     // e_bot3

    to_stream[k++] = 2*(vec[7*id + 4]*vec[7*id + 6]+vec[7*id + 3]*vec[7*id + 5]);     // e_par1
    to_stream[k++] = 2*(vec[7*id + 5]*vec[7*id + 6]-vec[7*id + 4]*vec[7*id + 3]);     // e_par2
    to_stream[k++] = 1-2*(vec[7*id + 4]*vec[7*id + 4]+vec[7*id + 5]*vec[7*id + 5]);   // e_par3
  }

  for (uint8_t i = 0; i < NUM_MAGNETS * 9 + 2; i++) {
    Serial.print(to_stream[i]);
    Serial.print(", ");
  }
  Serial.println();
}

// Print an Eigen Matrix
void LM_library::printMatrix(Eigen::MatrixXf M) {
  Serial.print("Matrix: ");
  for (int i = 0; i < M.rows(); i++) {
    for (int j = 0; j < M.cols(); j++) {
      Serial.print(M(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Print the position and orientation of the magnet in cylindrical case
// void LM_library::streamMagTracking(float* vec) {
//   // sending these data takes around 600 us
//   const int numFloats = NUM_MAGNETS * POSE_PARAMS + 2;
//   float to_stream[numFloats];
//   to_stream[0] = (float) SERIAL_HEADER;
//   to_stream[NUM_MAGNETS * POSE_PARAMS + 1] = (float) SERIAL_TAIL;
//   for (uint8_t id = 0; id < NUM_MAGNETS; id++)
//     for (uint8_t k = 0; k < POSE_PARAMS; k++)
//       to_stream[POSE_PARAMS*id + k + 1] = vec[POSE_PARAMS*id + k];

//     // Send the entire float array as bytes
//     byte* bytePtr = (byte*)to_stream;
//     Serial.write(bytePtr, 4*numFloats);
// }

void LM_library::randomize(float* vec) {
  float norm_pos[NUM_MAGNETS] = {0.0};
  uint8_t index = n / NUM_MAGNETS;

  for (uint8_t j = 0; j < NUM_MAGNETS; j++) {
    for (int i = 0; i <= 2; i++) {
        norm_pos[j] += vec[i+index*j] * vec[i+index*j];
    }

    if((std::sqrt(vec[0+index*j] * vec[0+index*j] + vec[1+index*j] * vec[1+index*j]) < 73.0/1000.0) || (std::sqrt(norm_pos[j]) > 0.4)) {
      Serial.print("Random: ");
      srand(time(NULL));
      int random_number = rand() % 100; 
      //int random_number2 = rand() % 100;
      float radius = ((float)random_number)/99.0;
      float phi = ((float)random_number)*2*PI/99.0;
      radius = 0.08 + radius*0.04;
      vec[0+index*j] = radius*cos(phi);
      vec[1+index*j] = radius*sin(phi);
      vec[2+index*j] = 0;
    }
  }
}

float LM_library::determination_coeff(Eigen::MatrixXf res, Eigen::MatrixXf B) {
  float sum = 0.0f;

  Eigen::MatrixXf B_T = B.transpose();
  Eigen::Map<Eigen::MatrixXf> B_reshaped(B_T.data(), 1, NUM_SENSORS * 3);

  for (uint8_t i = 0; i < 3 * NUM_SENSORS; i++)
    sum += (B_reshaped(0, i) - B_reshaped.norm()) * (B_reshaped(0, i) - B_reshaped.norm());

  return 1.0f - res.squaredNorm()/sum;
}

/*Added by Federico*/

Eigen::MatrixXf LM_library::Cross_rowwise(Eigen::MatrixXf A, Eigen::MatrixXf B){
  
  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(A.rows(), A.cols());

  for (uint8_t i = 0; i < A.rows(); i++)
    C.row(i) = Eigen::Vector3f(A.row(i).x(), A.row(i).y(), A.row(i).z()).cross(Eigen::Vector3f(B.row(i).x(), B.row(i).y(), B.row(i).z()));// A.row(i).cross(B.row(i));

  return C;
}

Eigen::VectorXf LM_library::Dot_rowwise(Eigen::MatrixXf A, Eigen::MatrixXf B){

  Eigen::VectorXf dProd(A.rows());
  
  for (uint8_t i = 0; i < A.rows(); i++) {
    dProd(i) = (A.row(i).array() * B.row(i).array()).sum();
  }
  return dProd;
}

// Vale
Eigen::MatrixXf LM_library::dot_star(Eigen::MatrixXf A, Eigen::MatrixXf B) {
  if (A.cols() < B.cols())
    return A.replicate(1, B.cols()).array() * B.array();
  else if (A.cols() > B.cols())
    return A.array() * B.replicate(1, A.cols()).array();
  else
    return A.array() * B.array();
}

// Vale
Eigen::MatrixXf LM_library::dot_div(Eigen::MatrixXf A, Eigen::MatrixXf B) {
  return A.array() / B.replicate(1, A.cols()).array();
}

// Vale
Eigen::MatrixXf LM_library::scalar_sum(float s, Eigen::MatrixXf A) {
  Eigen::MatrixXf S = Eigen::MatrixXf::Constant(A.rows(), A.cols(), s);
  return S + A;
}

// Vale
Eigen::VectorXf LM_library::scalar_sum(float s, Eigen::VectorXf A) {
  Eigen::VectorXf S = Eigen::VectorXf::Constant(A.rows(), A.cols(), s);
  return S + A;
}

Eigen::VectorXf LM_library::Mat2norm(Eigen::MatrixXf A){
  return A.rowwise().norm();
}

// MODEL: dipole, OPTIMIZATION: lmder1 ---------------------------------------------------------------------------------------------------

// Compute the dipole model and the Jacobian
std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> LM_library::dipole_model_jac(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, bool ConsiderDist, int iflag) {
  // B_est calculation ---------------------------------------------------------------------------------------------
  Eigen::MatrixXf B_est = Eigen::MatrixXf::Zero(1, NUM_SENSORS*3);
  // Update B_est only if the residuals need to be updated in the cost function
  if (iflag == 1) {
    Eigen::MatrixXf xRel = Eigen::MatrixXf::Ones(NUM_SENSORS * NUM_MAGNETS, 6);

    for (uint8_t i = 0; i < NUM_MAGNETS; i++) {
      Eigen::RowVectorXf tempp = x_0.block(0, i*6, 1, 6);
      xRel.block(i * NUM_SENSORS, 0, NUM_SENSORS, 6).array().rowwise() *= tempp.array();
      xRel.block(i * NUM_SENSORS, 0, NUM_SENSORS, 3) -= sPos;
    }

    // relative distance, norm of x
    Eigen::VectorXf r = (xRel.leftCols(3).rowwise().norm()).transpose();

    xRel.leftCols(3).array().colwise() /= r.array();

    // dot product between distance vector and magnetic moment
    Eigen::VectorXf dProd = (xRel.leftCols(3).array() * xRel.rightCols(3).array()).rowwise().sum();

    // scalar product * distance vector
    xRel.leftCols(3).array().colwise() *= dProd.array();

    Eigen::VectorXf r3 = (Eigen::VectorXf)pow(r.array(), 3);

    // first term in dipole equation
    xRel.leftCols(3) = 3.0f * K * (xRel.leftCols(3).array().colwise() / r3.array());

    // second term in dipole equation
    xRel.rightCols(3) = K * (xRel.rightCols(3).array().colwise() / r3.array());

    // individual magnetic field of each magnet
    Eigen::MatrixXf BsmT = xRel.leftCols(3) - xRel.rightCols(3);
    Eigen::MatrixXf Bsm = BsmT.transpose();

    // compound field
    Eigen::Map<Eigen::MatrixXf> B_estT(Bsm.data(), NUM_SENSORS * 3, NUM_MAGNETS);
    
    if (NUM_MAGNETS == 1)
      B_est = B_estT.transpose(); 
    else {
      B_est = B_estT.rowwise().sum().transpose();
    }
  }

  // J calculation ------------------------------------------------------------------------------------------------
  Eigen::MatrixXf Grad;
  if (ConsiderDist)
    Grad = Eigen::MatrixXf::Zero(NUM_SENSORS*3, NUM_MAGNETS*6+3);
  else
    Grad = Eigen::MatrixXf::Zero(NUM_SENSORS*3, NUM_MAGNETS*6);

  if (iflag == 2) {
    Eigen::VectorXf grad_vec = Eigen::VectorXf::Zero(5);

    for (int i = 0; i < NUM_SENSORS; i++) {
      for (int j = 0; j < NUM_MAGNETS; j++) {
        Eigen::VectorXf r_ji = x_0.block(0, j*6, 1, 3).transpose() - sPos.row(i).transpose();
        float r_ji_norm = r_ji.norm();
        float r_ij2   = r_ji_norm*r_ji_norm;
        float r_ij2_0 = r_ji(0)*r_ji(0);
        float r_ij2_0x5 = 5.0f*r_ij2_0;
        float r_ij2_1 = r_ji(1)*r_ji(1);
        float r_ij2_1x5 = 5.0f*r_ij2_1;
        float r_ij2_2 = r_ji(2)*r_ji(2);
        float r_ij2_2x5 = 5.0f*r_ij2_2;
        float r_ijProd = -5.0f*r_ji(0)*r_ji(1)*r_ji(2);
        float e1 = r_ji(1)*(r_ij2 - r_ij2_0x5);
        float e2 = r_ji(2)*(r_ij2 - r_ij2_0x5);
        float e3 = r_ji(0)*(r_ij2 - r_ij2_1x5);
        float e4 = r_ji(2)*(r_ij2 - r_ij2_1x5);

        Eigen::Matrix<float, 5, 3> temp {{r_ji(0)*(3.0f*r_ij2 - r_ij2_0x5),   e1,                                 e2},
                                         {e1,                                 e3,                                 r_ijProd  },
                                         {e2,                                 r_ijProd,                           r_ji(0)*(r_ij2 - r_ij2_2x5)},
                                         {e3,                                 r_ji(1)*(3.0f*r_ij2 - r_ij2_1x5),   e4},
                                         {r_ijProd,                           e4,                                 r_ji(1)*(r_ij2 - r_ij2_2x5)}};
        grad_vec = temp.cast<float>() * x_0.block(0, 3+j*6, 1, 3).transpose(); //x_0.rightCols(3).row(j).transpose();

        Eigen::Matrix<float, 3, 3> grad_temp {{grad_vec(0), grad_vec(1), grad_vec(2)},
                                              {grad_vec(1), grad_vec(3), grad_vec(4)},
                                              {grad_vec(2), grad_vec(4), -(grad_vec(0) + grad_vec(3))}};                                    
        Grad.block(i*3, j*6, 3, 3) = -3.0f * K / powf(r_ji_norm, 7) * grad_temp;

        Eigen::Matrix<float, 5, 1> temp2 {{3.0f*r_ij2_0 - r_ij2},
                                          {3.0f*r_ji(0)*r_ji(1)},
                                          {3.0f*r_ji(0)*r_ji(2)},
                                          {3.0f*r_ij2_1 - r_ij2},
                                          {3.0f*r_ji(1)*r_ji(2)}};
        Eigen::Matrix<float, 3, 3> grad_temp2 {{temp2(0), temp2(1), temp2(2)},
                                      {temp2(1), temp2(3), temp2(4)},
                                      {temp2(2), temp2(4), -(temp2(0) + temp2(3))}};
        Grad.block(i*3, 3+j*6, 3, 3) = -K / powf(r_ji_norm, 5) * grad_temp2;
      }

      if (ConsiderDist) {
        Grad.block(i*3, Grad.cols()-3, 3, 3) = -Eigen::MatrixXf::Identity(3, 3);
      }
    }
  }
  // if the Jacobian doesn't have to be computed, just put the disturbance blocks to identity if needed
  else {
    if (ConsiderDist) {
      for (uint8_t i = 0; i < NUM_SENSORS; i++)
        Grad.block(i*3, Grad.cols()-3, 3, 3) = -Eigen::MatrixXf::Identity(3, 3);
    }
  }

  return std::make_tuple(B_est, Grad);
}

// Cost function
std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> LM_library::localization_cost_dipole_jac(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist, int iflag) {
  // obtain the dipole model and the Jacobian from the dipole function
  Eigen::MatrixXf B_est, J;

  if (ConsiderDist)
    std::tie(B_est, J) = dipole_model_jac(x_0.block(0, 0, x_0.rows(), x_0.cols()-3), sPos, ConsiderDist, iflag);
  else
    std::tie(B_est, J) = dipole_model_jac(x_0, sPos, ConsiderDist, iflag);
  
  // find the residuals
  Eigen::MatrixXf B_fieldT = B_field.transpose();
  Eigen::Map<Eigen::MatrixXf> B_field_reshaped(B_fieldT.data(), 1, NUM_SENSORS * 3);

  // if iflag==2, B_est is a Zero matrix, so this computation doesn't alter res
  Eigen::MatrixXf res = Eigen::MatrixXf::Zero(1, NUM_SENSORS*3);
  
  if (ConsiderDist) {
    // Environmental disturbances
    Eigen::MatrixXf disturbances = x_0.block(0, x_0.cols()-3, 1, 3);
    Eigen::MatrixXf G = disturbances.replicate(1, NUM_SENSORS);

    res = B_field_reshaped / 100.0 - B_est - G;  // /100.0 for conversion in Gauss
  }
  else 
    res = B_field_reshaped / 100.0 - B_est;

  // check if there are sensors to discard
  if (size != 0) {
    Eigen::Map<Eigen::MatrixXf> res_reshaped1(res.data(), res.size() / 3, 3);
    res = res_reshaped1;
    for (int i = 0; i < size; i++)
      res.row(sensor_2_ignore[i]) = Eigen::MatrixXf::Zero(1, 6);
    Eigen::Map<Eigen::MatrixXf> res_reshaped2(res.transpose().data(), 1, res.size());
    res = res_reshaped2;
  }

  // compute the final residuals and Jacobian
  Eigen::MatrixXf J_final = Eigen::MatrixXf::Identity(J.rows() + NUM_MAGNETS, J.cols());
  J_final.block(0, 0, J.rows(), J.cols()) = J;

  // Eigen::MatrixXf res_finalT = Eigen::MatrixXf::Zero(res.rows(), res.cols() + NUM_MAGNETS);
  Eigen::MatrixXf res_finalT = Eigen::MatrixXf::Zero(res.rows(), res.cols() + NUM_MAGNETS);
  res_finalT.block(0, 0, res.rows(), res.cols()) = res;

  // only if ValRel > 0, so there's a penalty
  if (ValRel > 0) {
    for (int i = 0; i < NUM_MAGNETS; i++) {
      res_finalT(0, i + NUM_SENSORS * 3) = ValRel * (x_0.block(0, 3+i*6, 1, 3).squaredNorm() - 1);

      // Update the Jacobian only if needed, otherwise leave the penalty row as 0
      if (iflag == 2) {
        // if (ConsiderDist)
        //   J_final.block(J.rows() + i, 3+i*6, 1, 3) = 2.0f * ValRel * x_0.block(0, 3+i*6, 1, 3);
        // else
        //   J_final.rightCols(3).row(J.rows() + i) = 2.0f * ValRel * x_0.block(0, 3+i*6, 1, 3);
        J_final.block(J.rows() + i, 3+i*6, 1, 3) = 2.0f * ValRel * x_0.block(0, 3+i*6, 1, 3);
      }
    }
  }

  Eigen::MatrixXf res_final = res_finalT.transpose();

  return std::make_tuple(res_final, J_final);
}

// Cost function in lmder1 format
int LM_library::localization_cost_dipole_jac_cmin(void* p, int m, int n, const float* x_0, float* residuals, float* fjac, int ldfjac, int iflag) {
  OptAndInstance* optAndInstance = static_cast<OptAndInstance*>(p);
  LM_library* instance = optAndInstance->instance;
  OptimizationData* optData = optAndInstance->optData;

  //bool ConsiderDist = optData->ConsiderDist;

  Eigen::MatrixXf x0 = Eigen::MatrixXf::Zero(1, n);
  for (int i = 0; i < n; i++) {
    x0(0, i) = x_0[i];
  }

  Eigen::MatrixXf res, J;
  std::tie(res, J) = instance->localization_cost_dipole_jac(x0, optData->B_field, optData->sPos, optData->sensor_2_ignore, optData->size, optData->ValRel, optData->ConsiderDist, iflag);

  // Update only the residuals (fvec)
  if (iflag == 1) {
    float R_pow2 = instance->determination_coeff(res, optData->B_field);
    if (R_pow2 > 0.9f) {
      for (int i = 0; i < m; i++) {
        residuals[i] = res(i, 0);  // Only one column in the residual matrix
      }
    }

    return 0;
  }

  // Update only fjac
  if(iflag == 2) {
    for (int i = 0; i < NUM_MAGNETS; i++) {
      for (int j = 0; j < m*n; j++) {
        fjac[j] = J(j%m, j/m); // to fill by columns, not rows
      }
    }

    return 0;
  }

  return 0;
}

// Function to call in the main loop to execute the lmder1 optimization
void LM_library::localize_magnets_lmder(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist) {
  OptAndInstance optAndInstance;
  optAndInstance.instance = this;
  optAndInstance.optData = new OptimizationData;
  optAndInstance.optData->sPos = sPos;
  optAndInstance.optData->B_field = B_field;
  optAndInstance.optData->sensor_2_ignore = sensor_2_ignore;
  optAndInstance.optData->size = size;
  optAndInstance.optData->ValRel = ValRel;
  optAndInstance.optData->ConsiderDist = ConsiderDist;

  if (ConsiderDist)
    lmder1(localization_cost_dipole_jac_cmin, &optAndInstance, m, n+3, x0, residuals, fjac_dist, m, Tol, ipvt_dist, Wa_lmder_dist, Lwa_lmder_dist);
  else
    lmder1(localization_cost_dipole_jac_cmin, &optAndInstance, m, n, x0, residuals, fjac, m, Tol, ipvt, Wa_lmder, Lwa_lmder); 

  // deallocation
  delete optAndInstance.optData;

  // normalization
  float norm = 0.0f;
  for (uint8_t j = 0; j < NUM_MAGNETS; j++) {
    norm = 0.0f;
    for (uint8_t i = 3; i <= 5; i++) {
      norm += x0[i + 6*j] * x0[i + 6*j];
    }
    norm = std::sqrt(norm);
    for (int i = 3; i <= 5; ++i) {
      x0[i + 6*j] /= norm;
    }
  }

}

// FOR HUIMIN --------------------------------------------------------------------------

// Function to call in the main loop to execute the lmder1 optimization
void LM_library::localize_magnets_lmder_huimin(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel) {
  OptAndInstance optAndInstance;
  optAndInstance.instance = this;
  optAndInstance.optData = new OptimizationData;
  optAndInstance.optData->sPos = sPos;
  optAndInstance.optData->B_field = B_field;
  optAndInstance.optData->sensor_2_ignore = sensor_2_ignore;
  optAndInstance.optData->size = size;
  optAndInstance.optData->ValRel = ValRel;

  lmder1(localization_cost_dipole_jac_cmin_huimin, &optAndInstance, m, n, x0, residuals, fjac, m, Tol, ipvt, Wa_lmder, Lwa_lmder); 

  // deallocation
  delete optAndInstance.optData;

}

// Cost function in lmder1 format
int LM_library::localization_cost_dipole_jac_cmin_huimin(void* p, int m, int n, const float* x_0, float* residuals, float* fjac, int ldfjac, int iflag) {
  OptAndInstance* optAndInstance = static_cast<OptAndInstance*>(p);
  LM_library* instance = optAndInstance->instance;
  OptimizationData* optData = optAndInstance->optData;

  //bool ConsiderDist = optData->ConsiderDist;

  Eigen::MatrixXf x0 = Eigen::MatrixXf::Zero(1, n);
  for (int i = 0; i < n; i++) {
    x0(0, i) = x_0[i];
  }

  Eigen::MatrixXf res, J;
  std::tie(res, J) = instance->localization_cost_dipole_jac_huimin(x0, optData->B_field, optData->sPos, optData->sensor_2_ignore, optData->size, optData->ValRel, iflag);

  // Update only the residuals (fvec)
  if (iflag == 1) {
    float R_pow2 = instance->determination_coeff(res, optData->B_field);
    if (R_pow2 > 0.9f) {
      for (int i = 0; i < m; i++) {
        residuals[i] = res(i, 0);  // Only one column in the residual matrix
      }
    }

    return 0;
  }

  // Update only fjac
  if(iflag == 2) {
    for (int i = 0; i < NUM_MAGNETS; i++) {
      for (int j = 0; j < m*n; j++) {
        fjac[j] = J(j%m, j/m); // to fill by columns, not rows
      }
    }

    return 0;
  }

  return 0;
}

// Cost function
std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> LM_library::localization_cost_dipole_jac_huimin(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, int iflag) {
  // obtain the dipole model and the Jacobian from the dipole function
  Eigen::MatrixXf B_est, J;

  std::tie(B_est, J) = dipole_model_jac_huimin(x_0, sPos, iflag);
  
  // find the residuals
  Eigen::MatrixXf B_fieldT = B_field.transpose();
  Eigen::Map<Eigen::MatrixXf> B_field_reshaped(B_fieldT.data(), 1, NUM_SENSORS * 3);

  // if iflag==2, B_est is a Zero matrix, so this computation doesn't alter res
  Eigen::MatrixXf res = Eigen::MatrixXf::Zero(1, NUM_SENSORS*3);
  
  res = B_field_reshaped / 100.0 - B_est;

  // check if there are sensors to discard
  if (size != 0) {
    Eigen::Map<Eigen::MatrixXf> res_reshaped1(res.data(), res.size() / 3, 3);
    res = res_reshaped1;
    for (int i = 0; i < size; i++)
      res.row(sensor_2_ignore[i]) = Eigen::MatrixXf::Zero(1, 6);
    Eigen::Map<Eigen::MatrixXf> res_reshaped2(res.transpose().data(), 1, res.size());
    res = res_reshaped2;
  }

  // compute the final residuals and Jacobian
  Eigen::MatrixXf J_final = Eigen::MatrixXf::Identity(J.rows() + NUM_MAGNETS, J.cols());
  J_final.block(0, 0, J.rows(), J.cols()) = J;

  // Eigen::MatrixXf res_finalT = Eigen::MatrixXf::Zero(res.rows(), res.cols() + NUM_MAGNETS);
  Eigen::MatrixXf res_finalT = Eigen::MatrixXf::Zero(res.rows(), res.cols() + NUM_MAGNETS);
  res_finalT.block(0, 0, res.rows(), res.cols()) = res;

  Eigen::MatrixXf res_final = res_finalT.transpose();

  return std::make_tuple(res_final, J_final);
}

// Compute the dipole model and the Jacobian
std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> LM_library::dipole_model_jac_huimin(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, int iflag) {
  // B_est calculation ---------------------------------------------------------------------------------------------
  Eigen::MatrixXf B_est = Eigen::MatrixXf::Zero(1, NUM_SENSORS*3);

  Eigen::RowVectorXf tempp = Eigen::MatrixXf::Zero(1, 6);
  tempp(0) = x_0(0);
  tempp(1) = x_0(1);
  tempp(2) = x_0(2);
  tempp(3) = 0;
  tempp(4) = -1;
  tempp(5) = 0;

  // Serial.print("tempp: ");
  // Serial.print(tempp(0),5);
  // Serial.print(" ");
  // Serial.print(tempp(1),5);
  // Serial.print(" ");
  // Serial.print(tempp(2),5);
  // Serial.print(" ");
  // Serial.print(tempp(3));
  // Serial.print(" ");
  // Serial.print(tempp(4));
  // Serial.print(" ");
  // Serial.println(tempp(5));

  // Update B_est only if the residuals need to be updated in the cost function
  if (iflag == 1) {
    Eigen::MatrixXf xRel = Eigen::MatrixXf::Ones(NUM_SENSORS * NUM_MAGNETS, 6);

    for (uint8_t i = 0; i < NUM_MAGNETS; i++) {
      xRel.block(i * NUM_SENSORS, 0, NUM_SENSORS, 6).array().rowwise() *= tempp.array();
      xRel.block(i * NUM_SENSORS, 0, NUM_SENSORS, 3) -= sPos;
    }

    // relative distance, norm of x
    Eigen::VectorXf r = (xRel.leftCols(3).rowwise().norm()).transpose();

    xRel.leftCols(3).array().colwise() /= r.array();

    // dot product between distance vector and magnetic moment
    Eigen::VectorXf dProd = (xRel.leftCols(3).array() * xRel.rightCols(3).array()).rowwise().sum();

    // scalar product * distance vector
    xRel.leftCols(3).array().colwise() *= dProd.array();

    Eigen::VectorXf r3 = (Eigen::VectorXf)pow(r.array(), 3);

    // first term in dipole equation
    xRel.leftCols(3) = 3.0f * K * (xRel.leftCols(3).array().colwise() / r3.array());

    // second term in dipole equation
    xRel.rightCols(3) = K * (xRel.rightCols(3).array().colwise() / r3.array());

    // individual magnetic field of each magnet
    Eigen::MatrixXf BsmT = xRel.leftCols(3) - xRel.rightCols(3);
    Eigen::MatrixXf Bsm = BsmT.transpose();

    // compound field
    Eigen::Map<Eigen::MatrixXf> B_estT(Bsm.data(), NUM_SENSORS * 3, NUM_MAGNETS);
    
    if (NUM_MAGNETS == 1)
      B_est = B_estT.transpose(); 
    else {
      B_est = B_estT.rowwise().sum().transpose();
    }
  }

  // J calculation ------------------------------------------------------------------------------------------------
  Eigen::MatrixXf Grad;
  Grad = Eigen::MatrixXf::Zero(NUM_SENSORS*3, NUM_MAGNETS*3);

  if (iflag == 2) {
    Eigen::VectorXf grad_vec = Eigen::VectorXf::Zero(5);

    for (int i = 0; i < NUM_SENSORS; i++) {
      for (int j = 0; j < NUM_MAGNETS; j++) {
        Eigen::VectorXf r_ji = x_0.block(0, j*3, 1, 3).transpose() - sPos.row(i).transpose();
        float r_ji_norm = r_ji.norm();
        float r_ij2   = r_ji_norm*r_ji_norm;
        float r_ij2_0 = r_ji(0)*r_ji(0);
        float r_ij2_0x5 = 5.0f*r_ij2_0;
        float r_ij2_1 = r_ji(1)*r_ji(1);
        float r_ij2_1x5 = 5.0f*r_ij2_1;
        float r_ij2_2 = r_ji(2)*r_ji(2);
        float r_ij2_2x5 = 5.0f*r_ij2_2;
        float r_ijProd = -5.0f*r_ji(0)*r_ji(1)*r_ji(2);
        float e1 = r_ji(1)*(r_ij2 - r_ij2_0x5);
        float e2 = r_ji(2)*(r_ij2 - r_ij2_0x5);
        float e3 = r_ji(0)*(r_ij2 - r_ij2_1x5);
        float e4 = r_ji(2)*(r_ij2 - r_ij2_1x5);

        Eigen::Matrix<float, 5, 3> temp {{r_ji(0)*(3.0f*r_ij2 - r_ij2_0x5),   e1,                                 e2},
                                         {e1,                                 e3,                                 r_ijProd  },
                                         {e2,                                 r_ijProd,                           r_ji(0)*(r_ij2 - r_ij2_2x5)},
                                         {e3,                                 r_ji(1)*(3.0f*r_ij2 - r_ij2_1x5),   e4},
                                         {r_ijProd,                           e4,                                 r_ji(1)*(r_ij2 - r_ij2_2x5)}};
        grad_vec = temp.cast<float>() * tempp.block(0, 3+j*6, 1, 3).transpose(); //x_0.rightCols(3).row(j).transpose();

        // Serial.print("Grad Vec: ");
        // printMatrix(grad_vec);

        Eigen::Matrix<float, 3, 3> grad_temp {{grad_vec(0), grad_vec(1), grad_vec(2)},
                                              {grad_vec(1), grad_vec(3), grad_vec(4)},
                                              {grad_vec(2), grad_vec(4), -(grad_vec(0) + grad_vec(3))}};                                    
        Grad.block(i*3, j*6, 3, 3) = -3.0f * K / powf(r_ji_norm, 7) * grad_temp;
      }
    }
  }
  // if the Jacobian doesn't have to be computed, just put the disturbance blocks to identity if needed
  else {
  }

  return std::make_tuple(B_est, Grad);
}

// END FOR HUIMIN ----------------------------------------------------------------------

// MODEL: dipole, OPTIMIZATION: lmdif1 --------------------------------------------------------------------------------------------------------------------

// Function to calculate the dipole model
Eigen::MatrixXf LM_library::dipole_model(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos) {
  Eigen::MatrixXf xRel = Eigen::MatrixXf::Ones(NUM_SENSORS * NUM_MAGNETS, 6);

  for (uint8_t i = 0; i < NUM_MAGNETS; i++) {
    Eigen::RowVectorXf tempp = x_0.block(0, i*6, 1, 6);
    xRel.block(i * NUM_SENSORS, 0, NUM_SENSORS, 6).array().rowwise() *= tempp.array();
    xRel.block(i * NUM_SENSORS, 0, NUM_SENSORS, 3) -= sPos;
  }

  // relative distance, norm of x
  Eigen::VectorXf r = (xRel.leftCols(3).rowwise().norm()).transpose();

  xRel.leftCols(3).array().colwise() /= r.array();

  // dot product between distance vector and magnetic moment
  Eigen::VectorXf dProd = (xRel.leftCols(3).array() * xRel.rightCols(3).array()).rowwise().sum();  //dot(xRel(:,1:3), xRel(:,4:6), 2);

  // scalar product * distance vector
  xRel.leftCols(3).array().colwise() *= dProd.array();

  Eigen::VectorXf r3 = (Eigen::VectorXf)pow(r.array(), 3);

  // first term in dipole equation
  xRel.leftCols(3) = 3.0 * K * (xRel.leftCols(3).array().colwise() / r3.array());

  // second term in dipole equation
  xRel.rightCols(3) = K * (xRel.rightCols(3).array().colwise() / r3.array());

  // individual magnetic field of each magnet
  Eigen::MatrixXf BsmT = xRel.leftCols(3) - xRel.rightCols(3);
  Eigen::MatrixXf Bsm = BsmT.transpose();

  // compound field
  Eigen::Map<Eigen::MatrixXf> B_estT(Bsm.data(), NUM_SENSORS * 3, Bsm.size() / (NUM_SENSORS * 3));

  Eigen::MatrixXf B_est = Eigen::MatrixXf::Zero(1, NUM_SENSORS*3);

  if (NUM_MAGNETS == 1)
    B_est = B_estT.transpose(); 
  else {
    B_est = B_estT.rowwise().sum().transpose();
  }

  return B_est;
}

// Cost function
Eigen::MatrixXf LM_library::localization_cost_dipole(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist) {
  Eigen::MatrixXf B_est = Eigen::MatrixXf::Zero(1, NUM_SENSORS*3);
  if (ConsiderDist)
    B_est = dipole_model(x_0.block(0, 0, x_0.rows(), x_0.cols()-3), sPos);
  else
    B_est = dipole_model(x_0, sPos);
  
  Eigen::MatrixXf B_fieldT = B_field.transpose();
  Eigen::Map<Eigen::MatrixXf> B_field_reshaped(B_fieldT.data(), 1, NUM_SENSORS * 3);

  Eigen::MatrixXf res = Eigen::MatrixXf::Zero(1, NUM_SENSORS*3);

  if (ConsiderDist) {
    // Environmental disturbances
    Eigen::MatrixXf disturbances = x_0.block(0, x_0.cols()-3, 1, 3);
    Eigen::MatrixXf G = disturbances.replicate(1, NUM_SENSORS);

    res = B_field_reshaped / 100.0 - B_est - G;  // /100.0 for conversion in Gauss
  }
  else 
    res = B_field_reshaped / 100.0 - B_est;

  // if there are sensors to exclude, remove them by putting the respective residuals to 0
  if (size != 0) {
    Eigen::Map<Eigen::MatrixXf> res_reshaped1(res.data(), res.size() / 3, 3);
    res = res_reshaped1;
    for (int i = 0; i < size; i++)
      res.row(sensor_2_ignore[i]) = Eigen::MatrixXf::Zero(1, 6);
    Eigen::Map<Eigen::MatrixXf> res_reshaped2(res.transpose().data(), 1, res.size());
    res = res_reshaped2;
  }

  Eigen::MatrixXf res_finalT = Eigen::MatrixXf::Zero(res.rows(), res.cols() + NUM_MAGNETS);
  res_finalT.block(0, 0, res.rows(), res.cols()) = res;

  if (ValRel > 0) {
    for (int i = 0; i < NUM_MAGNETS; i++) {
      res_finalT(0, i + NUM_SENSORS * 3) = ValRel * (x_0.block(0, 3+i*6, 1, 3).squaredNorm() - 1);
    }
  }
  Eigen::MatrixXf res_final = res_finalT.transpose();

  return res_final;
}

// Cost function in lmdif1 format
int LM_library::localization_cost_dipole_cmin(void* p, int m, int n, const float* x_0, float* residuals, int iflag) {
  OptAndInstance* optAndInstance = static_cast<OptAndInstance*>(p);
  LM_library* instance = optAndInstance->instance;
  OptimizationData* optData = optAndInstance->optData;

  //bool ConsiderDist = optData->ConsiderDist;

  Eigen::MatrixXf x0 = Eigen::MatrixXf::Zero(1, n);
  for (int i = 0; i < n; i++) {
    x0(0, i) = x_0[i];
  }

  Eigen::MatrixXf res = instance->localization_cost_dipole(x0, optData->B_field, optData->sPos, optData->sensor_2_ignore, optData->size, optData->ValRel, optData->ConsiderDist);

  float R_pow2 = instance->determination_coeff(res, optData->B_field);
  if (R_pow2 > 0.9f) {
    for (int i = 0; i < m; i++) {
      residuals[i] = res(i, 0);  // Only one column in the residual matrix
    }
  }

  return 0;
}

// function to call in main for lmdif optimization with dipole model
void LM_library::localize_magnets_lmdif(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist) {
  OptAndInstance optAndInstance;
  optAndInstance.instance = this;
  optAndInstance.optData = new OptimizationData;
  optAndInstance.optData->sPos = sPos;
  optAndInstance.optData->B_field = B_field;
  optAndInstance.optData->sensor_2_ignore = sensor_2_ignore;
  optAndInstance.optData->size = size;
  optAndInstance.optData->ValRel = ValRel;
  optAndInstance.optData->ConsiderDist = ConsiderDist;

  if (ConsiderDist)
    lmdif1(localization_cost_dipole_cmin, &optAndInstance, m, n+3, x0, residuals, Tol, Iwa_dist, Wa_lmdif_dist, Lwa_lmdif_dist);
  else
    lmdif1(localization_cost_dipole_cmin, &optAndInstance, m, n, x0, residuals, Tol, Iwa, Wa_lmdif, Lwa_lmdif);

  // deallocation
  delete optAndInstance.optData;

  // normalization
  float norm = 0.0f;

  for (uint8_t j = 0; j < NUM_MAGNETS; j++) {
    norm = 0.0f;
    for (uint8_t i = 3; i <= 5; i++) {
      norm += x0[i + 6*j] * x0[i + 6*j];
    }
    norm = std::sqrt(norm);
    for (int i = 3; i <= 5; ++i) {
      x0[i + 6*j] /= norm;
    }
  }
}

// Utility functions for cylindrical model in matrix form -----------------------------------------------------------------------------------------------

/**
 * @brief Computes the generalized complete elliptic integral cel(kc, p, a, b),
 *        as defined by Bulirsch (1965).
 *
 * This implementation is a C translation of the algorithm presented in
 * Derby and Olbert (2009).
 *
 * @param kc   First parameter of the cel integral.
 * @param p    Second parameter of the cel integral.
 * @param a    Third parameter of the cel integral.
 * @param b    Fourth parameter of the cel integral.
 * @param tol  Tolerance used in the numerical computation of the integral.
 *
 * @return The computed value of the generalized complete elliptic integral cel.
 *
 * @note For computational efficiency, this routine assumes p > 0.
 *       This condition is always satisfied in the computation of the magnetic
 *       field and its gradient for permanent magnet cylinders with arbitrary
 *       magnetization.
 *
 * @see Bulirsch (1965), Derby & Olbert (2009)
 */
float LM_library::BulirschCEL(float kc, float p, float a, float b, float BulirschTol) {

  float eps = BulirschTol;
  float k  = fabs(kc);
  float pp = sqrtf(p);
  float aa = a;
  float em = 1;
  float bb = b/pp;
  float f = aa;

  aa += bb/pp;

  float g = k/pp;

  bb = 2 * (bb+f*g);
  pp += g;
  g = em;
  em += k;
  float kk = k;

  while (fabs(g - k) > g * eps) {

    k  = 2 * sqrtf(kk);
    kk = k * em;
    f  = aa;
    aa = aa + bb / pp;
    g  = kk / pp;
    bb = 2 * (bb + f * g);
    pp = g + pp;
    g  = em;
    em = k + em;

  }

  return (PI/2) * (bb + aa * em) / (em * (em + pp));

}

/**
 * @brief Computes Heuman's Lambda function using the generalized complete 
 *        elliptic integral CEL as defined by Bulirsch (1965).
 *
 * @param beta         Amplitude angles beta (in radians).
 * @param k            Elliptic modulus k.
 * @param BulirschTol  Tolerance used for evaluating the Bulirsch CEL integral.
 *
 * @return The computed value of the Heuman's Lambda
 *
 * @note This implementation uses the approximation that when beta is 
 *       numerically close to π/2, the result is set to 1.
 *
 * @see Bulirsch (1965), Derby & Olbert (2009)
 */
float LM_library::HeumanLambda(float beta, float k, float BulirschTol) {

  float m, x, t, t_sqrt, p, bulirsch; // aux variables
  float lambda;                       // output

  m = k*k;
  lambda = 2 * beta / PI;

  if (m != 1) {
    x = fmod(beta, PI/2);
    t = 1 - m;
    t_sqrt = sqrtf(t);
    p = 1 + m * tanf(x)*tanf(x);
    x = sinf(x) * sqrtf(p) / (PI/2);

    bulirsch = BulirschCEL(t_sqrt, p, 1, t, BulirschTol);

    t = bulirsch;
    lambda = t * x;
  }

  if ( fabs(beta - (PI/2)) < EPS_IDENTITY )
    lambda = 1;

  return lambda;

}

/**
 * @brief Auxiliary function FLAMBDA used in computing the magnetic field and field gradient 
 *        of a uniformly magnetized cylinder.
 *
 * This function evaluates an expression involving Heuman's Lambda function, used
 * to compute the axial component of the magnetic field gradient in cylindrical 
 * coordinates. It distinguishes between three regions: above, within, and below 
 * the magnet (based on the dimensionless axial coordinate `z` relative to the 
 * dimensionless half-height `l`).
 *
 * @param rho         Dimensionless radial coordinate.
 * @param z           Dimensionless axial coordinate.
 * @param l           Dimensionless half-height (length) of the magnet.
 * @param kP          Elliptic modulus corresponding to the upper surface.
 * @param kM          Elliptic modulus corresponding to the lower surface.
 * @param sigmaP      Auxiliary variable related to the upper surface.
 * @param sigmaM      Auxiliary variable related to the lower surface.
 * @param BulirschTol Tolerance used in the numerical evaluation of the Bulirsch CEL integral.
 *
 * @return Value of the auxiliary function fLambda.
 *
 * @details In the notation of the referenced article, the function implements:
 * 
 * ```
 * fLambda =   sign(1 - rho) * (Λ(σ₊, k₊) - Λ(σ₋, k₋)),   z >  L
 *             sign(1 - rho) * (Λ(σ₊, k₊) + Λ(σ₋, k₋)),   |z| ≤ L
 *           - sign(1 - rho) * (Λ(σ₊, k₊) - Λ(σ₋, k₋)),   z < -L
 * ```
 * where Λ(σ, k) is the normalized Heuman's Lambda function.
 *
 * @see Masiero and Sinibaldi (2024) https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/advs.202301033
 */
float LM_library::fLambdaLocalization(float rho, float z, float l, float kP, float kM, float sigmaP, float sigmaM, float BulirschTol) {

  float betaP, betaM, heumanP, heumanM, fL;
  betaP = asin(sigmaP);
  betaM = asin(sigmaM);
  heumanP = HeumanLambda(betaP, kP, BulirschTol); 
  heumanM = HeumanLambda(betaM, kM, BulirschTol);

  if (fabs(z) > l) {
    fL = (PI/2) * (heumanP - heumanM);
    if (z < 0)
      fL = fL * (-1);
  }
  else
    fL = (PI/2) * (heumanP + heumanM);

  if ((1 - rho) < 0)
    fL = fL * (-1);

  return fL;

}

/**
 * @brief Auxiliary function FUN1 used to compute the magnetic field generated 
 *        by a uniformly magnetized cylinder.
 *
 * This function evaluates part of the magnetic field expression in cylindrical
 * coordinates using the Bulirsch complete elliptic integral CEL and an auxiliary 
 * term involving Heuman’s Lambda function (`fLambdaLocalization`).
 *
 * @param rho         Dimensionless radial coordinate.
 * @param z           Dimensionless axial coordinate.
 * @param l           Dimensionless half-height (length) of the magnet.
 * @param auxVar      Struct containing precomputed auxiliary variables 
 *                    (e.g., distances, elliptic parameters).
 * @param BulirschTol Tolerance used in the numerical evaluation of the CEL integral.
 *
 * @return Value of the auxiliary function `f1`, representing part of the magnetic 
 *         field contribution.
 *
 * @details In the notation of the referenced article, the function implements:
 *
 * ```
 *            _                            _ +
 *           |                              |
 * f₁ = ¼ ×  | zᵢ / dᵢ · CEL(k_cᵢ, 1, 1, 1) |  +  fLambda
 *           |_                            _|
 *                                           -
 * ```
 * where `CEL(kc, 1, 1, 1)` is the complete elliptic integral computed via 
 * the Bulirsch algorithm, and `fLambda` is the output of the corresponding 
 * Lambda function correction term. The brackets indicate the jump operator.
 *
 * @note All auxiliary variables required for the computation (e.g., `kcP`, `kcM`, 
 * `zP`, `zM`, `dP`, `dM`, etc.) are expected to be precomputed and passed in the 
 * `auxVar` structure for efficiency.
 *
 * @see Masiero and Sinibaldi (2024) https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/advs.202301033
 */
float LM_library::fun1Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol) {

  float zP     = auxVar.zP;
  float zM     = auxVar.zM;
  float dP     = auxVar.dP;
  float dM     = auxVar.dM;
  float kP     = auxVar.kP;
  float kM     = auxVar.kM;
  float kcP    = auxVar.kcP;
  float kcM    = auxVar.kcM;
  float sigmaP = auxVar.sigmaP;
  float sigmaM = auxVar.sigmaM;

  float fL = fLambdaLocalization(rho, z, l, kP, kM, sigmaP, sigmaM, BulirschTol); 

  float bulirschP = BulirschCEL(kcP, 1, 1, 1, BulirschTol);
  float bulirschM = BulirschCEL(kcM, 1, 1, 1, BulirschTol);

  return 0.25f * (zP / dP * bulirschP \
                - zM / dM * bulirschM + fL);

}

/**
 * @brief Auxiliary function FUN2 used to compute the magnetic field generated 
 *        by a uniformly magnetized cylinder.
 *
 * This function evaluates an expression contributing to the radial or axial 
 * component of the magnetic field (depending on the formulation) using the 
 * generalized complete elliptic integral CEL (Bulirsch form) and a correction 
 * term based on Heuman’s Lambda function.
 *
 * @param rho         Dimensionless radial coordinate.
 * @param z           Dimensionless axial coordinate.
 * @param l           Dimensionless half-height (length) of the magnet.
 * @param auxVar      Struct containing precomputed auxiliary variables 
 *                    (e.g., distances, elliptic parameters).
 * @param BulirschTol Tolerance used in the numerical evaluation of the CEL integral.
 *
 * @return Value of the auxiliary function `f2`, representing part of the magnetic 
 *         field contribution.
 *
 * @details In the notation of the referenced article, the function implements:
 *
 * ```
 *            _                                              _ +
 *           |                                                |
 * f₂ = 1 / (4·ρ³) × | zᵢ / dᵢ · CEL(k_cᵢ, 1, 1 - 2ρ, 1 + 2ρ) |  −  fLambda
 *           |_                                              _|
 *                                                            -
 * ```
 * where `CEL(kc, 1, 1 - 2·rho, 1 + 2·rho)` is the generalized complete elliptic 
 * integral computed via the Bulirsch algorithm, and `fLambda` is the correction 
 * term derived from the Heuman’s Lambda function. The brackets indicate the jump 
 * operator.
 *
 * @note All auxiliary variables (e.g., `kcP`, `kcM`, `zP`, `zM`, `dP`, `dM`, etc.) 
 * are expected to be precomputed and stored in the `auxVar` structure for efficiency.
 *
 * @see Masiero and Sinibaldi (2024) https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/advs.202301033
 */
float LM_library::fun2Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol) {

  float zP     = auxVar.zP;
  float zM     = auxVar.zM;
  float dP     = auxVar.dP;
  float dM     = auxVar.dM;
  float kP     = auxVar.kP;
  float kM     = auxVar.kM;
  float kcP    = auxVar.kcP;
  float kcM    = auxVar.kcM;
  float sigmaP = auxVar.sigmaP;
  float sigmaM = auxVar.sigmaM;

  float fL = fLambdaLocalization(rho, z, l, kP, kM, sigmaP, sigmaM, BulirschTol);

    if (rho > 5E-4) {
      float bulirschP = BulirschCEL(kcP, 1, 1-2*rho, 1+2*rho, BulirschTol);
      float bulirschM = BulirschCEL(kcM, 1, 1-2*rho, 1+2*rho, BulirschTol);
      return 0.25f*1/powf(rho,3) * (zP / dP * bulirschP \
                                   -zM / dM * bulirschM - fL);
    }
    else
      return rho*3*PI/32 * ( zM/powf(zM*zM + 1, 2.5f) \
                           - zP/powf(zP*zP + 1, 2.5f) );   

}

/**
 * @brief Auxiliary function FUN3 used to compute both the magnetic field and 
 *        magnetic field gradient of a uniformly magnetized cylinder.
 *
 * This function evaluates a term based on the generalized complete elliptic 
 * integral CEL (in Bulirsch form) involving a transformation of the elliptic 
 * modulus, contributing to higher-order magnetic field and gradient calculations.
 *
 * @param rho         Dimensionless radial coordinate.
 * @param z           Dimensionless axial coordinate.
 * @param l           Dimensionless half-height (length) of the magnet.
 * @param auxVar      Struct containing precomputed auxiliary variables 
 *                    (e.g., distances, elliptic parameters).
 * @param BulirschTol Tolerance used in the numerical evaluation of the CEL integral.
 *
 * @return Value of the auxiliary function `f3`, representing the higher-order 
 *         contribution to the magnetic field and its gradient.
 *
 * @details In the notation of the referenced article, the function implements:
 *
 * ```
 *             _                                                           _ +
 *            |                                                             |
 * f₃ = 4 ×   | 1 / dᵢ³ · CEL(2·√k_cᵢ / (1 + k_cᵢ), 1, 0, 2 / (1 + k_cᵢ)³)  |
 *            |_                                                           _|
 *                                                                           -
 * ```
 * where `CEL(kc, p, a, b)` is the generalized complete elliptic integral 
 * computed via Bulirsch’s algorithm. The brackets indicate the jump operator.
 *
 * @note The required auxiliary variables (e.g., `kcP`, `kcM`, `dP`, `dM`, etc.) 
 * must be precomputed and passed in via the `auxVar` structure for efficiency.
 *
 * @see Masiero and Sinibaldi (2024) https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/advs.202301033
 */
float LM_library::fun3Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol) {

  float dP  = auxVar.dP;
  float dM  = auxVar.dM;
  float kcP = auxVar.kcP;
  float kcM = auxVar.kcM;

  float kcP1 = 2 * sqrtf(kcP) / (1 + kcP);
  float kcP2 = 2 / powf(1 + kcP, 3);
  float kcM1 = 2 * sqrtf(kcM) / (1 + kcM);
  float kcM2 = 2 / powf(1 + kcM, 3);

  float bulirschP = BulirschCEL(kcP1, 1, 0, kcP2, BulirschTol);
  float bulirschM = BulirschCEL(kcM1, 1, 0, kcM2, BulirschTol);

  return  4 * ((1 / powf(dP, 3)) * bulirschP \
             - (1 / powf(dM, 3)) * bulirschM );

}

/**
 * @brief Auxiliary function FUN4 used to compute the magnetic field gradient 
 *        of a uniformly magnetized cylinder.
 *
 * This function evaluates a term contributing specifically to the magnetic field 
 * gradient in cylindrical coordinates. It uses the generalized complete elliptic 
 * integral CEL (Bulirsch form).
 *
 * @param rho         Dimensionless radial coordinate.
 * @param z           Dimensionless axial coordinate.
 * @param l           Dimensionless half-height (length) of the magnet.
 * @param auxVar      Struct containing precomputed auxiliary variables 
 *                    (e.g., distances, elliptic parameters).
 * @param BulirschTol Tolerance used in the numerical evaluation of the CEL integral.
 *
 * @return Value of the auxiliary function `f4`, representing a component of the 
 *         magnetic field gradient.
 *
 * @details In the notation of the referenced article, the function implements:
 *
 * ```
 *           _                                       _ +
 *          |                                         |
 * f₄ =     | zᵢ / dᵢ³ · CEL(k_cᵢ, 1, 1 / k_cᵢ², -1)  |
 *          |_                                       _|
 *                                                     -
 * ```
 * where `CEL(kc, p, a, b)` is the generalized complete elliptic integral 
 * computed via Bulirsch’s algorithm. The brackets indicate the jump operator.
 *
 * @note Auxiliary variables such as `kcP`, `kcM`, `zP`, `zM`, `dP`, and `dM` 
 * must be precomputed and provided in the `auxVar` structure.
 *
 * @see Masiero and Sinibaldi (2024) https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/advs.202301033
 */
float LM_library::fun4Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol) {

  float zP  = auxVar.zP;
  float zM  = auxVar.zM;
  float dP  = auxVar.dP;
  float dM  = auxVar.dM;
  float kcP = auxVar.kcP;
  float kcM = auxVar.kcM;

  float kcP2 = 1 / (kcP*kcP);
  float kcM2 = 1 / (kcM*kcM);

  float bulirschP = BulirschCEL(kcP, 1, kcP2, -1, BulirschTol);
  float bulirschM = BulirschCEL(kcM, 1, kcM2, -1, BulirschTol);

  return  zP / powf(dP, 3) * bulirschP \
        - zM / powf(dM, 3) * bulirschM;

}

/**
 * @brief Auxiliary function FUN5 used to compute the magnetic field gradient 
 *        of a uniformly magnetized cylinder.
 *
 * This function contributes to the evaluation of the magnetic field gradient 
 * in cylindrical coordinates using the generalized complete elliptic integral 
 * CEL (Bulirsch form).
 *
 * @param rho         Dimensionless radial coordinate.
 * @param z           Dimensionless axial coordinate.
 * @param l           Dimensionless half-height (length) of the magnet.
 * @param auxVar      Struct containing precomputed auxiliary variables 
 *                    (e.g., distances, elliptic parameters).
 * @param BulirschTol Tolerance used in the numerical evaluation of the CEL integral.
 *
 * @return Value of the auxiliary function `f5`, representing a radial contribution 
 *         to the magnetic field gradient.
 *
 * @details In the notation of the referenced article, the function implements:
 *
 * ```
 *           _                                               _ +
 *          |                                                 |
 * f₅ =     | 1 / dᵢ³ · CEL(k_cᵢ, 1, (1 - ρ) / k_cᵢ², 1 + ρ)  |
 *          |_                                               _|
 *                                                             -
 * ```
 * where `CEL(kc, p, a, b)` is the generalized complete elliptic integral 
 * computed using the Bulirsch algorithm. The brackets indicate the jump operator.
 *
 * @note All auxiliary variables such as `kcP`, `kcM`, `dP`, and `dM` 
 * must be precomputed and provided in the `auxVar` structure.
 *
 * @see Masiero and Sinibaldi (2024) https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/advs.202301033
 */
float LM_library::fun5Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol) {

  float dP  = auxVar.dP;
  float dM  = auxVar.dM;
  float kcP = auxVar.kcP;
  float kcM = auxVar.kcM;

  float rhoP = (1 - rho) / (kcP * kcP);
  float rhoM = (1 - rho) / (kcM * kcM);

  float bulirschP = BulirschCEL(kcP, 1, rhoP, 1 + rho, BulirschTol);
  float bulirschM = BulirschCEL(kcM, 1, rhoM, 1 + rho, BulirschTol);

  return 1.0f / powf(dP, 3) * bulirschP \
       - 1.0f / powf(dM, 3) * bulirschM;
}

// MODEL: cylindrical, OPTIMIZATION: lmdif1 -----------------------------------------------------------------------------------------------
// cylindrical model computation
Eigen::MatrixXf LM_library::cyl_modelf(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, float BulirschTol){

  Eigen::MatrixXf Best = Eigen::MatrixXf::Zero(NUM_SENSORS, 3);

  for (uint8_t j = 0; j < NUM_MAGNETS; j++){

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      // extract the pose of the j-th magnet
      // extract the position of the j-th magnet
      Eigen::Vector3f r_j = x_0.row(j).leftCols(3);  // [m]
      // extract the orientation of the j-th magnet and normalize
      Eigen::VectorXf q_j = x_0.row(j).rightCols(4); // [-]
      q_j /= q_j.norm();
      // extract magnet unit vectors w.r.t. the Global Reference frame
      // e_parallel is the unit vector aligned with cylinder axis
      Eigen::Vector3f e_par (   2*(q_j(1)*q_j(3)+q_j(0)*q_j(2)), \
                                2*(q_j(2)*q_j(3)-q_j(1)*q_j(0)), \
                              1-2*(q_j(1)*q_j(1)+q_j(2)*q_j(2)) );    
      // e_bot is a unit vector perpendicular to cylinder axis which is
      // parallel to the diametric component of magnetization (if present)
      Eigen::Vector3f e_bot ( 1-2*(q_j(2)*q_j(2)+q_j(3)*q_j(3)), \
                                2*(q_j(1)*q_j(2)+q_j(0)*q_j(3)), \
                                2*(q_j(1)*q_j(3)-q_j(0)*q_j(2)) );
      // projection of the magnetization components along the magnet unit
      // vectors/axes, w.r.t. the global reference frame.
      // N.B.:
      // M_star(3) is M_parallel or axial component of the magnetization
      // M_star(1) is M_bot or diametric component of the magnetization
      Eigen::Vector3f M_par = M_starf[2]*e_par; // [A/m]
      Eigen::Vector3f M_bot = M_starf[0]*e_bot; // [A/m]
      // recast matrix of vector distances normalized by R_j
      Eigen::Vector3f r_ij {(sPos(i,0) - r_j(0))/R_j, (sPos(i,1) - r_j(1))/R_j, (sPos(i,2) - r_j(2))/R_j};

      // Eigen::MatrixXf rho_temp = Cross_rowwise(r_ij, e_par);
      float rho = r_ij.cross(e_par).norm();
      Eigen::Vector3f nu = r_ij.cross(e_par) / rho;     // radial component [-]
      float z   = r_ij.dot(e_par);                      // axial component  [-]

      // definition extension close to axis
      if(rho <= AXIS_TH){
        rho = 0;
        nu(0) = 0; nu(1) = 0; nu(2) = 0;
      }

      // auxiliary vectors for projection and reflection of local cylindrical components
      Eigen::Vector3f v = (r_ij.dot(M_par - M_bot)) * e_par - M_starf[2] * r_ij;
      Eigen::Vector3f u = rho * (M_bot - 2.0f * (M_bot.dot(nu)) * nu);

      // adimensional semi-length
      float l = (L_j/2.0f)/R_j;

      // compute auxiliary expressions
      float zP      = z + l;
      float zM      = z - l;
      float dP      = sqrtf( (1+rho)*(1+rho) + zP*zP) ;
      float dM      = sqrtf( (1+rho)*(1+rho) + zM*zM) ;
      float kP      = sqrtf( (4*rho) / (dP*dP) );
      float kM      = sqrtf( (4*rho) / (dM*dM) );
      float kcP     = sqrtf( 1 - kP*kP );
      float kcM     = sqrtf( 1 - kM*kM );
      float sigmaP  = sqrtf( (zP*zP)/( (1-rho)*(1-rho) + zP*zP ) );
      float sigmaM  = sqrtf( (zM*zM)/( (1-rho)*(1-rho) + zM*zM ) );

      varsF AuxVars;
      AuxVars.zP = zP;
      AuxVars.zM = zM;
      AuxVars.dP = dP;
      AuxVars.dM = dM;
      AuxVars.kP = kP;
      AuxVars.kM = kM;
      AuxVars.kcP = kcP;
      AuxVars.kcM = kcM;
      AuxVars.sigmaP = sigmaP;
      AuxVars.sigmaM = sigmaM;

      // Field computation in coordinate free representation
      float f1 = fun1Localization(rho, z, l, AuxVars, BulirschTol);
      float f2 = fun2Localization(rho, z, l, AuxVars, BulirschTol);
      float f3 = fun3Localization(rho, z, l, AuxVars, BulirschTol);

      Best.row(i) += (f1 * (2*M_par - M_bot) + f2 * u + f3 * v).transpose();
    } 
  }
  Best *= Kcyl;
  Eigen::MatrixXf Best_transpose = Best.transpose();
  Eigen::Map<Eigen::MatrixXf> B_estT(Best_transpose.data(), NUM_SENSORS * 3, Best_transpose.size() / (NUM_SENSORS * 3));
  Eigen::MatrixXf Best_final = B_estT.transpose();
  return Best_final;

}

// Cost function
Eigen::MatrixXf LM_library::localization_cost_cyl(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel) {
  Eigen::MatrixXf B_est = cyl_modelf(x_0, sPos, 1e-6);
  Eigen::MatrixXf B_fieldT = B_field.transpose();
  Eigen::Map<Eigen::MatrixXf> B_field_reshaped(B_fieldT.data(), 1, NUM_SENSORS * 3);
  Eigen::MatrixXf res = B_field_reshaped/100.0f - B_est; 

  if (size != 0) {
    Eigen::Map<Eigen::MatrixXf> res_reshaped1(res.data(), res.size() / 3, 3);
    res = res_reshaped1;
    for (int i = 0; i < size; i++)
      res.row(sensor_2_ignore[i]) = Eigen::MatrixXf::Zero(1, 6);
    Eigen::Map<Eigen::MatrixXf> res_reshaped2(res.transpose().data(), 1, res.size());
    res = res_reshaped2;
  }

  Eigen::MatrixXf x_0T = x_0.transpose();
  Eigen::Map<Eigen::MatrixXf> x_0_reshapedT(x_0T.data(), 7, x_0.size() / 7);
  Eigen::MatrixXf x_0_reshaped = x_0_reshapedT.transpose();
  Eigen::MatrixXf res_finalT = Eigen::MatrixXf::Zero(res.rows(), res.cols() + NUM_MAGNETS);
  res_finalT.block(0, 0, res.rows(), res.cols()) = res;

  if (ValRel > 0) {
    for (int i = 0; i < NUM_MAGNETS; i++) {
      res_finalT(0, i + NUM_SENSORS * 3) = ValRel * (x_0_reshaped.row(i).rightCols(4).squaredNorm() - 1);
    }
  }
  Eigen::MatrixXf res_final = res_finalT.transpose();
  return res_final;
}

// Cost function in lmdif1 form
int LM_library::localization_cost_cyl_cmin(void* p, int m, int n, const float* x_0, float* residuals, int iflag) {
  OptAndInstance* optAndInstance = static_cast<OptAndInstance*>(p);
  LM_library* instance = optAndInstance->instance;
  OptimizationData* optData = optAndInstance->optData;

  Eigen::MatrixXf x0 = Eigen::MatrixXf::Zero(1, n);
  for (int i = 0; i < n; i++) {
    x0(0, i) = x_0[i];
  }

  Eigen::MatrixXf res = instance->localization_cost_cyl(x0, optData->B_field, optData->sPos, optData->sensor_2_ignore, optData->size, optData->ValRel);

  float R_pow2 = instance->determination_coeff(res, optData->B_field);
  if (R_pow2 > 0.9f) {
    for (int i = 0; i < m; i++) {
      residuals[i] = res(i, 0);  // Only one column in the residual matrix
    }
  }

  return 0;
}

// Function to call in main
void LM_library::localize_magnets_cyl_lmdif(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, const int size, uint8_t ValRel) {
  OptAndInstance optAndInstance;
  optAndInstance.instance = this;
  optAndInstance.optData = new OptimizationData;
  optAndInstance.optData->sPos = sPos;
  optAndInstance.optData->B_field = B_field;
  optAndInstance.optData->sensor_2_ignore = sensor_2_ignore;
  optAndInstance.optData->size = size;
  optAndInstance.optData->ValRel = ValRel;

  lmdif1(localization_cost_cyl_cmin, &optAndInstance, m, n, x0, residuals, Tol, Iwa, Wa_lmdif, Lwa_lmdif);

  // deallocation
  delete optAndInstance.optData;

  // normalization
  float norm = 0.0;
  for (int i = 3; i <= 6; i++) {
    norm += x0[i] * x0[i];
  }
  norm = std::sqrt(norm);
  for (int i = 3; i <= 6; i++) {
    x0[i] /= norm;
  }
}

// MODEL: cylindrical, OPTIMIZATION: lmder1 ---------------------------------------------------------------------------------------------------------------------------------
// Cylindrical model, returning both Best and Grad
std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> LM_library::cyl_model_jacf(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, int iflag, float BulirschTol, bool ConsiderDist){
  // Serial.print("x_0: ");
  // printMatrix(x_0);
  
  Eigen::MatrixXf Best = Eigen::MatrixXf::Zero(NUM_SENSORS, 3);
  Eigen::MatrixXf Grad;
  if (ConsiderDist)
    Grad = Eigen::MatrixXf::Zero(NUM_SENSORS*3, NUM_MAGNETS*7+3);
  else
    Grad = Eigen::MatrixXf::Zero(NUM_SENSORS*3, NUM_MAGNETS*7);
  Eigen::VectorXf J_par = Eigen::VectorXf::Zero(9);
  Eigen::VectorXf J_bot = Eigen::VectorXf::Zero(9);

  for (uint8_t j = 0; j < NUM_MAGNETS; j++){

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      // extract the pose of the j-th magnet
      // extract the position of the j-th magnet
      Eigen::Vector3f r_j = x_0.block(0, j*7, 1, 3).transpose();  // [m]
      // extract the orientation of the j-th magnet and normalize
      Eigen::VectorXf q_j = x_0.block(0, 3+j*7, 1, 4).transpose(); // [-]
      //q_j /= q_j.norm();
      // extract magnet unit vectors w.r.t. the Global Reference frame
      // e_parallel is the unit vector aligned with cylinder axis
      Eigen::Vector3f e_par (   2*(q_j(1)*q_j(3)+q_j(0)*q_j(2)), \
                                2*(q_j(2)*q_j(3)-q_j(1)*q_j(0)), \
                              1-2*(q_j(1)*q_j(1)+q_j(2)*q_j(2)) );        
      // e_bot is a unit vector perpendicular to cylinder axis which is
      // parallel to the diametric component of magnetization (if present)
      Eigen::Vector3f e_bot ( 1-2*(q_j(2)*q_j(2)+q_j(3)*q_j(3)), \
                                2*(q_j(1)*q_j(2)+q_j(0)*q_j(3)), \
                                2*(q_j(1)*q_j(3)-q_j(0)*q_j(2)) );
      // projection of the magnetization components along the magnet unit
      // vectors/axes, w.r.t. the global reference frame.
      // N.B.:
      // M_star(3) is M_parallel or axial component of the magnetization
      // M_star(1) is M_bot or diametric component of the magnetization
      Eigen::Vector3f M_par = M_starf[2]*e_par; // [A/m]
      Eigen::Vector3f M_bot = M_starf[0]*e_bot; // [A/m]
      // recast matrix of vector distances normalized by R_j
      Eigen::Vector3f r_ij {(sPos(i,0) - r_j(0))/R_j, (sPos(i,1) - r_j(1))/R_j, (sPos(i,2) - r_j(2))/R_j};

      // Eigen::MatrixXf rho_temp = Cross_rowwise(r_ij, e_par);
      float rho = r_ij.cross(e_par).norm();
      Eigen::Vector3f nu = r_ij.cross(e_par) / rho;     // radial component [-]
      float z   = r_ij.dot(e_par);                      // axial component  [-]

      // definition extension close to axis
      if(rho <= AXIS_TH){
        rho = 0;
        nu(0) = 0; nu(1) = 0; nu(2) = 0;
      }

      // auxiliary vectors for projection and reflection of local cylindrical components
      Eigen::Vector3f v = (r_ij.dot(M_par - M_bot)) * e_par - M_starf[2] * r_ij;
      Eigen::Vector3f u = rho * (M_bot - 2.0f * (M_bot.dot(nu)) * nu);

      // adimensional semi-length
      float l = (L_j/2.0f)/R_j;

      // compute auxiliary expressions
      float zP      = z + l;
      float zM      = z - l;
      float dP      = sqrtf( (1+rho)*(1+rho) + zP*zP) ;
      float dM      = sqrtf( (1+rho)*(1+rho) + zM*zM) ;
      float kP      = sqrtf( (4*rho) / (dP*dP) );
      float kM      = sqrtf( (4*rho) / (dM*dM) );
      float kcP     = sqrtf( 1 - kP*kP );
      float kcM     = sqrtf( 1 - kM*kM );
      float sigmaP  = sqrtf( (zP*zP)/( (1-rho)*(1-rho) + zP*zP ) );
      float sigmaM  = sqrtf( (zM*zM)/( (1-rho)*(1-rho) + zM*zM ) );

      varsF AuxVars;
      AuxVars.zP = zP;
      AuxVars.zM = zM;
      AuxVars.dP = dP;
      AuxVars.dM = dM;
      AuxVars.kP = kP;
      AuxVars.kM = kM;
      AuxVars.kcP = kcP;
      AuxVars.kcM = kcM;
      AuxVars.sigmaP = sigmaP;
      AuxVars.sigmaM = sigmaM;

      // Field computation in coordinate free representation
      float f1 = fun1Localization(rho, z, l, AuxVars, BulirschTol);
      float f2 = fun2Localization(rho, z, l, AuxVars, BulirschTol);
      float f3 = fun3Localization(rho, z, l, AuxVars, BulirschTol);

      if (iflag == 1)
        Best.row(i) += (f1 * (2*M_par - M_bot) + f2 * u + f3 * v).transpose();

      if (iflag == 2) {
        // Jacobian    
        float c = r_ij.dot(e_bot) / rho;
        float s = r_ij.dot(e_par.cross(e_bot)) / rho;

        if(rho <= AXIS_TH){
          c = 0.0f;
          s = 0.0f;
        }

        float f4 = fun4Localization(rho, z, l, AuxVars, BulirschTol);
        float f5 = fun5Localization(rho, z, l, AuxVars, BulirschTol);

        // Computation of auxiliary expressions for the gradient in local coordinates
        float g = 2.0f * f3 - f5;
        float g1 = c*c*g;
        float g2 = s*c*g;
        float g3 = s*s*g;
        float g4 = c*f4;
        float g5 = s*f4;
        float g6 = 8.0f * f2 + f4;
        // Calculate J_par
        J_par(0) = g1 - f3;
        J_par(1) = g2;
        J_par(2) = g4;
        J_par(3) = J_par(1);
        J_par(4) = g3 - f3;
        J_par(5) = g5;
        J_par(6) = J_par(2);
        J_par(7) = J_par(5);
        J_par(8) = f5;

        // Calculate J_bot
        J_bot(0) = c * (6.0f * f2 - c*c*g6);
        J_bot(1) = s * (2.0f * f2 - c*c*g6);
        J_bot(2) = J_par(1);
        J_bot(3) = J_bot(1);
        J_bot(4) = c * (2.0f * f2 - s*s*g6);
        J_bot(5) = g2;
        J_bot(6) = J_par(1);
        J_bot(7) = J_bot(5);
        J_bot(8) = J_par(2);

        if (rho <= AXIS_TH) {
          float gstar = PI/4 * ( 1/powf(1+zP*zP,1.5f) - 1/powf(1+zM*zM,1.5f) );

          J_par(0) = -gstar;
          J_par(1) = -gstar;
          J_bot(2) = -gstar;
        }

        Eigen::Matrix3f grad_rot;
        grad_rot.col(0) = e_bot;
        grad_rot.col(1) = e_par.cross(e_bot);
        grad_rot.col(2) = e_par;

        Eigen::Matrix3f G;
        G.col(0) = (M_starf[2] * J_par.block(0, 0, 3, 1) + M_starf[0] * J_bot.block(0, 0, 3, 1))/R_j;
        G.col(1) = (M_starf[2] * J_par.block(3, 0, 3, 1) + M_starf[0] * J_bot.block(3, 0, 3, 1))/R_j;
        G.col(2) = (M_starf[2] * J_par.block(6, 0, 3, 1) + M_starf[0] * J_bot.block(6, 0, 3, 1))/R_j;

        // Cartesian coordinates of the magnetic field
        Grad.block(i*3, j*7, 3, 3) = Kcyl * grad_rot * G * grad_rot.transpose();

        float f1_drho    = f4 / 2.0f;
        float f1_dz      = f5 / 2.0f;
        float f2_drho    = -1.0f / (2.0f*rho)*(f4 + 6.0f*f2);
        float f2_dz      = 1.0f / (2.0f*rho)*(2.0f*f3 - f5);
        float f3_drho    = (f5 - 2.0f*f3) / rho;
        float f3_dz      = -f4 / rho;

        if (rho <= AXIS_TH) {
          f2_drho = -3.0f*PI/32.0f * ( zP/sqrt(powf(1+zP*zP,5)) - zM/sqrt(powf(1+zM*zM, 5)) );
          f2_dz   = 0.0f;
          f3_drho = 0.0f;
          f3_dz   = -3.0f*PI/4.0f * ( zP/sqrt(powf(1+zP*zP,5)) - zM/sqrt(powf(1+zM*zM, 5)) );
        }

        Eigen::Vector3f de_par_dqc = Eigen::VectorXf::Zero(3);
        Eigen::Vector3f de_bot_dqc = Eigen::VectorXf::Zero(3);
        Eigen::Vector3f dz_dqc_vec = Eigen::VectorXf::Zero(3);
        float dz_dqc = 0.0f;
        Eigen::Vector3f drho_dqc_vec = Eigen::VectorXf::Zero(3);
        float drho_dqc = 0.0f;

        for (int p = 0; p < 4; p++) {
          switch (p) {
            case 0:
              de_par_dqc(0) = q_j(2);
              de_par_dqc(1) = -q_j(1);
              de_par_dqc(2) = 0.0f;
              de_par_dqc *= 2.0f;

              de_bot_dqc(0) = 0.0f;
              de_bot_dqc(1) = q_j(3);
              de_bot_dqc(2) = -q_j(2);
              de_bot_dqc *= 2.0f;

              dz_dqc_vec(0) = q_j(2);
              dz_dqc_vec(1) = -q_j(1);
              dz_dqc_vec(2) = 0.0f;
              dz_dqc = 2.0f * r_ij.dot(dz_dqc_vec);

              drho_dqc_vec(0) = q_j(1) * r_ij(2);
              drho_dqc_vec(1) = q_j(2) * r_ij(2);
              drho_dqc_vec(2) = -(q_j(1)*r_ij(0) + q_j(2)*r_ij(1));
              drho_dqc = 2.0f * drho_dqc_vec.dot(nu);
              break;
            case 1:
              de_par_dqc(0) = q_j(3);
              de_par_dqc(1) = -q_j(0);
              de_par_dqc(2) = -2.0f*q_j(1);
              de_par_dqc *= 2.0f;

              de_bot_dqc(0) = 0.0f;
              de_bot_dqc(1) = q_j(2);
              de_bot_dqc(2) = q_j(3);
              de_bot_dqc *= 2.0f;

              dz_dqc_vec(0) = q_j(3);
              dz_dqc_vec(1) = -q_j(0);
              dz_dqc_vec(2) = -2.0f*q_j(1);
              dz_dqc = 2.0f * r_ij.dot(dz_dqc_vec);

              drho_dqc_vec(0) = -2.0f*q_j(1)*r_ij(1) + q_j(0)*r_ij(2);
              drho_dqc_vec(1) = 2.0f*q_j(1)*r_ij(0) + q_j(3)*r_ij(2);
              drho_dqc_vec(2) = -(q_j(0)*r_ij(0) + q_j(3)*r_ij(1));
              drho_dqc = 2.0f * drho_dqc_vec.dot(nu);
              break;
            case 2:
              de_par_dqc(0) = q_j(0);
              de_par_dqc(1) = q_j(3);
              de_par_dqc(2) = -2.0f*q_j(2);
              de_par_dqc *= 2.0f;

              de_bot_dqc(0) = -2.0f*q_j(2);
              de_bot_dqc(1) = q_j(1);
              de_bot_dqc(2) = -q_j(0);
              de_bot_dqc *= 2.0f;

              dz_dqc_vec(0) = q_j(0);
              dz_dqc_vec(1) = q_j(3);
              dz_dqc_vec(2) = -2.0f*q_j(2);
              dz_dqc = 2.0f * r_ij.dot(dz_dqc_vec);

              drho_dqc_vec(0) = -(2.0f*q_j(2)*r_ij(1) + q_j(3)*r_ij(2));
              drho_dqc_vec(1) = 2.0f*q_j(2)*r_ij(0) + q_j(0)*r_ij(2);
              drho_dqc_vec(2) = q_j(3)*r_ij(0) - q_j(0)*r_ij(1);
              drho_dqc = 2.0f * drho_dqc_vec.dot(nu);
              break;
            case 3:
              de_par_dqc(0) = q_j(1);
              de_par_dqc(1) = q_j(2);
              de_par_dqc(2) = 0.0f;
              de_par_dqc *= 2.0f;

              de_bot_dqc(0) = -2.0f*q_j(3);
              de_bot_dqc(1) = q_j(0);
              de_bot_dqc(2) = q_j(1);
              de_bot_dqc *= 2.0f;

              dz_dqc_vec(0) = q_j(1);
              dz_dqc_vec(1) = q_j(2);
              dz_dqc_vec(2) = 0.0f;
              dz_dqc = 2.0f * r_ij.dot(dz_dqc_vec);

              drho_dqc_vec(0) = -q_j(2) * r_ij(2);
              drho_dqc_vec(1) = q_j(1) * r_ij(2);
              drho_dqc_vec(2) = q_j(2)*r_ij(0) - q_j(1)*r_ij(1);
              drho_dqc = 2.0f * drho_dqc_vec.dot(nu);
              break;
            default:
              break;
          }

          Eigen::Vector3f dv_dep1, dv_dep2, dv_dep3, pXe_bot, nuXp;
          Eigen::Vector3f dv_deb1, dv_deb2, dv_deb3, du_dep1, du_dep2, du_dep3, du_deb1, du_deb2, du_deb3;

          dv_dep1(0) = r_ij(0)*M_starf[2]*e_par(0)+r_ij.dot(M_starf[2]*e_par-M_starf[0]*e_bot);
          dv_dep1(1) = r_ij(0)*M_starf[2]*e_par(1);
          dv_dep1(2) = r_ij(0)*M_starf[2]*e_par(2);

          dv_dep2(0) = r_ij(1)*M_starf[2]*e_par(0);
          dv_dep2(1) = r_ij(1)*M_starf[2]*e_par(1)+r_ij.dot(M_starf[2]*e_par-M_starf[0]*e_bot);
          dv_dep2(2) = r_ij(1)*M_starf[2]*e_par(2);

          dv_dep3(0) = r_ij(2)*M_starf[2]*e_par(0);
          dv_dep3(1) = r_ij(2)*M_starf[2]*e_par(1);
          dv_dep3(2) = r_ij(2)*M_starf[2]*e_par(2)+r_ij.dot(M_starf[2]*e_par-M_starf[0]*e_bot);

          dv_deb1(0) = -r_ij(0)*M_starf[0]*e_par(0);
          dv_deb1(1) = -r_ij(0)*M_starf[0]*e_par(1);
          dv_deb1(2) = -r_ij(0)*M_starf[0]*e_par(2);

          dv_deb2(0) = -r_ij(1)*M_starf[0]*e_par(0); 
          dv_deb2(1) = -r_ij(1)*M_starf[0]*e_par(1);
          dv_deb2(2) = -r_ij(1)*M_starf[0]*e_par(2);

          dv_deb3(0) = -r_ij(2)*M_starf[0]*e_par(0);
          dv_deb3(1) = -r_ij(2)*M_starf[0]*e_par(1);
          dv_deb3(2) = -r_ij(2)*M_starf[0]*e_par(2);
                  
          pXe_bot = r_ij.cross(e_bot);
          nuXp    = nu.cross(r_ij);

          du_dep1(0) = M_starf[0] * ( nuXp(0) * ( e_bot(0)+2*( nu.dot(e_bot) * nu(0) ) ) + 2 *   pXe_bot(0) * nu(0) );
          du_dep1(1) = M_starf[0] * ( nuXp(0) * ( e_bot(1)+2*( nu.dot(e_bot) * nu(1) ) ) + 2 * ( pXe_bot(0) * nu(1) + nu.dot(e_bot)*(-r_ij(2)) ) );
          du_dep1(2) = M_starf[0] * ( nuXp(0) * ( e_bot(2)+2*( nu.dot(e_bot) * nu(2) ) ) + 2 * ( pXe_bot(0) * nu(2) + nu.dot(e_bot)*(+r_ij(1)) ) );

          du_dep2(0) = M_starf[0] * ( nuXp(1) * ( e_bot(0)+2*( nu.dot(e_bot) * nu(0) ) ) + 2 * ( pXe_bot(1) * nu(0) + nu.dot(e_bot)*(+r_ij(2)) ) );
          du_dep2(1) = M_starf[0] * ( nuXp(1) * ( e_bot(1)+2*( nu.dot(e_bot) * nu(1) ) ) + 2 *   pXe_bot(1) * nu(1) );
          du_dep2(2) = M_starf[0] * ( nuXp(1) * ( e_bot(2)+2*( nu.dot(e_bot) * nu(2) ) ) + 2 * ( pXe_bot(1) * nu(2) + nu.dot(e_bot)*(-r_ij(0)) ) );

          du_dep3(0) = M_starf[0] * ( nuXp(2) * ( e_bot(0)+2*( nu.dot(e_bot) * nu(0) ) ) + 2 * ( pXe_bot(2) * nu(0) + nu.dot(e_bot)*(-r_ij(1)) ) );
          du_dep3(1) = M_starf[0] * ( nuXp(2) * ( e_bot(1)+2*( nu.dot(e_bot) * nu(1) ) ) + 2 * ( pXe_bot(2) * nu(1) + nu.dot(e_bot)*(+r_ij(0)) ) );
          du_dep3(2) = M_starf[0] * ( nuXp(2) * ( e_bot(2)+2*( nu.dot(e_bot) * nu(2) ) ) + 2 *   pXe_bot(2) * nu(2) );

          du_deb1(0) = -M_starf[0] * rho * (2*nu(0)*nu(0)-1);
          du_deb1(1) = -M_starf[0] * rho * (2*nu(0)*nu(1));
          du_deb1(2) = -M_starf[0] * rho * (2*nu(0)*nu(2));

          du_deb2(0) = -M_starf[0] * rho * (2*nu(1)*nu(0));
          du_deb2(1) = -M_starf[0] * rho * (2*nu(1)*nu(1)-1);
          du_deb2(2) = -M_starf[0] * rho * (2*nu(1)*nu(2));

          du_deb3(0) = -M_starf[0] * rho * (2*nu(2)*nu(0));
          du_deb3(1) = -M_starf[0] * rho * (2*nu(2)*nu(1));
          du_deb3(2) = -M_starf[0] * rho * (2*nu(2)*nu(2)-1);

          Eigen::Vector3f auxD1, auxD2, dv_dqc, du_dqc;
          auxD1 = 2 * e_par * M_starf[2] - e_bot * M_starf[0];
          auxD2 = 2 * M_starf[2] * de_par_dqc - M_starf[0] * de_bot_dqc;
          dv_dqc = dv_dep1 * de_par_dqc(0) + dv_dep2 * de_par_dqc(1) + dv_dep3 * de_par_dqc(2) + dv_deb1 * de_bot_dqc(0) + dv_deb2 * de_bot_dqc(1) + dv_deb3 * de_bot_dqc(2);
          du_dqc = du_dep1 * de_par_dqc(0) + du_dep2 * de_par_dqc(1) + du_dep3 * de_par_dqc(2) + du_deb1 * de_bot_dqc(0) + du_deb2 * de_bot_dqc(1) + du_deb3 * de_bot_dqc(2);

          float df1_dqc = 0.0f, df2_dqc = 0.0f, df3_dqc = 0.0f;
          df1_dqc = f1_drho * drho_dqc + f1_dz * dz_dqc;
          df2_dqc = f2_drho * drho_dqc + f2_dz * dz_dqc;
          df3_dqc = f3_drho * drho_dqc + f3_dz * dz_dqc;

          Grad.block(i*3, j*(n/NUM_MAGNETS)+3+p, 3, 1) = -Kcyl * (df1_dqc * auxD1 + f1 * auxD2 + df3_dqc * v + f3 * dv_dqc + df2_dqc * u + f2 * du_dqc);
        }
      }

      if (j == NUM_MAGNETS-1) {
        if (ConsiderDist) {
          Grad.block(i*3, Grad.cols()-3, 3, 3) = -Eigen::MatrixXf::Identity(3, 3);
        }
      }
    } 
  }

  if (iflag == 1)
    Best *= Kcyl;

  Eigen::MatrixXf Best_transpose = Best.transpose();
  Eigen::Map<Eigen::MatrixXf> B_estT(Best_transpose.data(), NUM_SENSORS * 3, Best_transpose.size() / (NUM_SENSORS * 3));
  Eigen::MatrixXf Best_final = B_estT.transpose();

  return std::make_tuple(Best_final, Grad);

}

// Cost function
std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> LM_library::localization_cost_cyl_jac(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist, int iflag) {
  // obtain the dipole model and the Jacobian from the dipole function
  Eigen::MatrixXf B_est, J;
  if (ConsiderDist)
    std::tie(B_est, J) = LM_library::cyl_model_jacf_opt(x_0.block(0, 0, x_0.rows(), x_0.cols()-3), sPos, iflag, 1E-6, ConsiderDist);
  else
    std::tie(B_est, J) = LM_library::cyl_model_jacf_opt(x_0, sPos, iflag, 1E-6, ConsiderDist);

  // find the residuals
  Eigen::MatrixXf B_fieldT = B_field.transpose();
  Eigen::Map<Eigen::MatrixXf> B_field_reshaped(B_fieldT.data(), 1, NUM_SENSORS * 3);

  // if iflag==2, B_est is a Zero matrix, so this computation doesn't alter res
  Eigen::MatrixXf res = Eigen::MatrixXf::Zero(1, NUM_SENSORS*3);
  
  if (ConsiderDist) {
    // Environmental disturbances
    Eigen::MatrixXf disturbances = x_0.block(0, x_0.cols()-3, 1, 3);
    Eigen::MatrixXf G_dist = disturbances.replicate(1, NUM_SENSORS);

    res = B_field_reshaped / 100.0f - B_est - G_dist;  // /100.0 for conversion in Gauss
  }
  else
    res = B_field_reshaped / 100.0f - B_est;  // /100.0 for conversion in Gauss

  // check if there are sensors to discard
  if (size != 0) {
    Eigen::Map<Eigen::MatrixXf> res_reshaped1(res.data(), res.size() / 3, 3);
    res = res_reshaped1;
    for (int i = 0; i < size; i++)
      res.row(sensor_2_ignore[i]) = Eigen::MatrixXf::Zero(1, 6);
    Eigen::Map<Eigen::MatrixXf> res_reshaped2(res.transpose().data(), 1, res.size());
    res = res_reshaped2;
  }

  // compute the final residuals and Jacobian
  Eigen::MatrixXf J_final = Eigen::MatrixXf::Zero(J.rows() + NUM_MAGNETS, J.cols());
  J_final.block(0, 0, J.rows(), J.cols()) = J;

  Eigen::MatrixXf res_finalT = Eigen::MatrixXf::Zero(res.rows(), res.cols() + NUM_MAGNETS);
  res_finalT.block(0, 0, res.rows(), res.cols()) = res;

  // only if ValRel > 0, so there's a penalty
  if (ValRel > 0) {
    for (int i = 0; i < NUM_MAGNETS; i++) {
      // Update the residuals only if needed
      res_finalT(0, i + NUM_SENSORS * 3) = ValRel * (x_0.block(0, 3+i*7, 1, 4).squaredNorm() - 1);

      // Update the Jacobian only if needed, otherwise leave the penalty row as 0
      if (iflag == 2) {
        J_final.block(J.rows() + i, 3+i*7, 1, 4) = 2.0f * ValRel * x_0.block(0, 3+i*7, 1, 4);
      }
    }
  }

  Eigen::MatrixXf res_final = res_finalT.transpose();

  return std::make_tuple(res_final, J_final);
}

// Cost function in lmder1 form
int LM_library::localization_cost_cyl_jac_cmin(void* p, int m, int n, const float* x_0, float* residuals, float* fjac, int ldfjac, int iflag) {
  OptAndInstance* optAndInstance = static_cast<OptAndInstance*>(p);
  LM_library* instance = optAndInstance->instance;
  OptimizationData* optData = optAndInstance->optData;

  //bool ConsiderDist = optData->ConsiderDist;

  Eigen::MatrixXf x0 = Eigen::MatrixXf::Zero(1, n);
  for (int i = 0; i < n; i++) {
    x0(0, i) = x_0[i];
  }

  Eigen::MatrixXf res, J;
  std::tie(res, J) = instance->localization_cost_cyl_jac(x0, optData->B_field, optData->sPos, optData->sensor_2_ignore, optData->size, optData->ValRel, optData->ConsiderDist, iflag);

  // Update only the residuals (fvec)
  if (iflag == 1) {
    float R_pow2 = instance->determination_coeff(res, optData->B_field);
    if (R_pow2 > 0.9f) {
      for (int i = 0; i < m; i++) {
        residuals[i] = res(i, 0);  // Only one column in the residual matrix
      }
    }

    return 0;
  }

  // Update only fjac
  if(iflag == 2) {
    for (int i = 0; i < NUM_MAGNETS; i++) {
      for (int j = 0; j < m*n; j++) {
        fjac[j] = J(j%m, j/m); // to fill by columns, not rows
      }
    }

    return 0;
  }

  return 0;
}

// Function to call in main
void LM_library::localize_magnets_cyl_lmder(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist) {
  OptAndInstance optAndInstance;
  optAndInstance.instance = this;
  optAndInstance.optData = new OptimizationData;
  optAndInstance.optData->sPos = sPos;
  optAndInstance.optData->B_field = B_field;
  optAndInstance.optData->sensor_2_ignore = sensor_2_ignore;
  optAndInstance.optData->size = size;
  optAndInstance.optData->ValRel = ValRel;
  optAndInstance.optData->ConsiderDist = ConsiderDist;

  if (ConsiderDist)
    lmder1(localization_cost_cyl_jac_cmin, &optAndInstance, m, n+3, x0, residuals, fjac_dist, m, Tol, ipvt_dist, Wa_lmder_dist, Lwa_lmder_dist);
  else
    lmder1(localization_cost_cyl_jac_cmin, &optAndInstance, m, n, x0, residuals, fjac, m, Tol, ipvt, Wa_lmder, Lwa_lmder);

  // deallocation
  delete optAndInstance.optData;

  // normalization
  float norm = 0.0f;
  for (uint8_t j = 0; j < NUM_MAGNETS; j++) {
    norm = 0.0f;
    for (uint8_t i = 3; i <= 6; i++) {
      norm += x0[i + 7*j] * x0[i + 7*j];
    }
    norm = std::sqrt(norm);
    for (int i = 3; i <= 6; ++i) {
      x0[i + 7*j] /= norm;
    }
  }
}

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> LM_library::cyl_model_jacf_opt(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, int iflag, float BulirschTol, bool ConsiderDist){
  
  Eigen::MatrixXf Best = Eigen::MatrixXf::Zero(NUM_SENSORS, 3);
  Eigen::MatrixXf Grad;
  if (ConsiderDist)
    Grad = Eigen::MatrixXf::Zero(NUM_SENSORS*3, NUM_MAGNETS*7+3);
  else
    Grad = Eigen::MatrixXf::Zero(NUM_SENSORS*3, NUM_MAGNETS*7);
  Eigen::VectorXf J_par = Eigen::VectorXf::Zero(9);
  Eigen::VectorXf J_bot = Eigen::VectorXf::Zero(9);

  for (uint8_t j = 0; j < NUM_MAGNETS; j++){

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      // extract the pose of the j-th magnet
      // extract the position of the j-th magnet
      Eigen::Vector3f r_j = x_0.block(0, j*7, 1, 3).transpose();  // [m]
      // extract the orientation of the j-th magnet and normalize
      Eigen::VectorXf q_j = x_0.block(0, 3+j*7, 1, 4).transpose(); // [-]
      q_j /= q_j.norm();
      // extract magnet unit vectors w.r.t. the Global Reference frame
      // e_parallel is the unit vector aligned with cylinder axis
      Eigen::Vector3f e_par (   2*(q_j(1)*q_j(3)+q_j(0)*q_j(2)), \
                                2*(q_j(2)*q_j(3)-q_j(1)*q_j(0)), \
                              1-2*(q_j(1)*q_j(1)+q_j(2)*q_j(2)) );        
      // e_bot is a unit vector perpendicular to cylinder axis which is
      // parallel to the diametric component of magnetization (if present)
      Eigen::Vector3f e_bot ( 1-2*(q_j(2)*q_j(2)+q_j(3)*q_j(3)), \
                                2*(q_j(1)*q_j(2)+q_j(0)*q_j(3)), \
                                2*(q_j(1)*q_j(3)-q_j(0)*q_j(2)) );
      // projection of the magnetization components along the magnet unit
      // vectors/axes, w.r.t. the global reference frame.
      // N.B.:
      // M_star(3) is M_parallel or axial component of the magnetization
      // M_star(1) is M_bot or diametric component of the magnetization
      Eigen::Vector3f M_par = M_starf[2]*e_par; // [A/m]
      Eigen::Vector3f M_bot = M_starf[0]*e_bot; // [A/m]
      // recast matrix of vector distances normalized by R_j
      Eigen::Vector3f r_ij {(sPos(i,0) - r_j(0))/R_j, (sPos(i,1) - r_j(1))/R_j, (sPos(i,2) - r_j(2))/R_j};

      Eigen::Vector3f nu = r_ij.cross(e_par);
      float rho = nu.norm();                // radial component [-]
      nu = nu/rho;                          // nu normalization
      float z   = r_ij.dot(e_par);          // axial component  [-]

      // definition extension close to axis
      if(rho <= AXIS_TH){
        rho = 0;
        nu(0) = 0; nu(1) = 0; nu(2) = 0;
      }

      // auxiliary vectors for projection and reflection of local cylindrical components
      Eigen::Vector3f v = (r_ij.dot(M_par - M_bot)) * e_par - M_starf[2] * r_ij;
      Eigen::Vector3f u = rho * (M_bot - 2 * (M_bot.dot(nu)) * nu);

      // adimensional semi-length
      float l = (L_j/2)/R_j;

      // compute auxiliary expressions
      float zP      = z + l;
      float zM      = z - l;
      float zP2     = zP*zP;
      float zM2     = zM*zM;
      float onePlusRho  = 1+rho;
      float oneMinusRho = 1-rho;

      float dP2     = onePlusRho*onePlusRho + zP2;
      float dM2     = onePlusRho*onePlusRho + zM2;

      float dP      = sqrtf( dP2 );
      float dM      = sqrtf( dM2 );

      float dP3_1   = 1/(dP2*dP);
      float dM3_1   = 1/(dM2*dM);

      float kP      = sqrtf( (4*rho) / dP2 );
      float kM      = sqrtf( (4*rho) / dM2 );
      float kcP2    = 1 - kP*kP;
      float kcM2    = 1 - kM*kM;

      float kcP     = sqrtf( kcP2 );
      float kcM     = sqrtf( kcM2 );
      float sigmaP  = sqrtf( (zP2)/( oneMinusRho*oneMinusRho + zP2 ) );
      float sigmaM  = sqrtf( (zM2)/( oneMinusRho*oneMinusRho + zM2 ) );

      // Field computation in coordinate free representation
      float fL = fLambdaLocalization(rho, z, l, kP, kM, sigmaP, sigmaM, BulirschTol); 

      // To minimize CEL calls
      float bulirschP_A = BulirschCEL(kcP, 1, 1, 0, BulirschTol);
      float bulirschP_B = BulirschCEL(kcP, 1, 0, 1, BulirschTol);
      float bulirschM_A = BulirschCEL(kcM, 1, 1, 0, BulirschTol);
      float bulirschM_B = BulirschCEL(kcM, 1, 0, 1, BulirschTol);

      // float bulirschP = BulirschCEL(kcP, 1, 1, 1, BulirschTol);
      // float bulirschM = BulirschCEL(kcM, 1, 1, 1, BulirschTol);
      float zP_dP = zP / dP;
      float zM_dM = zM / dM;
      float f1 = 0.25f * (zP_dP * (bulirschP_A + bulirschP_B) \
                        - zM_dM * (bulirschM_A + bulirschM_B) + fL);

      float f2 = 0;
      if (rho > 5E-4) {
        // bulirschP = BulirschCEL(kcP, 1, 1-2*rho, 1+2*rho, BulirschTol);
        // bulirschM = BulirschCEL(kcM, 1, 1-2*rho, 1+2*rho, BulirschTol);
        f2 = 0.25f/powf(rho,3) * (zP_dP * ( (oneMinusRho-rho)*bulirschP_A + (onePlusRho+rho)*bulirschP_B ) \
                                - zM_dM * ( (oneMinusRho-rho)*bulirschM_A + (onePlusRho+rho)*bulirschM_B ) - fL);
      }
      else{
        f2 = rho*3*PI/32 * ( zM/powf(zM2 + 1, 2.5f) \
                           - zP/powf(zP2 + 1, 2.5f) );
      } 
                        
      float kcP1   = 2 * sqrtf(kcP) / (1 + kcP);
      float kcP2_2 = 2 / powf(1 + kcP, 3);
      float kcM1   = 2 * sqrtf(kcM) / (1 + kcM);
      float kcM2_2 = 2 / powf(1 + kcM, 3);
    
      float bulirschP = BulirschCEL(kcP1, 1, 0, kcP2_2, BulirschTol);
      float bulirschM = BulirschCEL(kcM1, 1, 0, kcM2_2, BulirschTol);
    
      float f3 = 4 * ( dP3_1 * bulirschP \
                     - dM3_1 * bulirschM );

      if (iflag == 1)
        Best.row(i) += (f1 * (2*M_par - M_bot) + f2 * u + f3 * v).transpose();

      if (iflag == 2) {
        // Jacobian    
        Eigen::Vector3f e_aux = e_par.cross(e_bot);
        float c = r_ij.dot(e_bot) / rho;
        float s = r_ij.dot(e_aux) / rho;

        if(rho <= AXIS_TH){
          c = 0;
          s = 0;
        }

        float kcP2_1 = 1 / kcP2;
        float kcM2_1 = 1 / kcM2;
      
        // bulirschP = BulirschCEL(kcP, 1, kcP2, -1, BulirschTol);
        // bulirschM = BulirschCEL(kcM, 1, kcM2, -1, BulirschTol);
      
        float f4 = zP*dP3_1 * ( kcP2_1*bulirschP_A - bulirschP_B ) \
                 - zM*dM3_1 * ( kcM2_1*bulirschM_A - bulirschM_B );

        float rhoP = oneMinusRho * kcP2_1;
        float rhoM = oneMinusRho * kcM2_1;
      
        // bulirschP = BulirschCEL(kcP, 1, rhoP, 1 + rho, BulirschTol);
        // bulirschM = BulirschCEL(kcM, 1, rhoM, 1 + rho, BulirschTol);
      
        float f5 = dP3_1 * ( rhoP*bulirschP_A + onePlusRho*bulirschP_B ) \
                 - dM3_1 * ( rhoM*bulirschM_A + onePlusRho*bulirschM_B );

        // Computation of auxiliary expressions for the gradient in local coordinates

        float c2 = c*c;
        float s2 = 1-c2;
        float cs = c*s;

        float g  = 2*f3 - f5;
        float g1 = c2*g;
        float g2 = cs*g;
        float g3 = s2*g;
        float g4 = c*f4;
        float g5 = s*f4;
        float g6 = 8*f2 + f4;
        // Calculate J_par
        J_par(0) = g1 - f3;
        J_par(1) = g2;
        J_par(2) = g4;
        J_par(3) = J_par(1);
        J_par(4) = g3 - f3;
        J_par(5) = g5;
        J_par(6) = J_par(2);
        J_par(7) = J_par(5);
        J_par(8) = f5;

        // Calculate J_bot
        J_bot(0) = c * (6*f2 - c2*g6);
        J_bot(1) = s * (2*f2 - c2*g6);
        J_bot(2) = J_par(0);
        J_bot(3) = J_bot(1);
        J_bot(4) = c * (2*f2 - s2*g6);
        J_bot(5) = g2;
        J_bot(6) = J_par(0);
        J_bot(7) = J_bot(5);
        J_bot(8) = J_par(2);

        if (rho <= AXIS_TH) {
          float gstar = PI/4 * ( 1/powf(1+zP2,1.5f) - 1/powf(1+zM2,1.5f) );

          J_par(0) = -gstar;
          J_par(4) = -gstar;
          J_par(8) = 2*gstar;
          J_bot(2) = -gstar;
          J_bot(6) = -gstar;
        }

        Eigen::Matrix3f grad_rot;
        grad_rot.col(0) = e_bot;
        grad_rot.col(1) = e_aux;
        grad_rot.col(2) = e_par;

        Eigen::Matrix3f G;
        G.col(0) = (M_starf[2] * J_par.block(0, 0, 3, 1) + M_starf[0] * J_bot.block(0, 0, 3, 1))/R_j;
        G.col(1) = (M_starf[2] * J_par.block(3, 0, 3, 1) + M_starf[0] * J_bot.block(3, 0, 3, 1))/R_j;
        G.col(2) = (M_starf[2] * J_par.block(6, 0, 3, 1) + M_starf[0] * J_bot.block(6, 0, 3, 1))/R_j;

        // Cartesian coordinates of the magnetic field
        Grad.block(i*3, j*7, 3, 3) = Kcyl * grad_rot * G * grad_rot.transpose();

        float f1_drho    = f4 / 2;
        float f1_dz      = f5 / 2;
        float f2_drho    = -1 / (2*rho)*(f4 + 6*f2);
        float f2_dz      = g / (2*rho);
        float f3_drho    = -2*f2_dz;
        float f3_dz      = -f4 / rho;

        if (rho <= AXIS_TH) {
          f2_drho = -3*PI/32 * ( zP/powf(1+zP2,2.5f) - zM/powf(1+zM2, 2.5f) );
          f2_dz   = 0;
          f3_drho = 0;
          f3_dz   = 8*f2_drho;
        }

        Eigen::Vector3f de_par_dqc = Eigen::VectorXf::Zero(3);
        Eigen::Vector3f de_bot_dqc = Eigen::VectorXf::Zero(3);
        Eigen::Vector3f dz_dqc_vec = Eigen::VectorXf::Zero(3);
        float dz_dqc = 0;
        Eigen::Vector3f drho_dqc_vec = Eigen::VectorXf::Zero(3);
        float drho_dqc = 0;

        for (int p = 0; p < 4; p++) {
          switch (p) {
            case 0:
              de_par_dqc(0) = q_j(2);
              de_par_dqc(1) = -q_j(1);
              de_par_dqc(2) = 0;
              de_par_dqc *= 2;

              de_bot_dqc(0) = 0;
              de_bot_dqc(1) = q_j(3);
              de_bot_dqc(2) = -q_j(2);
              de_bot_dqc *= 2;

              // dz_dqc_vec(0) = q_j(2);
              // dz_dqc_vec(1) = -q_j(1);
              // dz_dqc_vec(2) = 0;
              dz_dqc = r_ij.dot(de_par_dqc);

              drho_dqc_vec(0) = q_j(1) * r_ij(2);
              drho_dqc_vec(1) = q_j(2) * r_ij(2);
              drho_dqc_vec(2) = -(q_j(1)*r_ij(0) + q_j(2)*r_ij(1));
              drho_dqc = 2 * drho_dqc_vec.dot(nu);
              break;
            case 1:
              de_par_dqc(0) = q_j(3);
              de_par_dqc(1) = -q_j(0);
              de_par_dqc(2) = -2*q_j(1);
              de_par_dqc *= 2;

              de_bot_dqc(0) = 0;
              de_bot_dqc(1) = q_j(2);
              de_bot_dqc(2) = q_j(3);
              de_bot_dqc *= 2;

              // dz_dqc_vec(0) = q_j(3);
              // dz_dqc_vec(1) = -q_j(0);
              // dz_dqc_vec(2) = -2*q_j(1);
              dz_dqc = r_ij.dot(de_par_dqc);

              drho_dqc_vec(0) = -2*q_j(1)*r_ij(1) + q_j(0)*r_ij(2);
              drho_dqc_vec(1) = 2*q_j(1)*r_ij(0) + q_j(3)*r_ij(2);
              drho_dqc_vec(2) = -(q_j(0)*r_ij(0) + q_j(3)*r_ij(1));
              drho_dqc = 2 * drho_dqc_vec.dot(nu);
              break;
            case 2:
              de_par_dqc(0) = q_j(0);
              de_par_dqc(1) = q_j(3);
              de_par_dqc(2) = -2*q_j(2);
              de_par_dqc *= 2;

              de_bot_dqc(0) = -2*q_j(2);
              de_bot_dqc(1) = q_j(1);
              de_bot_dqc(2) = -q_j(0);
              de_bot_dqc *= 2;

              // dz_dqc_vec(0) = q_j(0);
              // dz_dqc_vec(1) = q_j(3);
              // dz_dqc_vec(2) = -2*q_j(2);
              dz_dqc = r_ij.dot(de_par_dqc);

              drho_dqc_vec(0) = -(2*q_j(2)*r_ij(1) + q_j(3)*r_ij(2));
              drho_dqc_vec(1) = 2*q_j(2)*r_ij(0) + q_j(0)*r_ij(2);
              drho_dqc_vec(2) = q_j(3)*r_ij(0) - q_j(0)*r_ij(1);
              drho_dqc = 2 * drho_dqc_vec.dot(nu);
              break;
            case 3:
              de_par_dqc(0) = q_j(1);
              de_par_dqc(1) = q_j(2);
              de_par_dqc(2) = 0;
              de_par_dqc *= 2;

              de_bot_dqc(0) = -2*q_j(3);
              de_bot_dqc(1) = q_j(0);
              de_bot_dqc(2) = q_j(1);
              de_bot_dqc *= 2;

              // dz_dqc_vec(0) = q_j(1);
              // dz_dqc_vec(1) = q_j(2);
              // dz_dqc_vec(2) = 0;
              dz_dqc = r_ij.dot(de_par_dqc);

              drho_dqc_vec(0) = -q_j(2) * r_ij(2);
              drho_dqc_vec(1) = q_j(1) * r_ij(2);
              drho_dqc_vec(2) = q_j(2)*r_ij(0) - q_j(1)*r_ij(1);
              drho_dqc = 2 * drho_dqc_vec.dot(nu);
              break;
            default:
              break;
          }

          Eigen::Vector3f dv_dep1, dv_dep2, dv_dep3, pXe_bot, nuXp;
          Eigen::Vector3f dv_deb1, dv_deb2, dv_deb3, du_dep1, du_dep2, du_dep3, du_deb1, du_deb2, du_deb3;

          float r_dot_Msub = r_ij.dot(M_par-M_bot);

          dv_dep1(0) = r_ij(0)*M_par(0)+r_dot_Msub;
          dv_dep1(1) = r_ij(0)*M_par(1);
          dv_dep1(2) = r_ij(0)*M_par(2);

          dv_dep2(0) = r_ij(1)*M_par(0);
          dv_dep2(1) = r_ij(1)*M_par(1)+r_dot_Msub;
          dv_dep2(2) = r_ij(1)*M_par(2);

          dv_dep3(0) = r_ij(2)*M_par(0);
          dv_dep3(1) = r_ij(2)*M_par(1);
          dv_dep3(2) = r_ij(2)*M_par(2)+r_dot_Msub;

          dv_deb1(0) = -r_ij(0)*M_starf[0]*e_par(0);
          dv_deb1(1) = -r_ij(0)*M_starf[0]*e_par(1);
          dv_deb1(2) = -r_ij(0)*M_starf[0]*e_par(2);

          dv_deb2(0) = -r_ij(1)*M_starf[0]*e_par(0); 
          dv_deb2(1) = -r_ij(1)*M_starf[0]*e_par(1);
          dv_deb2(2) = -r_ij(1)*M_starf[0]*e_par(2);

          dv_deb3(0) = -r_ij(2)*M_starf[0]*e_par(0);
          dv_deb3(1) = -r_ij(2)*M_starf[0]*e_par(1);
          dv_deb3(2) = -r_ij(2)*M_starf[0]*e_par(2);
                  
          pXe_bot = r_ij.cross(e_bot);
          nuXp    = nu.cross(r_ij);

          float nu_dot_ebot = nu.dot(e_bot);
          float du_ep_aux1 = e_bot(0)+2*( nu_dot_ebot * nu(0) );
          float du_ep_aux2 = e_bot(1)+2*( nu_dot_ebot * nu(1) );
          float du_ep_aux3 = e_bot(2)+2*( nu_dot_ebot * nu(2) );

          du_dep1(0) = M_starf[0] * ( nuXp(0) * du_ep_aux1 + 2 *   pXe_bot(0) * nu(0) );
          du_dep1(1) = M_starf[0] * ( nuXp(0) * du_ep_aux2 + 2 * ( pXe_bot(0) * nu(1) + nu_dot_ebot*(-r_ij(2)) ) );
          du_dep1(2) = M_starf[0] * ( nuXp(0) * du_ep_aux3 + 2 * ( pXe_bot(0) * nu(2) + nu_dot_ebot*(+r_ij(1)) ) );

          du_dep2(0) = M_starf[0] * ( nuXp(1) * du_ep_aux1 + 2 * ( pXe_bot(1) * nu(0) + nu_dot_ebot*(+r_ij(2)) ) );
          du_dep2(1) = M_starf[0] * ( nuXp(1) * du_ep_aux2 + 2 *   pXe_bot(1) * nu(1) );
          du_dep2(2) = M_starf[0] * ( nuXp(1) * du_ep_aux3 + 2 * ( pXe_bot(1) * nu(2) + nu_dot_ebot*(-r_ij(0)) ) );

          du_dep3(0) = M_starf[0] * ( nuXp(2) * du_ep_aux1 + 2 * ( pXe_bot(2) * nu(0) + nu_dot_ebot*(-r_ij(1)) ) );
          du_dep3(1) = M_starf[0] * ( nuXp(2) * du_ep_aux2 + 2 * ( pXe_bot(2) * nu(1) + nu_dot_ebot*(+r_ij(0)) ) );
          du_dep3(2) = M_starf[0] * ( nuXp(2) * du_ep_aux3 + 2 *   pXe_bot(2) * nu(2) );


          float Mb_rho = -2*M_starf[0]*rho;
          du_deb1(0) = Mb_rho * (nu(0)*nu(0)-0.5f);
          du_deb1(1) = Mb_rho * (nu(0)*nu(1));
          du_deb1(2) = Mb_rho * (nu(0)*nu(2));

          du_deb2(0) = Mb_rho * (nu(1)*nu(0));
          du_deb2(1) = Mb_rho * (nu(1)*nu(1)-0.5f);
          du_deb2(2) = Mb_rho * (nu(1)*nu(2));

          du_deb3(0) = Mb_rho * (nu(2)*nu(0));
          du_deb3(1) = Mb_rho * (nu(2)*nu(1));
          du_deb3(2) = Mb_rho * (nu(2)*nu(2)-0.5f);

          Eigen::Vector3f auxD1, auxD2, dv_dqc, du_dqc;
          auxD1 = 2 * M_par - M_bot;
          auxD2 = 2 * M_starf[2] * de_par_dqc - M_starf[0] * de_bot_dqc;
          dv_dqc = dv_dep1 * de_par_dqc(0) + dv_dep2 * de_par_dqc(1) + dv_dep3 * de_par_dqc(2) + \
                   dv_deb1 * de_bot_dqc(0) + dv_deb2 * de_bot_dqc(1) + dv_deb3 * de_bot_dqc(2);
          du_dqc = du_dep1 * de_par_dqc(0) + du_dep2 * de_par_dqc(1) + du_dep3 * de_par_dqc(2) + \
                   du_deb1 * de_bot_dqc(0) + du_deb2 * de_bot_dqc(1) + du_deb3 * de_bot_dqc(2);

          float df1_dqc = 0, df2_dqc = 0, df3_dqc = 0;
          df1_dqc = f1_drho * drho_dqc + f1_dz * dz_dqc;
          df2_dqc = f2_drho * drho_dqc + f2_dz * dz_dqc;
          df3_dqc = f3_drho * drho_dqc + f3_dz * dz_dqc;

          Grad.block(i*3, j*(n/NUM_MAGNETS)+3+p, 3, 1) = -Kcyl * (df1_dqc * auxD1 + f1 * auxD2 + df3_dqc * v + f3 * dv_dqc + df2_dqc * u + f2 * du_dqc);
        }
      }

      if (j == NUM_MAGNETS-1) {
        if (ConsiderDist) {
          Grad.block(i*3, Grad.cols()-3, 3, 3) = -Eigen::MatrixXf::Identity(3, 3);
        }
      }
    } 
  }

  if (iflag == 1)
    Best *= Kcyl;

  Eigen::MatrixXf Best_transpose = Best.transpose();
  Eigen::Map<Eigen::MatrixXf> B_estT(Best_transpose.data(), NUM_SENSORS * 3, Best_transpose.size() / (NUM_SENSORS * 3));
  Eigen::MatrixXf Best_final = B_estT.transpose();

  return std::make_tuple(Best_final, Grad);

}
